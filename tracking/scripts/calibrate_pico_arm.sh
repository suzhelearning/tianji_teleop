#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

profile_root="$(cd "$repo_root/.." && pwd -P)"
profile_python="${TIANJI_PYTHON:-$profile_root/.venv/bin/python}"
calibration_python="$profile_python"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"

last_operation=""
calibration_verbose="${PICO_CALIBRATION_VERBOSE:-0}"
calibration_user=""
help_requested=0
filtered_arguments=()
while (($#)); do
  case "$1" in
    --user)
      if [[ -n "$calibration_user" ]]; then
        echo "Duplicate option: --user" >&2
        exit 2
      fi
      if (($# < 2)) || [[ -z "$2" || "$2" == -* ]]; then
        echo "--user requires a person name." >&2
        exit 2
      fi
      calibration_user="$2"
      shift 2
      ;;
    --verbose) calibration_verbose="1"; shift ;;
    -h|--help) help_requested=1; shift ;;
    *) filtered_arguments+=("$1"); shift ;;
  esac
done
set -- "${filtered_arguments[@]}"


usage() {
  cat >&2 <<EOF
用法:
  $0                         交互式菜单
  $0 <left|right> <tcp|wrist|geometry|all>
  $0 status                  查看左右侧 artifact 状态
  $0 [以上参数] --verbose     显示完整 validator/Gate JSON
  $0 [以上参数] --user NAME   保存到指定人员；status 只读取已发布版本
  geometry 可选: --start-mode space|auto --domain ID --countdown-s N --duration-scale SCALE

默认 ROS_DOMAIN_ID=$ROS_DOMAIN_ID，ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY
EOF
}

if ((help_requested)); then
  usage
  exit 0
fi

validate_geometry_arguments() {
  local option value
  while (($#)); do
    option="$1"
    case "$option" in
      --side|--tcp-artifact|--wrist-pivot-artifact|--output-dir)
        if [[ -n "$calibration_user" || -n "${PICO_CALIBRATION_DIR:-}" ]]; then
          echo "Named calibration does not allow artifact, output or side overrides: $option" >&2
          return 2
        fi
        ;;
      --start-mode|--domain|--countdown-s|--duration-scale) ;;
      *) echo "Unknown geometry argument: $option" >&2; return 2 ;;
    esac
    if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then
      echo "$option requires a value." >&2
      return 2
    fi
    value="$2"
    case "$option" in
      --side) [[ "$value" == left || "$value" == right ]] || return 2 ;;
      --start-mode) [[ "$value" == space || "$value" == auto ]] || return 2 ;;
      --domain) [[ "$value" =~ ^[0-9]+$ ]] || return 2 ;;
      --countdown-s) [[ "$value" =~ ^[+-]?[0-9]+$ ]] || return 2 ;;
      --duration-scale)
        if [[ ! "$value" =~ ^[+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ||
              ! "${value%%[eE]*}" =~ [1-9] ]]; then
          echo "--duration-scale must be positive." >&2
          return 2
        fi
        ;;
    esac
    shift 2
  done
}

# Reject malformed commands before the lifecycle helper can create a draft.
if (($# == 0)); then
  if [[ ! -t 0 ]]; then
    echo "无交互终端；请使用: $0 <left|right> <tcp|wrist|geometry|all> [--user NAME]" >&2
    exit 2
  fi
elif [[ "$1" == status ]]; then
  (($# == 1)) || { usage; exit 2; }
else
  [[ "$1" == left || "$1" == right ]] || { usage; exit 2; }
  case "${2:-}" in
    tcp|wrist|all)
      (($# == 2)) || { echo "$2 不接受额外参数" >&2; exit 2; }
      ;;
    geometry) validate_geometry_arguments "${@:3}" || { usage; exit 2; } ;;
    *) usage; exit 2 ;;
  esac
fi

source "$repo_root/scripts/environment.sh" --python-only

if [[ -n "$calibration_user" ]]; then
  if [[ "${1:-}" == status ]]; then
    if ! PICO_CALIBRATION_DIR="$(env -u PICO_CALIBRATION_DIR "$profile_python" \
        "$profile_root/teleop_profile.py" --user "$calibration_user" --component pico)"; then
      echo "无法读取 $calibration_user 的已发布 PICO 标定；status 不会创建或修改草稿。" >&2
      exit 2
    fi
    export PICO_CALIBRATION_DIR
    calibration_python="$profile_python"
  else
    export PICO_CALIBRATION_VERBOSE="$calibration_verbose"
    exec "$profile_python" "$profile_root/teleop_profile.py" \
      --user "$calibration_user" --calibrate -- \
      bash "$repo_root/scripts/calibrate_pico_arm.sh" "$@"
  fi
fi

calibration_dir="${PICO_CALIBRATION_DIR:-$HOME/.config/pico_tracker}"
recordings_dir="$repo_root/recordings"
if [[ -n "${PICO_CALIBRATION_DIR:-}" && "${1:-}" != status ]]; then
  if [[ "$PICO_CALIBRATION_DIR" != /* || "${PICO_CALIBRATION_RECORDINGS_DIR:-}" != /* ]]; then
    echo "Named calibration requires absolute PICO_CALIBRATION_DIR and PICO_CALIBRATION_RECORDINGS_DIR." >&2
    exit 2
  fi
  recordings_dir="$PICO_CALIBRATION_RECORDINGS_DIR"
fi

side_path() {
  local side="$1"
  local kind="$2"
  case "$kind" in
    tcp) printf '%s\n' "$calibration_dir/pico_${side}_palm_tcp.yaml" ;;
    wrist) printf '%s\n' "$calibration_dir/pico_${side}_wrist_pivot.yaml" ;;
    geometry) printf '%s\n' "$calibration_dir/pico_${side}_arm_geometry.yaml" ;;
    *) return 2 ;;
  esac
}

validate_artifact() {
  local path="$1"
  local kind="$2"
  local side="$3"
  local summary
  summary="$("$calibration_python" "$repo_root/src/pico_bridge/scripts/pico_calibration_artifact.py" validate \
    --path "$path" \
    --kind "$kind" \
    --side "$side" \
    --tcp-path "$(side_path "$side" tcp)" \
    --wrist-path "$(side_path "$side" wrist)")" || return
  if [[ "$calibration_verbose" == "1" ]]; then
    echo "$summary"
  fi
}

require_artifact() {
  local side="$1"
  local kind="$2"
  local path
  path="$(side_path "$side" "$kind")"
  if ! validate_artifact "$path" "$kind" "$side"; then
    echo "缺少或无效的 $side $kind artifact: $path" >&2
    return 2
  fi
}

activate_calibration_candidate() {
  local side="$1"
  local kind="$2"
  local candidate="$3"
  local active
  active="$(side_path "$side" "$kind")"
  local summary
  summary="$("$calibration_python" "$repo_root/src/pico_bridge/scripts/pico_calibration_artifact.py" activate \
    --path "$candidate" \
    --active "$active" \
    --kind "$kind" \
    --side "$side" \
    --tcp-path "$(side_path "$side" tcp)" \
    --wrist-path "$(side_path "$side" wrist)")" || return
  if [[ "$calibration_verbose" == "1" ]]; then
    echo "$summary"
  fi
}

print_artifact_result() {
  local side="$1"
  local kind="$2"
  local path="$3"
  local session_dir="${4:-}"
  "$calibration_python" - "$side" "$kind" "$path" "$session_dir" <<'PY'
import math
from pathlib import Path
import sys
import yaml

side, kind, path_text, session_text = sys.argv[1:]
path = Path(path_text).expanduser()
document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
labels = {"left": "左侧", "right": "右侧"}
if kind == "tcp":
    value = document.get("translation_m", [0.0, 0.0, 0.0])
    if isinstance(value, dict):
        value = [value.get(axis, 0.0) for axis in "xyz"]
    rms = float(document.get("quality", {}).get("position_rms_m", math.nan))
    print(f"✓ {labels[side]} TCP 标定成功")
    print("  手柄到掌心: [" + ", ".join(f"{float(item):.4f}" for item in value) + "] m")
    print(f"  位置误差 RMS: {rms:.4f} m")
elif kind == "wrist":
    value = document.get("wrist_to_palm_m", {})
    vector = [float(value.get(axis, 0.0)) for axis in "xyz"]
    distance = math.sqrt(sum(item * item for item in vector))
    rms = float(document.get("quality", {}).get("position_rms_m", math.nan))
    print(f"✓ {labels[side]} 掌心到手腕标定成功")
    print(f"  掌心到手腕距离: {distance:.4f} m")
    print(f"  腕部局部向量: [{vector[0]:.4f}, {vector[1]:.4f}, {vector[2]:.4f}] m")
    print(f"  位置误差 RMS: {rms:.4f} m")
else:
    print(f"✓ {labels[side]} 上臂/前臂骨长标定成功")
    print(f"  上臂长度: {float(document['upper_arm_length_m']):.3f} m")
    print(f"  前臂长度: {float(document['forearm_length_m']):.3f} m")
print(f"  标定结果文件: {path}")
if session_text:
    print(f"  本次采集目录: {session_text}")
PY
}

print_invalid_artifact_help() {
  local kind="$1"
  local path="$2"
  "$calibration_python" - "$kind" "$path" <<'PY'
import math
from pathlib import Path
import sys
import yaml

kind, path_text = sys.argv[1:]
path = Path(path_text).expanduser()
if not path.exists():
    print("    原因: 还没有进行这项标定")
    raise SystemExit(0)
try:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
except Exception:
    print("    原因: 标定文件无法读取，请重新标定")
    raise SystemExit(0)
if kind == "tcp":
    quality = document.get("quality", {})
    if "sample_matrix_rank" not in quality:
        print("    原因: 旧 TCP 文件缺少动作覆盖证据，请重新进行 TCP 标定")
    else:
        print("    原因: TCP 质量检查未通过，请增大四次采样之间的手柄姿态变化")
elif kind == "wrist":
    value = document.get("wrist_to_palm_m", {})
    try:
        vector = [float(value.get(axis, 0.0)) for axis in "xyz"]
        distance = math.sqrt(sum(item * item for item in vector))
    except (AttributeError, TypeError, ValueError):
        distance = math.nan
    if math.isfinite(distance) and distance < 0.01:
        print(f"    原因: 掌心到手腕距离过短（{distance:.4f} m）")
        print("    建议: 保持手腕不动，让手掌充分左右转动后重新进行腕部标定")
    else:
        print("    原因: 腕部标定质量检查未通过，请重新进行腕部标定")
else:
    print("    原因: 骨长标定尚未通过全部动作一致性检查")
PY
}

run_tcp() {
  local side="$1"
  local output session_dir candidate
  output="$(side_path "$side" tcp)"
  session_dir="$recordings_dir/pico_${side}_tcp_$(date +%Y%m%d_%H%M%S)_$$"
  candidate="$session_dir/pico_${side}_palm_tcp_candidate.yaml"
  echo
  echo "开始 $side TCP 标定："
  echo "  - 手套和手柄必须固定牢靠"
  echo "  - 第 1～4 次空格只标定 TCP 位置"
  echo "  - 这四次都让掌心参考点保持在同一位置，只改变手柄朝向"
  echo "  - 朝向变化要明显，并覆盖至少两个不同旋转方向"
  echo "  - 位置通过后，终端会明确提示进入姿态阶段"
  echo "  - 此时双臂向正前方水平伸直、左右掌心相对"
  echo "  - 摆好后按第 5 次空格启动所选侧 TCP 姿态采集"
  echo "  - 按下后保持当前姿势约 1～2 秒，系统会自动进行多帧平均"
  echo "两个阶段都通过后才保存结果并自动进入下一阶段；q 取消。"
  ros2 run pico_bridge pico_palm_tcp_calibrator \
    --side "$side" --output "$candidate" || return
  if [[ ! -f "$candidate" ]]; then
    echo "TCP 标定未生成 candidate: $candidate" >&2
    return 2
  fi
  activate_calibration_candidate "$side" tcp "$candidate" || return
  echo
  print_artifact_result "$side" tcp "$output" "$session_dir"
}

run_wrist() (
  local side="$1"
  local tcp wrist session_dir candidate publisher_pid
  publisher_pid=""
  cleanup_wrist() {
    local owned_pid="$publisher_pid"
    trap - EXIT INT TERM
    publisher_pid=""
    pico_stop_process_group "$owned_pid"
  }
  trap cleanup_wrist EXIT INT TERM
  tcp="$(side_path "$side" tcp)"
  wrist="$(side_path "$side" wrist)"
  require_artifact "$side" tcp || return
  session_dir="$recordings_dir/pico_${side}_wrist_$(date +%Y%m%d_%H%M%S)_$$"
  candidate="$session_dir/pico_${side}_wrist_pivot_candidate.yaml"
  echo
  echo "开始 $side 掌心到手腕标定："
  echo "  - 保持手腕空间位置不动"
  echo "  - 按空格后约 5 秒内，缓慢并充分地左右转动手掌"
  echo "  - 不要让整个手臂跟着移动，也不要让手套滑动"
  echo "成功后自动进入下一阶段，q 取消。"
  setsid ros2 run pico_bridge pico_palm_tcp_publisher \
    --side "$side" --artifact "$tcp" &
  publisher_pid=$!
  # Do not open the interactive wrist collector when its required palm
  # publisher failed during import/startup.  Otherwise the operator sees a
  # plausible collection prompt that can never receive a sample.
  local startup_attempt
  for ((startup_attempt=0; startup_attempt<8; startup_attempt++)); do
    sleep 0.1
    if ! kill -0 -- "-$publisher_pid" 2>/dev/null; then
      echo >&2
      echo "✗ 临时 TCP publisher 启动失败，腕部标定尚未开始。" >&2
      echo "  已成功的 TCP 标定结果保持不变；请检查上方 publisher 错误后重试腕部标定。" >&2
      return 2
    fi
  done
  if ros2 run pico_bridge pico_palm_wrist_calibrator \
    --side "$side" --sample-count 450 --output "$candidate"; then
    if [[ ! -f "$candidate" ]]; then
      echo "腕部标定未生成 candidate: $candidate" >&2
      return 2
    fi
    activate_calibration_candidate "$side" wrist "$candidate" || return
    echo
    print_artifact_result "$side" wrist "$wrist" "$session_dir"
  else
    local rc=$?
    return "$rc"
  fi
)

run_geometry() {
  local side="$1"
  shift
  local tcp wrist session_dir candidate active
  tcp="$(side_path "$side" tcp)"
  wrist="$(side_path "$side" wrist)"
  require_artifact "$side" tcp || return
  require_artifact "$side" wrist || return
  session_dir="$recordings_dir/pico_${side}_arm_geometry_$(date +%Y%m%d_%H%M%S)_$$"
  candidate="$session_dir/pico_${side}_arm_geometry_candidate.yaml"
  active="$(side_path "$side" geometry)"
  local -a geometry_arguments
  geometry_arguments=(
    --side "$side" \
    --domain "$ROS_DOMAIN_ID" \
    --tcp-artifact "$tcp" \
    --wrist-pivot-artifact "$wrist" \
    --start-mode space \
    --output-dir "$session_dir"
  )
  if [[ "$calibration_verbose" == "1" ]]; then
    geometry_arguments+=(--verbose)
  fi
  geometry_arguments+=("$@")
  if ! bash "$repo_root/src/pico_bridge/scripts/pico_arm_geometry_runtime.sh" \
      "${geometry_arguments[@]}"; then
    # The calibrator prints the actionable Gate reasons and evidence paths.
    # Only add a wrapper-level error when it died before producing evidence.
    if [[ ! -f "$candidate" ]]; then
      echo
      echo "✗ $side 骨长标定进程提前退出，未生成标定候选。" >&2
      echo "  请检查上方运行错误；已有效的 TCP、腕部和骨长配置均未修改。" >&2
    fi
    return 2
  fi
  if ! activate_calibration_candidate "$side" geometry "$candidate"; then
    echo "✗ 骨长 candidate 未通过最终验证，当前有效配置保持不变。" >&2
    return 2
  fi
  echo
  print_artifact_result "$side" geometry "$active" "$session_dir"
}

show_one_status() {
  local side="$1"
  local kind path summary
  echo "[$side]"
  for kind in tcp wrist geometry; do
    path="$(side_path "$side" "$kind")"
    if validate_artifact "$path" "$kind" "$side" >/dev/null 2>&1; then
      print_artifact_result "$side" "$kind" "$path" | sed 's/^/  /'
    else
      echo "  ✗ $kind：尚未完成或文件无效"
      echo "    文件: $path"
      print_invalid_artifact_help "$kind" "$path"
    fi
  done
}

show_status() {
  show_one_status left
  show_one_status right
}

choose_side() {
  local choice
  while true; do
    read -r -p "请选择侧别 [1=左侧, 2=右侧, q=退出]: " choice
    case "$choice" in
      1|left) printf 'left\n'; return 0 ;;
      2|right) printf 'right\n'; return 0 ;;
      q|Q) return 1 ;;
      *) echo "请输入 1、2 或 q。" >&2 ;;
    esac
  done
}

choose_operation() {
  local choice
  while true; do
    read -r -p "请选择标定项目 [1=TCP, 2=掌心到手腕, 3=上臂/前臂骨长, q=退出]: " choice
    case "$choice" in
      1|tcp) printf 'tcp\n'; return 0 ;;
      2|wrist) printf 'wrist\n'; return 0 ;;
      3|geometry|arm) printf 'geometry\n'; return 0 ;;
      q|Q) return 1 ;;
      *) echo "请输入 1、2、3 或 q。" >&2 ;;
    esac
  done
}

run_operation() {
  local side="$1"
  local operation="$2"
  last_operation="$operation"
  case "$operation" in
    tcp) run_tcp "$side" ;;
    wrist) run_wrist "$side" ;;
    geometry) run_geometry "$side" ;;
    *) echo "未知标定项目: $operation" >&2; return 2 ;;
  esac
}

post_geometry_menu() {
  local side="$1"
  local result="$2"
  local choice
  while true; do
    echo
    if ((result == 0)); then
      echo "$side 上臂/前臂骨长标定完成。"
    else
      echo "$side 上臂/前臂骨长标定未通过 Gate；capture 和 candidate 已保留，未激活。"
    fi
    echo "  1) 返回主菜单"
    echo "  2) 重新进行当前骨长标定"
    echo "  3) 退出"
    read -r -p "请选择: " choice
    case "$choice" in
      1) return 0 ;;
      2)
        if run_operation "$side" geometry; then
          result=0
        else
          result=$?
          [[ -z "${PICO_CALIBRATION_DIR:-}" ]] || return "$result"
        fi
        ;;
      3|q|Q) return 10 ;;
      *) echo "请输入 1、2 或 3。" >&2 ;;
    esac
  done
}

run_all() {
  local side="$1"
  echo "新操作人员适配顺序: TCP -> 掌心到手腕 -> 上臂/前臂骨长"
  run_operation "$side" tcp || return
  run_operation "$side" wrist || return
  run_operation "$side" geometry
}

interactive_menu() {
  local menu side operation result
  while true; do
    echo
    echo "PICO 标定菜单 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, localhost=$ROS_LOCALHOST_ONLY)"
    echo "  1) 单项标定"
    echo "  2) 新操作人员完整适配 (TCP -> wrist -> geometry)"
    echo "  3) 查看当前标定状态"
    echo "  4) 退出"
    read -r -p "请选择: " menu
    case "$menu" in
      1)
        if side="$(choose_side)" && operation="$(choose_operation)"; then
          if run_operation "$side" "$operation"; then
            result=0
          else
            result=$?
            echo "标定失败，可修正后重新选择。" >&2
            [[ -z "${PICO_CALIBRATION_DIR:-}" ]] || return "$result"
          fi
          if [[ "$operation" == "geometry" ]]; then
            post_geometry_menu "$side" "$result" || {
              result=$?
              ((result == 10)) && return 0
              return "$result"
            }
          fi
        fi
        ;;
      2)
        if side="$(choose_side)"; then
          if run_all "$side"; then
            result=0
          else
            result=$?
            echo "完整适配中断，可修正后重新选择。" >&2
            [[ -z "${PICO_CALIBRATION_DIR:-}" ]] || return "$result"
          fi
          if [[ "$last_operation" == "geometry" ]]; then
            post_geometry_menu "$side" "$result" || {
              result=$?
              ((result == 10)) && return 0
              return "$result"
            }
          fi
        fi
        ;;
      3) show_status ;;
      4|q|Q) return 0 ;;
      *) echo "请输入 1、2、3 或 4。" >&2 ;;
    esac
  done
}

if [[ "${1:-}" == status ]]; then
  show_status
  exit 0
fi

# Reject missing upstream calibration before requiring a live ROS installation.
case "${2:-}" in
  wrist) require_artifact "$1" tcp || exit $? ;;
  geometry)
    require_artifact "$1" tcp || exit $?
    require_artifact "$1" wrist || exit $?
    ;;
esac

source "$repo_root/scripts/environment.sh"
source "$repo_root/src/pico_bridge/scripts/pico_process_cleanup.sh"


if (($# == 0)); then
  interactive_menu
  exit $?
fi

side="$1"
operation="$2"
shift 2
case "$operation" in
  geometry)
    if run_geometry "$side" "$@"; then
      echo
      echo "骨长标定命令已结束，已返回终端。"
      exit 0
    else
      geometry_rc=$?
      exit "$geometry_rc"
    fi
    ;;
  all) run_all "$side"; exit $? ;;
esac
run_operation "$side" "$operation"
