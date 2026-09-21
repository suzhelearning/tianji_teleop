#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
pico_scripts="$repo_root/src/teleop_inputs/pico_bridge/scripts"
calibration_dir=""
forwarded_arguments=()
while (($#)); do
  case "$1" in
    --calibration-dir)
      if [[ -n "$calibration_dir" ]]; then
        echo "Duplicate option: --calibration-dir" >&2
        exit 2
      fi
      if (($# < 2)) || [[ "$2" != /* ]]; then
        echo "--calibration-dir requires an absolute directory path." >&2
        exit 2
      fi
      calibration_dir="$2"
      shift 2
      ;;
    *)
      forwarded_arguments+=("$1")
      shift
      ;;
  esac
done
if [[ -n "$calibration_dir" ]]; then
  for argument in "${forwarded_arguments[@]}"; do
    case "$argument" in
      --left-tcp|--right-tcp|--left-wrist|--right-wrist|--left-geometry|--right-geometry|--require-left-geometry|--require-right-geometry|--left-tcp=*|--right-tcp=*|--left-wrist=*|--right-wrist=*|--left-geometry=*|--right-geometry=*|--require-left-geometry=*|--require-right-geometry=*)
        echo "--calibration-dir cannot be combined with artifact overrides: $argument" >&2
        exit 2
        ;;
    esac
  done
  if [[ ! -d "$calibration_dir" ]]; then
    echo "Calibration directory does not exist: $calibration_dir" >&2
    exit 2
  fi
  calibration_dir="$(cd -- "$calibration_dir" && pwd -P)"
fi

source "$repo_root/bash/environment.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-120}"
export EXO_REQUESTED_ROS_DOMAIN_ID="${EXO_REQUESTED_ROS_DOMAIN_ID:-$ROS_DOMAIN_ID}"
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS2CLI_DISABLE_DAEMON="${ROS2CLI_DISABLE_DAEMON:-1}"


artifact_dir="${calibration_dir:-$HOME/.config/pico_tracker}"
left_geometry="$artifact_dir/pico_left_arm_geometry.yaml"
right_geometry="$artifact_dir/pico_right_arm_geometry.yaml"
left_tcp="$artifact_dir/pico_left_palm_tcp.yaml"
right_tcp="$artifact_dir/pico_right_palm_tcp.yaml"
left_wrist="$artifact_dir/pico_left_wrist_pivot.yaml"
right_wrist="$artifact_dir/pico_right_wrist_pivot.yaml"

runtime_arguments=(
  --domain "$ROS_DOMAIN_ID"
  --left-tcp "$left_tcp"
  --right-tcp "$right_tcp"
  --left-wrist "$left_wrist"
  --right-wrist "$right_wrist"
)
valid_geometry_count=0
for side in left right; do
  if [[ "$side" == "left" ]]; then
    geometry="$left_geometry"
    tcp="$left_tcp"
    wrist="$left_wrist"
  else
    geometry="$right_geometry"
    tcp="$right_tcp"
    wrist="$right_wrist"
  fi
  if [[ -n "$calibration_dir" ]]; then
    if ! python "$pico_scripts/pico_calibration_artifact.py" \
        validate --path "$geometry" --kind geometry --side "$side" \
        --tcp-path "$tcp" --wrist-path "$wrist" >/dev/null; then
      echo "Invalid $side calibration chain in: $calibration_dir; refusing SMPL baseline fallback." >&2
      exit 2
    fi
    runtime_arguments+=("--${side}-geometry" "$geometry" "--require-${side}-geometry")
    valid_geometry_count=$((valid_geometry_count + 1))
    echo "Validated required $side calibration: $geometry"
  elif python "$pico_scripts/pico_calibration_artifact.py" \
      validate --path "$geometry" --kind geometry --side "$side" \
      --tcp-path "$tcp" --wrist-path "$wrist" >/dev/null 2>&1; then
    runtime_arguments+=("--${side}-geometry" "$geometry" "--require-${side}-geometry")
    valid_geometry_count=$((valid_geometry_count + 1))
    echo "✓ $side 个体骨长已通过严格验证，将启用掌心约束修正"
  else
    echo "⚠ $side 个体骨长或其上游 artifact 未通过严格验证；该侧不加载严格个体骨长，使用运行时 SMPL baseline" >&2
    echo "  如需修正该侧，请运行: ./bash/calibrate_pico_arm.sh $side all" >&2
  fi
done

if ((valid_geometry_count == 0)); then
  echo "至少需要一侧完成有效标定后才能启动 PICO M0 可视化。" >&2
  exit 2
fi


exec "$pico_scripts/start_pico_m0_runtime.sh" \
  "${runtime_arguments[@]}" \
  "${forwarded_arguments[@]}"
