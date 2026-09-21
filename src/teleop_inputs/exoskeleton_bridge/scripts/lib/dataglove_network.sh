#!/usr/bin/env bash

# 仅配置已校验 MAC 的数据手套网卡；source 本文件不会操作网络。
GLOVE_HOST="${DATAGLOVE_HOST:-192.168.7.2}"
GLOVE_PORT="${DATAGLOVE_PORT:-5580}"
GLOVE_INTERFACE="${DATAGLOVE_INTERFACE:-}"
GLOVE_MAC="${DATAGLOVE_MAC:-02:33:80:00:00:01}"
GLOVE_HOST_ADDRESS="${DATAGLOVE_HOST_ADDRESS:-192.168.7.1/24}"
GLOVE_ROUTE_TABLE="${DATAGLOVE_ROUTE_TABLE:-}"
GLOVE_MTU="${DATAGLOVE_MTU:-}"
GLOVE_NETWORK_CHECK_ONLY="${DATAGLOVE_NETWORK_CHECK_ONLY:-0}"
NET_SYS_CLASS="${DATAGLOVE_SYS_CLASS_NET:-/sys/class/net}"
USB_WAIT_TIMEOUT_SECONDS="${DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS:-30}"

_glove_ipv4_number() {
  local address="$1" octet value=0
  local -a octets
  [[ "$address" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  IFS=. read -r -a octets <<< "$address"
  for octet in "${octets[@]}"; do
    [[ "$octet" == 0 || "$octet" != 0* ]] || return 1
    ((10#$octet <= 255)) || return 1
    value=$(((value << 8) | 10#$octet))
  done
  # 不能把回环、未指定、多播或保留地址当作直连手套。
  ((octets[0] > 0 && octets[0] < 224 && octets[0] != 127)) || return 1
  printf '%s\n' "$value"
}

validate_glove_network_settings() {
  local host source prefix mask network broadcast
  if [[ -n "$GLOVE_MTU" ]] &&
      { ! [[ "$GLOVE_MTU" =~ ^[1-9][0-9]{1,3}$ ]] || ((GLOVE_MTU < 68 || GLOVE_MTU > 9000)); }; then
    echo "错误：DATAGLOVE_MTU 必须是 68..9000 的整数" >&2
    return 2
  fi
  if ! [[ "$GLOVE_MAC" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
    echo "错误：DATAGLOVE_MAC 不是合法 MAC：$GLOVE_MAC" >&2
    return 2
  fi
  if ! [[ "$USB_WAIT_TIMEOUT_SECONDS" =~ ^[1-9][0-9]{0,9}$ ]] ||
      ((USB_WAIT_TIMEOUT_SECONDS > 2147483647)); then
    echo "错误：DATAGLOVE_USB_WAIT_TIMEOUT_SECONDS 必须是 1..2147483647 的整数秒数" >&2
    return 2
  fi
  if ! [[ "$GLOVE_PORT" =~ ^[1-9][0-9]{0,4}$ ]] || ((GLOVE_PORT > 65535)); then
    echo "错误：DATAGLOVE_PORT 必须是 1..65535 的整数端口" >&2
    return 2
  fi
  if [[ -n "$GLOVE_INTERFACE" ]] &&
      ! [[ "$GLOVE_INTERFACE" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_.:-]{0,14}$ ]]; then
    echo "错误：DATAGLOVE_INTERFACE 不是合法网卡名称：$GLOVE_INTERFACE" >&2
    return 2
  fi
  if [[ -n "$GLOVE_ROUTE_TABLE" ]] &&
      { ! [[ "$GLOVE_ROUTE_TABLE" =~ ^[1-3][0-9]{4}$ ]] ||
        ((GLOVE_ROUTE_TABLE < 10000 || GLOVE_ROUTE_TABLE > 30000)); }; then
    echo "错误：DATAGLOVE_ROUTE_TABLE 必须是 10000..30000 的整数" >&2
    return 2
  fi
  if ! host="$(_glove_ipv4_number "$GLOVE_HOST")"; then
    echo "错误：DATAGLOVE_HOST 必须是有效的 IPv4 主机地址：$GLOVE_HOST" >&2
    return 2
  fi
  prefix="${GLOVE_HOST_ADDRESS##*/}"
  if [[ "$GLOVE_HOST_ADDRESS" != */* ]] ||
      ! [[ "$prefix" =~ ^([1-9]|[12][0-9]|3[0-2])$ ]] ||
      ! source="$(_glove_ipv4_number "${GLOVE_HOST_ADDRESS%/*}")"; then
    echo "错误：DATAGLOVE_HOST_ADDRESS 必须是有效的 IPv4 主机地址/CIDR（前缀 1..32）：$GLOVE_HOST_ADDRESS" >&2
    return 2
  fi
  mask=$(((0xffffffff << (32 - prefix)) & 0xffffffff))
  network=$((source & mask))
  broadcast=$((network | (0xffffffff ^ mask)))
  if ((host == source || (host & mask) != network)) ||
      { ((prefix < 31)) && ((host == network || host == broadcast || source == network || source == broadcast)); }; then
    echo "错误：手套 $GLOVE_HOST 与主机 $GLOVE_HOST_ADDRESS 必须是同一直连网段内不同的有效主机地址" >&2
    return 2
  fi
}

route_matches() {
  local destination="$1"
  local interface="$2"
  local source_address="${3%/*}"
  local route
  if [[ -n "$GLOVE_ROUTE_TABLE" ]]; then
    route="$(ip -4 route get "$destination" from "$source_address")" || return 1
    [[ " $route " == *" from $source_address "* ]] || return 1
    [[ " $route " == *" table $GLOVE_ROUTE_TABLE "* ]] || return 1
  else
    route="$(ip route get "$destination")" || return 1
    [[ " $route " == *" src $source_address "* ]] || return 1
  fi
  [[ " $route " == *" dev $interface "* ]] || return 1
  printf '%s\n' "$route"
}

detect_glove_interface() {
  local deadline=$((SECONDS + USB_WAIT_TIMEOUT_SECONDS))
  local address_file address interface
  local -a matches
  echo "等待数据手套 USB 网卡（MAC=$GLOVE_MAC）..." >&2
  while :; do
    if [[ -n "$GLOVE_INTERFACE" && -f "$NET_SYS_CLASS/$GLOVE_INTERFACE/address" ]]; then
      if ! read -r address < "$NET_SYS_CLASS/$GLOVE_INTERFACE/address" ||
          [[ "${address,,}" != "${GLOVE_MAC,,}" ]]; then
        echo "错误：指定网卡 $GLOVE_INTERFACE 的 MAC 不匹配 $GLOVE_MAC，拒绝修改该网卡" >&2
        return 1
      fi
      printf '%s\n' "$GLOVE_INTERFACE"
      return 0
    fi
    if [[ -n "$GLOVE_INTERFACE" ]]; then
      ((SECONDS < deadline)) || break
      sleep 0.2 || return 1
      continue
    fi
    matches=()
    for address_file in "$NET_SYS_CLASS"/*/address; do
      [[ -f "$address_file" ]] || continue
      read -r address < "$address_file" || continue
      if [[ "${address,,}" == "${GLOVE_MAC,,}" ]]; then
        interface="${address_file%/address}"
        matches+=("${interface##*/}")
      fi
    done
    if ((${#matches[@]} > 1)); then
      echo "错误：数据手套 MAC 匹配到多个网卡：${matches[*]}，拒绝配置" >&2
      return 1
    fi
    if ((${#matches[@]} == 1)); then
      printf '%s\n' "${matches[0]}"
      return 0
    fi
    ((SECONDS < deadline)) || break
    sleep 0.2 || return 1
  done
  echo "错误：${USB_WAIT_TIMEOUT_SECONDS}s 内未检测到数据手套 USB 网卡；请重新插入手套 USB 并确认手套已上电" >&2
  return 1
}

_glove_interface_present() {
  local address
  if [[ ! -f "$NET_SYS_CLASS/$GLOVE_INTERFACE/address" ]] ||
      ! read -r address < "$NET_SYS_CLASS/$GLOVE_INTERFACE/address" ||
      [[ "${address,,}" != "${GLOVE_MAC,,}" ]]; then
    echo "错误：数据手套网卡 $GLOVE_INTERFACE 已消失或 MAC 已改变；请重新插入 USB 并确认手套已上电" >&2
    return 1
  fi
}

_glove_address_matches() {
  local addresses
  if ! addresses="$(ip -o -4 addr show dev "$GLOVE_INTERFACE" 2>&1)"; then
    echo "错误：读取手套网卡地址失败：$addresses" >&2
    return 2
  fi
  [[ " $addresses " == *" inet $GLOVE_HOST_ADDRESS "* ]]
}

# 专用表和优先级属于本手套；发现外来规则/路由时只报错，绝不删除或覆盖。
_glove_check_policy() {
  local rules routes line priority source table index destination
  local -a fields
  glove_policy_rule_ready=false
  if ! rules="$(ip -4 rule show 2>&1)"; then
    echo "错误：读取 IPv4 策略规则失败：$rules" >&2
    return 1
  fi
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    read -r -a fields <<< "$line"
    priority="${fields[0]%:}"
    source="" table=""
    for ((index=1; index<${#fields[@]}-1; index++)); do
      case "${fields[index]}" in
        from) source="${fields[index+1]}" ;;
        lookup|table) table="${fields[index+1]}" ;;
      esac
    done
    if [[ "$priority" == "$GLOVE_ROUTE_TABLE" ||
          "${source%/32}" == "${GLOVE_HOST_ADDRESS%/*}" ||
          "$table" == "$GLOVE_ROUTE_TABLE" ]]; then
      if [[ "$priority" == "$GLOVE_ROUTE_TABLE" &&
            "${source%/32}" == "${GLOVE_HOST_ADDRESS%/*}" &&
            "$table" == "$GLOVE_ROUTE_TABLE" &&
            "${#fields[@]}" == 5 &&
            "${fields[1]}" == from &&
            ( "${fields[3]}" == lookup || "${fields[3]}" == table ) &&
            "$glove_policy_rule_ready" == false ]]; then
        glove_policy_rule_ready=true
      else
        echo "错误：手套策略规则与已有规则冲突，拒绝修改：$line" >&2
        return 1
      fi
    fi
  done <<< "$rules"
  if ! routes="$(LC_ALL=C ip -4 route show table "$GLOVE_ROUTE_TABLE" 2>&1)"; then
    # iproute2 对尚未创建的表返回 ENOENT；其他失败不能当作空表。
    if [[ "$routes" != *"FIB table does not exist"* ]]; then
      echo "错误：读取手套策略路由表失败：$routes" >&2
      return 1
    fi
    routes=""
  fi
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    read -r destination _ <<< "$line"
    if [[ "${destination%/32}" != "$GLOVE_HOST" ||
          " $line " != *" dev $GLOVE_INTERFACE "* ||
          " $line " != *" src ${GLOVE_HOST_ADDRESS%/*} "* ||
          " $line " == *" via "* ]]; then
      echo "错误：手套路由表 $GLOVE_ROUTE_TABLE 已被其他路由占用，拒绝覆盖：$line" >&2
      return 1
    fi
  done <<< "$routes"
}

_glove_mtu_matches() {
  [[ -z "$GLOVE_MTU" ]] && return 0
  local actual
  read -r actual < "$NET_SYS_CLASS/$GLOVE_INTERFACE/mtu" || return 1
  [[ "$actual" == "$GLOVE_MTU" ]]
}

_glove_verify_network() {
  _glove_interface_present || return 1
  if ! _glove_address_matches; then
    echo "错误：手套网卡 $GLOVE_INTERFACE 未配置主机地址 $GLOVE_HOST_ADDRESS" >&2
    return 1
  fi
  if ! _glove_mtu_matches; then
    echo "错误：手套网卡 $GLOVE_INTERFACE 的 MTU 必须为 $GLOVE_MTU" >&2
    return 1
  fi
  if ! route_matches "$GLOVE_HOST" "$GLOVE_INTERFACE" "$GLOVE_HOST_ADDRESS"; then
    echo "错误：手套路由验证失败：目标 $GLOVE_HOST 必须经 $GLOVE_INTERFACE，源地址必须为 ${GLOVE_HOST_ADDRESS%/*}" >&2
    return 1
  fi
  _glove_interface_present || return 1
}

_glove_ip_modify() {
  local output
  _glove_interface_present || return 1
  if ! output="$("${glove_ip_command[@]}" "$@" 2>&1)"; then
    echo "错误：配置手套网卡失败（ip $*）：$output" >&2
    _glove_interface_present || return 1
    echo "请在本机终端重新运行并完成 sudo 授权；无交互终端时不会等待密码，也不会自动修改 sudo 权限" >&2
    return 1
  fi
}

configure_dataglove_network() {
  validate_glove_network_settings || return $?
  if ! command -v ip >/dev/null 2>&1; then
    echo "错误：缺少 ip 命令，请安装 iproute2" >&2
    return 1
  fi
  GLOVE_INTERFACE="$(detect_glove_interface)" || return 1
  _glove_interface_present || return 1
  local glove_policy_rule_ready=false
  if [[ -n "$GLOVE_ROUTE_TABLE" ]]; then
    _glove_check_policy || return 1
  fi
  if [[ "$GLOVE_NETWORK_CHECK_ONLY" == 1 ]]; then
    _glove_verify_network || return 1
    echo "手套网络只读预检通过：$GLOVE_INTERFACE"
    return 0
  fi
  local address_ready=false address_status state output uid
  local -a changes=() glove_ip_command=(ip)
  if _glove_address_matches; then
    address_ready=true
  else
    address_status=$?
    ((address_status == 1)) || return 1
  fi
  if [[ "$address_ready" == true ]] && _glove_mtu_matches &&
      { [[ -z "$GLOVE_ROUTE_TABLE" ]] || [[ "$glove_policy_rule_ready" == true ]]; } &&
      route_matches "$GLOVE_HOST" "$GLOVE_INTERFACE" "$GLOVE_HOST_ADDRESS" >/dev/null 2>&1; then
    _glove_interface_present || return 1
    echo "数据手套网络已就绪：$GLOVE_INTERFACE，主机 $GLOVE_HOST_ADDRESS → $GLOVE_HOST:$GLOVE_PORT"
    return 0
  fi

  # device modify 只改运行时连接；+ 属性保留原有地址（包括 169.254.*）和其他路由。
  # /32 路由必须指定 src，单加地址仍可能让 Linux 选择原来的链路本地源地址。
  [[ "$address_ready" == true ]] || changes+=(+ipv4.addresses "$GLOVE_HOST_ADDRESS")
  [[ -z "$GLOVE_MTU" ]] || changes+=(802-3-ethernet.mtu "$GLOVE_MTU")
  changes+=(+ipv4.routes "$GLOVE_HOST/32 0.0.0.0 0 src=${GLOVE_HOST_ADDRESS%/*}")
  if [[ -n "$GLOVE_ROUTE_TABLE" ]]; then
    echo "使用手套专用源地址策略路由表 $GLOVE_ROUTE_TABLE，保留主路由表" >&2
  elif command -v nmcli >/dev/null 2>&1; then
    if state="$(LC_ALL=C nmcli -g GENERAL.STATE device show "$GLOVE_INTERFACE" 2>&1)"; then
      if [[ "$state" == 100 || "$state" == 100\ * ]]; then
        _glove_interface_present || return 1
        if output="$(LC_ALL=C nmcli --wait 10 device modify "$GLOVE_INTERFACE" "${changes[@]}" 2>&1)"; then
          _glove_verify_network || return 1
          echo "数据手套运行时网络已通过 NetworkManager 配置"
          return 0
        fi
        echo "NetworkManager 配置手套网卡失败：$output" >&2
        _glove_interface_present || return 1
        case "${output,,}" in
          *not\ authorized*|*not\ authorised*|*permission*|*not\ permitted*|*not\ managed*|*unmanaged*|*not\ activated*|*not\ active*|*not\ running*) ;;
          *) echo "错误：NetworkManager 未能应用手套地址/源路由，停止启动" >&2; return 1 ;;
        esac
      else
        echo "NetworkManager 未托管活动手套连接（状态：$state），改用原启动脚本的 ip 绑定方式" >&2
      fi
    else
      echo "NetworkManager 无法读取手套设备：$state；改用原启动脚本的 ip 绑定方式" >&2
    fi
  else
    echo "未找到 nmcli，使用原启动脚本的 ip 绑定方式配置手套网络" >&2
  fi

  _glove_interface_present || return 1
  uid="$(id -u)" || return 1
  if [[ "$uid" != 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
      echo "错误：NetworkManager 不可用且缺少 sudo，无法配置手套网络" >&2
      return 1
    fi
    if [[ -t 0 ]]; then
      echo "将自动绑定数据手套网卡；若 sudo 提示密码，请在当前终端输入本机用户密码。" >&2
      glove_ip_command=(sudo ip)
    else
      glove_ip_command=(sudo -n ip)
    fi
  fi
  _glove_ip_modify link set dev "$GLOVE_INTERFACE" up || return 1
  if [[ -n "$GLOVE_MTU" ]]; then
    _glove_ip_modify link set dev "$GLOVE_INTERFACE" mtu "$GLOVE_MTU" || return 1
  fi
  if [[ "$address_ready" != true ]]; then
    if [[ -n "$GLOVE_ROUTE_TABLE" ]]; then
      _glove_ip_modify addr replace "$GLOVE_HOST_ADDRESS" dev "$GLOVE_INTERFACE" noprefixroute || return 1
    else
      _glove_ip_modify addr replace "$GLOVE_HOST_ADDRESS" dev "$GLOVE_INTERFACE" || return 1
    fi
  fi
  if [[ -n "$GLOVE_ROUTE_TABLE" ]]; then
    _glove_check_policy || return 1
    _glove_ip_modify route replace "$GLOVE_HOST/32" dev "$GLOVE_INTERFACE" src "${GLOVE_HOST_ADDRESS%/*}" table "$GLOVE_ROUTE_TABLE" || return 1
    if [[ "$glove_policy_rule_ready" != true ]]; then
      _glove_ip_modify rule add priority "$GLOVE_ROUTE_TABLE" from "${GLOVE_HOST_ADDRESS%/*}/32" lookup "$GLOVE_ROUTE_TABLE" || return 1
    fi
  else
    _glove_ip_modify route replace "$GLOVE_HOST/32" dev "$GLOVE_INTERFACE" src "${GLOVE_HOST_ADDRESS%/*}" || return 1
  fi
  _glove_verify_network || return 1
  echo "数据手套运行时网络已通过 ip 配置"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  configure_dataglove_network
  exit $?
fi
