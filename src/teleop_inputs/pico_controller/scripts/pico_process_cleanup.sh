#!/usr/bin/env bash

pico_normalize_ros_domain() {
  local value="${1:-}"

  [[ "$value" =~ ^[0-9]+$ ]] || return 2
  while [[ ${#value} -gt 1 && "${value:0:1}" == "0" ]]; do
    value="${value:1}"
  done
  [[ ${#value} -le 3 ]] || return 2
  ((10#$value <= 232)) || return 2
  printf '%d\n' "$((10#$value))"
}

# A process group can outlive its leader.  Inspect all non-zombie members so a
# launch descendant cannot escape cleanup merely because the waitable leader
# handled SIGINT and exited first.
pico_process_group_is_running() {
  local group_id="${1:-}"
  local process_group
  local state

  [[ "$group_id" =~ ^[0-9]+$ ]] || return 1
  while read -r process_group state; do
    if [[ "$process_group" == "$group_id" && -n "$state" && "$state" != Z* ]]; then
      return 0
    fi
  done < <(/usr/bin/ps -e -o pgid= -o stat= 2>/dev/null)
  return 1
}

pico_stop_process_group() {
  local leader_pid="${1:-}"
  local interrupt_attempts="${2:-20}"
  local terminate_attempts="${3:-10}"
  local attempt

  [[ "$leader_pid" =~ ^[0-9]+$ ]] || return 0
  ((leader_pid > 1)) || return 0
  [[ "$interrupt_attempts" =~ ^[0-9]+$ ]] || interrupt_attempts=20
  [[ "$terminate_attempts" =~ ^[0-9]+$ ]] || terminate_attempts=10

  if ! pico_process_group_is_running "$leader_pid"; then
    wait "$leader_pid" 2>/dev/null || true
    return 0
  fi

  kill -INT -- "-$leader_pid" 2>/dev/null || true
  for ((attempt=0; attempt<interrupt_attempts; attempt++)); do
    pico_process_group_is_running "$leader_pid" || break
    sleep 0.05
  done

  if pico_process_group_is_running "$leader_pid"; then
    kill -TERM -- "-$leader_pid" 2>/dev/null || true
    for ((attempt=0; attempt<terminate_attempts; attempt++)); do
      pico_process_group_is_running "$leader_pid" || break
      sleep 0.05
    done
  fi

  if pico_process_group_is_running "$leader_pid"; then
    kill -KILL -- "-$leader_pid" 2>/dev/null || true
  fi
  wait "$leader_pid" 2>/dev/null || true
}
