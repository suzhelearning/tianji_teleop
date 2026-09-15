foreach(variable IN ITEMS VIEWER)
  if(NOT DEFINED ${variable} OR "${${variable}}" STREQUAL "")
    message(FATAL_ERROR "${variable} must be provided")
  endif()
endforeach()

set(viewer_command "${VIEWER}")
if(DEFINED CONFIG AND NOT "${CONFIG}" STREQUAL "")
  list(APPEND viewer_command --config "${CONFIG}")
endif()
list(APPEND viewer_command --headless --duration 1.0)
if(DEFINED MODEL AND NOT "${MODEL}" STREQUAL "")
  list(APPEND viewer_command --model "${MODEL}")
endif()
if(ENABLE_PICO)
  list(APPEND viewer_command --pico-teleop)
endif()

execute_process(
  # The headless smoke test exercises 13 controller/mode transitions. Give
  # each transition enough control cycles to export a valid motion state.
  COMMAND ${viewer_command}
  RESULT_VARIABLE viewer_result
  OUTPUT_VARIABLE viewer_stdout
  ERROR_VARIABLE viewer_stderr
)

if(NOT "${viewer_result}" STREQUAL "0" AND NOT "${viewer_result}" STREQUAL "2")
  message(FATAL_ERROR
    "Viewer exited with ${viewer_result}; expected 0 or 2.\n"
    "stdout:\n${viewer_stdout}\n"
    "stderr:\n${viewer_stderr}"
  )
endif()

function(assert_stdout_value variable prefix)
  if(DEFINED ${variable} AND NOT "${${variable}}" STREQUAL "")
    set(expected_output "${prefix}${${variable}}")
    string(FIND "${viewer_stdout}" "${expected_output}" expected_output_index)
    if(expected_output_index EQUAL -1)
      message(FATAL_ERROR
        "Viewer stdout did not contain '${expected_output}'.\n"
        "stdout:\n${viewer_stdout}\n"
        "stderr:\n${viewer_stderr}"
      )
    endif()
  endif()
endfunction()

assert_stdout_value(EXPECTED_CONFIG "viewer_config=")
assert_stdout_value(EXPECTED_CONTROL_LEVEL "control_level=")
assert_stdout_value(EXPECTED_ALGORITHM "algorithm=")
assert_stdout_value(EXPECTED_STATE_SOURCE "control_state_source=")
assert_stdout_value(EXPECTED_PICO_BIND "pico_udp_bind=")
