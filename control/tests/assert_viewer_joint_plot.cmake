if(NOT DEFINED VIEWER OR NOT DEFINED CONFIG OR NOT DEFINED MODEL)
  message(FATAL_ERROR "VIEWER, CONFIG and MODEL are required")
endif()

execute_process(
  COMMAND "${VIEWER}"
    --headless
    --duration 4.0
    --config "${CONFIG}"
    --model "${MODEL}"
  WORKING_DIRECTORY "${WORKING_DIRECTORY}"
  RESULT_VARIABLE viewer_result
  OUTPUT_VARIABLE viewer_output
  ERROR_VARIABLE viewer_error
)

if(NOT viewer_result EQUAL 0 AND NOT viewer_result EQUAL 2)
  message(FATAL_ERROR
    "Viewer failed with ${viewer_result}\nstdout:\n${viewer_output}\nstderr:\n${viewer_error}")
endif()

foreach(required_pattern IN ITEMS
    "joint_plot_drained=[1-9][0-9]*"
    "joint_plot_both_arms_finite=1"
    "joint_plot_reference_jerk_valid_seen=1"
    "joint_plot_actual_jerk_valid_seen=1")
  if(NOT viewer_output MATCHES "${required_pattern}")
    message(FATAL_ERROR
      "Missing ${required_pattern}\nstdout:\n${viewer_output}\nstderr:\n${viewer_error}")
  endif()
endforeach()
