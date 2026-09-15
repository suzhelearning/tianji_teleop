if(NOT DEFINED VIEWER OR NOT DEFINED CONFIG OR NOT DEFINED MODEL OR
   NOT DEFINED WORKING_DIRECTORY OR NOT DEFINED OUTPUT)
  message(FATAL_ERROR
    "VIEWER, CONFIG, MODEL, WORKING_DIRECTORY and OUTPUT are required")
endif()

file(REMOVE "${OUTPUT}")
execute_process(
  COMMAND "${VIEWER}"
    --headless
    --duration 1.0
    --config "${CONFIG}"
    --model "${MODEL}"
    --joint-telemetry "${OUTPUT}"
  WORKING_DIRECTORY "${WORKING_DIRECTORY}"
  RESULT_VARIABLE viewer_result
  OUTPUT_VARIABLE viewer_output
  ERROR_VARIABLE viewer_error
)

if(NOT viewer_result EQUAL 0 AND NOT viewer_result EQUAL 2)
  message(FATAL_ERROR
    "Viewer failed with ${viewer_result}\nstdout:\n${viewer_output}\nstderr:\n${viewer_error}")
endif()
if(NOT EXISTS "${OUTPUT}")
  message(FATAL_ERROR "Joint telemetry CSV was not created: ${OUTPUT}")
endif()

file(READ "${OUTPUT}" csv)
foreach(required_column IN ITEMS
    "left_j1_reference_q"
    "left_j3_position_upper"
    "left_j7_reference_jerk"
    "right_j1_reference_q"
    "right_j3_position_lower"
    "right_j7_jerk_upper")
  if(NOT csv MATCHES "(^|,)${required_column}(,|\\n)")
    message(FATAL_ERROR "Missing joint telemetry column: ${required_column}")
  endif()
endforeach()
file(STRINGS "${OUTPUT}" csv_lines LIMIT_COUNT 2)
list(LENGTH csv_lines csv_line_count)
if(csv_line_count LESS 2)
  message(FATAL_ERROR "Joint telemetry CSV contains no samples")
endif()
