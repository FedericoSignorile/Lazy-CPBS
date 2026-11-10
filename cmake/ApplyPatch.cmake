# Applies PATCH to the current source tree once (no-op if already applied).
execute_process(COMMAND git apply --reverse --check ${PATCH}
                RESULT_VARIABLE already_applied OUTPUT_QUIET ERROR_QUIET)
if(already_applied EQUAL 0)
  return()
endif()
execute_process(COMMAND git apply --whitespace=nowarn ${PATCH} RESULT_VARIABLE res)
if(NOT res EQUAL 0)
  message(FATAL_ERROR "Cannot apply ${PATCH}")
endif()
