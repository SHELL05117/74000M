if(NOT DEFINED INSPECTOR OR NOT DEFINED RECORDING)
    message(FATAL_ERROR "INSPECTOR and RECORDING are required")
endif()

execute_process(
    COMMAND "${INSPECTOR}" --json "${RECORDING}"
    RESULT_VARIABLE inspector_result
    OUTPUT_VARIABLE inspector_output
    ERROR_VARIABLE inspector_error)

if(NOT inspector_result EQUAL 2)
    message(FATAL_ERROR
        "Expected v5l_inspect exit 2, got ${inspector_result}: "
        "${inspector_output}${inspector_error}")
endif()

if(NOT inspector_output MATCHES "\"NO_FRAMES\"")
    message(FATAL_ERROR
        "Expected NO_FRAMES in inspector output: ${inspector_output}")
endif()
