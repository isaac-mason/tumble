# usage: cmake -DCONSUMER=<project dir> -DTUMBLE=<tumble checkout> -DBUILD=<build dir> -P build_consumer.cmake
# configures, builds and runs a consumer project as a user would, over this checkout in place of the git fetch
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER}" -B "${BUILD}" -G Ninja -DFETCHCONTENT_SOURCE_DIR_TUMBLE=${TUMBLE}
                RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "configure failed")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BUILD}" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "build failed")
endif()
execute_process(COMMAND "${BUILD}/game" RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "the game failed")
endif()
