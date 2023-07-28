cmake_minimum_required(VERSION 3.18)

if (GITLAB_CI_UNIT_TEST)
    #Setup projet for WPS standalone compilation with unit tests
    project(wireless_core)
    find_package(Catch2 REQUIRED)
    add_executable(sdk_unit_tests "")
    set(BUILD_SDK_UNIT_TESTS ON)

    #Unit tests only support single radio
    set(MULTI_TRANSCEIVER "SINGLE_TRANSCEIVER")

    #Enable error checking
    set(ACCESS_ADV_ERR_CHECK_EN "ACCESS_ADV_ERR_CHECK_EN=1" CACHE INTERNAL "Access advance error check flag")

    add_subdirectory(toolchain)
endif()

add_library(wps "")

set(TRANSCEIVER "SR1000" CACHE STRING "Set SR1000 transceiver model has default if the varaible wasn't initialized.")

if (MULTI_TRANSCEIVER STREQUAL "DUAL_TRANSCEIVER")
    target_compile_definitions(wps PUBLIC WPS_RADIO_COUNT=2)
else()
    target_compile_definitions(wps PUBLIC WPS_RADIO_COUNT=1)
endif()

message("WPS Core being compiled for a number of " ${WPS_RADIO_COUNT} " " ${TRANSCEIVER} " transceiver")

if(BUILD_SDK_UNIT_TESTS)
    add_subdirectory(unit-tests)
    if (GITLAB_CI_UNIT_TEST)
        add_subdirectory(lib/spark/queue)
        add_subdirectory(lib/spark/memory)
        add_subdirectory(lib/spark/buffer)
        add_subdirectory(lib/spark/test_utils)
        target_link_libraries(sdk_unit_tests PRIVATE Catch2::Catch2WithMain wps test_utils)
    endif()
endif()

target_compile_definitions(wps PUBLIC ${ACCESS_ADV_ERR_CHECK_EN} ${SPARK_WPS_CFG_FILE_EXISTS} ${TRANSCEIVER})

add_subdirectory(api)
add_subdirectory(link)
add_subdirectory(pairing)
add_subdirectory(phy)
add_subdirectory(protocol_stack)
add_subdirectory(transceiver)
add_subdirectory(xlayer)

target_link_libraries(wps PUBLIC buffer memory queue)



