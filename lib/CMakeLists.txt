# SPARK
if(BUILD_SDK_UNIT_TESTS)
    add_subdirectory(spark/logger)
    add_subdirectory(spark/test_utils)
endif()
add_subdirectory(spark/adpcm)
add_subdirectory(spark/buffer)
add_subdirectory(spark/crc)
if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/spark/fixed_point")
add_subdirectory(spark/fixed_point)
endif()
add_subdirectory(spark/memory)
add_subdirectory(spark/pseudo_data)
add_subdirectory(spark/queue)
add_subdirectory(spark/resampling)

# Third Party
include(third-party/arm-software/cmsis_5.cmake)
if(BUILD_SDK_UNIT_TESTS)
    add_subdirectory(third-party/catchorg/catch2)
    add_subdirectory(third-party/meekrosoft/fff)
elseif (HARDWARE STREQUAL "NUCLEO-U575ZI-Q")
    include(third-party/stmicroelectronics/cmsis_device_u5.cmake)
    include(third-party/stmicroelectronics/stm32u5xx_hal_driver.cmake)
else()
    include(third-party/stmicroelectronics/cmsis_device_g4.cmake)
    include(third-party/stmicroelectronics/stm32_mw_usb_device.cmake)
    include(third-party/stmicroelectronics/stm32g4xx_hal_driver.cmake)
endif()
