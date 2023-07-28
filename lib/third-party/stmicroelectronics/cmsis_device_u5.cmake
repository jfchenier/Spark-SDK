add_library(cmsis_device_u5 "")

set_target_properties(cmsis_device_u5 PROPERTIES LINKER_LANGUAGE C)

target_sources(cmsis_device_u5
    PUBLIC
        third-party/stmicroelectronics/cmsis_device_u5/Include/stm32u5xx.h
        third-party/stmicroelectronics/cmsis_device_u5/Include/system_stm32u5xx.h
)

target_link_libraries(cmsis_device_u5 PUBLIC cmsis_5)
target_compile_definitions(cmsis_device_u5 PUBLIC ${STM32U5_MCU_MODEL} USE_HAL_DRIVER)
target_include_directories(cmsis_device_u5 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/cmsis_device_u5/Include)
