add_library(cmsis_device_g4 "")

set_target_properties(cmsis_device_g4 PROPERTIES LINKER_LANGUAGE C)

target_sources(cmsis_device_g4
    PUBLIC
        third-party/stmicroelectronics/cmsis_device_g4/Include/stm32g4xx.h
        third-party/stmicroelectronics/cmsis_device_g4/Include/system_stm32g4xx.h
)

target_link_libraries(cmsis_device_g4 PUBLIC cmsis_5)
target_compile_definitions(cmsis_device_g4 PUBLIC ${STM32G4_MCU_MODEL} USE_HAL_DRIVER)
target_include_directories(cmsis_device_g4 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/cmsis_device_g4/Include)
