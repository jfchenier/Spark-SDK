add_library(stm32_mw_usb_device "")

target_sources(stm32_mw_usb_device
    PRIVATE
        third-party/stmicroelectronics/stm32_mw_usb_device/Class/CDC/Src/usbd_cdc.c
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Src/usbd_core.c
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Src/usbd_ctlreq.c
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Src/usbd_ioreq.c
    PUBLIC
        third-party/stmicroelectronics/stm32_mw_usb_device/Class/CDC/Inc/usbd_cdc.h
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Inc/usbd_core.h
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Inc/usbd_ctlreq.h
        third-party/stmicroelectronics/stm32_mw_usb_device/Core/Inc/usbd_ioreq.h
)

target_link_libraries(stm32_mw_usb_device PUBLIC cmsis_device_g4)
target_link_libraries(stm32_mw_usb_device PRIVATE stm32g4xx_hal_driver)
target_include_directories(stm32_mw_usb_device PUBLIC ${CMAKE_CURRENT_LIST_DIR}/stm32_mw_usb_device/Class/CDC/Inc ${CMAKE_CURRENT_LIST_DIR}/stm32_mw_usb_device/Core/Inc)
target_include_directories(stm32_mw_usb_device PUBLIC ${STM32G4_USB_DEVICE_APP_PATH} ${STM32G4_USB_DEVICE_TARGET_PATH})
