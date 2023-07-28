add_executable(hello_world_coordinator.elf "")
target_sources(hello_world_coordinator.elf
    PRIVATE
        hello_world_coord.c
        swc_cfg_coord.h
)
target_link_libraries(hello_world_coordinator.elf PRIVATE iface_core iface_hello_world wps)
if (MULTI_TRANSCEIVER STREQUAL "DUAL_TRANSCEIVER")
    target_compile_definitions(hello_world_coordinator.elf PRIVATE WPS_RADIO_COUNT=2 SWC_RADIO_COUNT=2)
else()
    target_compile_definitions(hello_world_coordinator.elf PRIVATE WPS_RADIO_COUNT=1 SWC_RADIO_COUNT=1)
endif()
add_custom_command(TARGET hello_world_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary hello_world_coordinator.elf hello_world_coordinator.bin
)

add_executable(hello_world_node.elf "")
target_sources(hello_world_node.elf
    PRIVATE
        hello_world_node.c
        swc_cfg_node.h
)
target_link_libraries(hello_world_node.elf PRIVATE iface_core iface_hello_world wps)
if (MULTI_TRANSCEIVER STREQUAL "DUAL_TRANSCEIVER")
    target_compile_definitions(hello_world_node.elf PRIVATE WPS_RADIO_COUNT=2 SWC_RADIO_COUNT=2)
else()
    target_compile_definitions(hello_world_node.elf PRIVATE WPS_RADIO_COUNT=1 SWC_RADIO_COUNT=1)
endif()
add_custom_command(TARGET hello_world_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary hello_world_node.elf hello_world_node.bin
)
