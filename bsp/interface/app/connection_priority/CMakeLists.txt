add_library(iface_connection_priority "")
set_target_properties(iface_connection_priority PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_connection_priority PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_connection_priority PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_connection_priority PRIVATE iface_connection_priority_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_connection_priority PRIVATE iface_connection_priority_pulsar.c)
endif()
