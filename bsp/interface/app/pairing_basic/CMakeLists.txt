add_library(iface_pairing_basic "")
set_target_properties(iface_pairing_basic PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_pairing_basic PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_pairing_basic PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_pairing_basic PRIVATE iface_pairing_basic_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_pairing_basic PRIVATE iface_pairing_basic_pulsar.c)
endif()