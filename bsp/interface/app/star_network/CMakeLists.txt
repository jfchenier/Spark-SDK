add_library(iface_star_network "")
set_target_properties(iface_star_network PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_star_network PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_star_network PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_star_network PRIVATE iface_star_network_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_star_network PRIVATE iface_star_network_pulsar.c)
endif()