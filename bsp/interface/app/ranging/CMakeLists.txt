add_library(iface_ranging "")

target_link_libraries(iface_ranging PUBLIC wps hardware ranging_core)
target_include_directories(iface_ranging PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_ranging PRIVATE iface_ranging_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_ranging PRIVATE iface_ranging_pulsar.c)
endif()
