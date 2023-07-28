add_library(iface_fragmentation "")

target_link_libraries(iface_fragmentation PUBLIC wps hardware)
target_include_directories(iface_fragmentation PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_fragmentation PRIVATE iface_fragmentation_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_fragmentation PRIVATE iface_fragmentation_pulsar.c)
endif()
