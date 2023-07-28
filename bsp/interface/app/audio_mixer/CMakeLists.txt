add_library(iface_audio_mixer "")
set_target_properties(iface_audio_mixer PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_audio_mixer PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_audio_mixer PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_audio_mixer PRIVATE iface_audio_mixer_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_audio_mixer PRIVATE iface_audio_mixer_pulsar.c)
endif()