add_library(iface_core "")
set_target_properties(iface_core PROPERTIES LINKER_LANGUAGE C)

set(INTERFACE_PATH ${CMAKE_CURRENT_LIST_DIR}/interface CACHE INTERNAL "Path to BSP interface cores and apps root.")
set(IFACE_WIRELESS_PATH ${INTERFACE_PATH}/wireless_core CACHE INTERNAL "Path to BSP interface Wireless Core source files.")
set(IFACE_AUDIO_PATH ${INTERFACE_PATH}/audio_core CACHE INTERNAL "Path to BSP interface Audio Core source files.")
set(IFACE_PAIRING_PATH ${INTERFACE_PATH}/pairing CACHE INTERNAL "Path to BSP interface Pairing source files.")
set(IFACE_APP_PATH ${INTERFACE_PATH}/app CACHE INTERNAL "Path to BSP interface App CMakeLists file.")
set(IFACE_LIB_PATH ${INTERFACE_PATH}/lib CACHE INTERNAL "Path to BSP interface SPARK Libraries files.")

message("Selecting board support package (bsp) for ${HARDWARE} hardware.")

target_link_libraries(iface_core PUBLIC wps max98091_driver audio_core)

if(HARDWARE STREQUAL "EVK")
    include(hardware/evk.cmake)
    target_link_libraries(iface_core PUBLIC hardware)

    set(IFACE_LIB_CRITICAL_SECTION_OPT_PATH ${IFACE_LIB_PATH}/queue/evk CACHE INTERNAL "Path to SPARK circular queue critical section optimization files.")

    target_sources(iface_core
        PRIVATE
            ${IFACE_WIRELESS_PATH}/iface_wireless_evk.c
            ${IFACE_AUDIO_PATH}/iface_audio_evk.c
            ${IFACE_PAIRING_PATH}/iface_pairing_evk.c
        PUBLIC
            ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH}/circular_queue_critical_section.h
    )

elseif(HARDWARE STREQUAL "PULSAR")
    include(hardware/pulsar.cmake)
    target_link_libraries(iface_core PUBLIC hardware)

    set(IFACE_LIB_CRITICAL_SECTION_OPT_PATH ${IFACE_LIB_PATH}/queue/pulsar CACHE INTERNAL "Path to SPARK circular queue critical section optimization files.")

    target_sources(iface_core
        PRIVATE
            ${IFACE_WIRELESS_PATH}/iface_wireless_pulsar.c
            ${IFACE_AUDIO_PATH}/iface_audio_pulsar.c
            ${IFACE_PAIRING_PATH}/iface_pairing_pulsar.c
        PUBLIC
            ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH}/circular_queue_critical_section.h
    )

elseif(HARDWARE STREQUAL "NUCLEO-U575ZI-Q")
    include(hardware/nucleo-u575zi-q.cmake)
    target_link_libraries(iface_core PUBLIC hardware)

    set(IFACE_LIB_CRITICAL_SECTION_OPT_PATH ${IFACE_LIB_PATH}/queue/nucleo-u575zi-q CACHE INTERNAL "Path to SPARK circular queue critical section optimization files.")

    target_sources(iface_core
        PRIVATE
            ${IFACE_WIRELESS_PATH}/iface_wireless_nucleo-u575zi-q.c
            ${IFACE_AUDIO_PATH}/iface_audio_nucleo-u575zi-q.c
            ${IFACE_PAIRING_PATH}/iface_pairing_nucleo-u575zi-q.c
        PUBLIC
            ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH}/circular_queue_critical_section.h
    )

endif()

set(SPARK_CIRC_QUEUE_OPT_PATH ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH} CACHE PATH "Path to BSP interface SPARK Libraries files project wide")
set(SPARK_LIB_OPT_DEPENDANCIES hardware CACHE INTERNAL "Path to the BSP library for SPARK library dependancies")

target_include_directories(iface_core PUBLIC ${IFACE_WIRELESS_PATH} ${IFACE_AUDIO_PATH} ${IFACE_PAIRING_PATH} ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH})

add_subdirectory(${IFACE_APP_PATH})
