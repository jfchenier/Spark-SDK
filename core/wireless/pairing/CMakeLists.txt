
target_sources(wps
    PRIVATE
        swc_pairing.c
    PUBLIC
        swc_pairing.h
)

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})
