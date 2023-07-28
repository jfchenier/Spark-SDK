add_library(cmsis_5 "")

set_target_properties(cmsis_5 PROPERTIES LINKER_LANGUAGE C)

target_sources(cmsis_5
    PRIVATE
        third-party/arm-software/cmsis_5/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_q15.c
        third-party/arm-software/cmsis_5/CMSIS/DSP/Source/FilteringFunctions/arm_fir_decimate_init_q15.c
        third-party/arm-software/cmsis_5/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_q15.c
        third-party/arm-software/cmsis_5/CMSIS/DSP/Source/FilteringFunctions/arm_fir_interpolate_init_q15.c
    PUBLIC
        third-party/arm-software/cmsis_5/CMSIS/Core/Include/core_cm4.h
        third-party/arm-software/cmsis_5/CMSIS/Core/Include/cmsis_version.h
        third-party/arm-software/cmsis_5/CMSIS/Core/Include/cmsis_gcc.h
        third-party/arm-software/cmsis_5/CMSIS/DSP/Include/arm_math.h
)

target_include_directories(cmsis_5 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/cmsis_5/CMSIS/Core/Include)
target_include_directories(cmsis_5 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/cmsis_5/CMSIS/DSP/Include)
