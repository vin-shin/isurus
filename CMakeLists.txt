cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME makolongfin2)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Src/encoder.c
    ${CMAKE_SOURCE_DIR}/Core/Src/csense.c
    ${CMAKE_SOURCE_DIR}/Core/Src/motor_pwm.c
    ${CMAKE_SOURCE_DIR}/Core/Src/openloop.c
    ${CMAKE_SOURCE_DIR}/Core/Src/foc.c
    ${CMAKE_SOURCE_DIR}/Core/Src/drive.c
    ${CMAKE_SOURCE_DIR}/Core/Src/position.c
    ${CMAKE_SOURCE_DIR}/Core/Src/can.c
    ${CMAKE_SOURCE_DIR}/Core/Src/haptic.c
)

# The 30 kHz control ISR is a hard real-time deadline, not a hot loop to be
# tuned later: overrun the 33.3 us PWM period and control steps are simply lost.
# The budget is 1 / PWM_FREQ_HZ, so raising the switching frequency shortens it.
#
# At -O0 every float operation round-trips through the stack, and the FOC
# transforms plus the position profile are almost entirely float. Measured with
# the whole project at -O0, worst-case ISR was 48.2 us with a motion profile
# running. That was already 96% of the 50 us budget this project had at 20 kHz;
# against the present 33.3 us period it does not fit at all.
#
# So the files on the ISR path are built optimised while everything else stays
# at -O0 and remains comfortable to step through. If you need to single-step
# the FOC maths itself, drop this block temporarily rather than lowering it
# globally; the loop will overrun while you do.
set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/Core/Src/foc.c
    ${CMAKE_SOURCE_DIR}/Core/Src/drive.c
    ${CMAKE_SOURCE_DIR}/Core/Src/position.c
    ${CMAKE_SOURCE_DIR}/Core/Src/haptic.c
    ${CMAKE_SOURCE_DIR}/Core/Src/csense.c
    ${CMAKE_SOURCE_DIR}/Core/Src/encoder.c
    ${CMAKE_SOURCE_DIR}/Core/Src/motor_pwm.c
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    PROPERTIES COMPILE_OPTIONS "-O2"
)
# can.c is deliberately NOT in that list. It is polled from the main loop and
# never runs in interrupt context, so it has no deadline to miss and is more
# useful left steppable.

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# sqrtf() is required by the C standard to set errno on a negative argument, so
# GCC emits a call into libm rather than the Cortex-M4F's single VSQRT.F32
# instruction - even with -mfpu=fpv4-sp-d16. In the control ISR that is not a
# micro-optimisation: the two sqrtf calls in the position profile cost ~1400
# cycles between them and pushed the worst-case ISR from 39.2 us to 50.1 us.
# Measured at 20 kHz, that was straight through the then-50 us PWM period; at
# the present 30 kHz the same 1400 cycles are a third of the entire budget.
#
# Nothing in this project reads errno, so dropping that requirement is free.
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -fno-math-errno
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
