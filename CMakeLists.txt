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
set(CMAKE_PROJECT_NAME Astra_Frame_V1.0)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/adaptive_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/adrc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/CAN_receive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/chassis_power_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/crc_ref.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/custom_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/Fuzzy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/hex_chassis_power_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/PID.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/referee.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/remote_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/RM_Cilent_UI.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Src/VT13_rc_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Src/bsp_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Src/bsp_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/Chassis.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/Data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Src/Gimbal.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Library/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
