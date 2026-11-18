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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Z_HEAVY_Robot_CHASIS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM, C and C++ languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
file(GLOB_RECURSE USER_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# ROBOT：工程适配层（robot_port / robot_config / robot_adapter / robot_task）
file(GLOB ROBOT_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/ROBOT/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ROBOT_SOURCES})

# SEGGER RTT：bsp_log 的输出后端（OpenOCD/DAPLink 通过 SWD 读取）
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER/SEGGER_RTT.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/Task/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/app
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/app/app_chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/app/app_safety
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Application/app/app_shooter
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_adc
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_led
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_gpio
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_mcu
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_pwm
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_spi
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_tick
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_uart
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_fdcan
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_common
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_can
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_log
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_watchdog
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/BSP/bsp_dwt
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_crc
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_kalman
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_lpf
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_quaternion
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_math
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_mecanum
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/Algorithm/alg_pid
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/bmi088
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/image_transmission
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/ix6_remote
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/referee_system
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/module_dji_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/module_dm_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/module_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/USERS/Components/module/module_shooter
    ${CMAKE_CURRENT_SOURCE_DIR}/ROBOT
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# C++ for embedded: no exceptions, no RTTI
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-use-cxa-atexit")

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/arm_cortexM7lfdp_math.lib

    # Add user defined libraries
)
