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

# set(SYSVIEW_ROOT_DIR ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/SEGGER)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME gimbal)

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
    Drivers/CMSIS/Lib/GCC
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    whx_imu/Components/kalman_filter.c
    whx_imu/Components/user_lib.c
    whx_imu/Components/Devices/BMI088Middleware.c
    whx_imu/Components/Devices/BMI088driver.c
    whx_imu/Components/Devices/transfer_function.c
    whx_imu/Components/Controller/controller.c
    whx_imu/Components/Algorithm/QuaternionEKF.c
    whx_imu/Application/ins_task.c
    whx_imu/Bsp/bsp_dwt.c
    whx_imu/Bsp/bsp_PWM.c

    USER/application/Task/INS_task.c

    # ${SYSVIEW_ROOT_DIR}/SEGGER_SYSVIEW.c
    # ${SYSVIEW_ROOT_DIR}/SEGGER_SYSVIEW_Config_FreeRTOS.c
    # ${SYSVIEW_ROOT_DIR}/SEGGER_SYSVIEW_FreeRTOS.c
    # ${SYSVIEW_ROOT_DIR}/SEGGER_RTT_ASM_ARMv7M.S
    # ${SYSVIEW_ROOT_DIR}/SEGGER_RTT_printf.c
)

file(GLOB_RECURSE USER_SOURCES
        "USER/**/*.*"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    whx_imu/Components/
    whx_imu/Components/Algorithm/
    whx_imu/Components/Algorithm/Include/
    whx_imu/Components/Controller/
    whx_imu/Components/Devices/
    whx_imu/Application/
    whx_imu/Bsp/

    # ${SYSVIEW_ROOT_DIR}/include

    USER/application/Task/
    USER/application/module/gimbal/
    USER/application/module/imu/
    USER/application/callback/
    USER/components/algorithm/
    USER/components/algorithm/include/
    USER/components/bsp/
    USER/components/controller/
    USER/components/device/bmi088/
    USER/components/device/ist8310/
    USER/components/device/DM/
    USER/components/device/DT7/
    USER/components/device/DJI_Motor/
    USER/components/support/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_DSP   
    ARM_MATH_MATRIX_CHECK      
    ARM_MATH_ROUNDING  
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
    arm_cortexM4lf_math
    m
)
