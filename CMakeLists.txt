# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.0)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.20)

# Language settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default board target (override with -DPICO_BOARD=...)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Import Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Import FreeRTOS Kernel Port
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/external/FreeRTOS-Kernel)

if(PICO_BOARD MATCHES "pico_w" OR PICO_BOARD MATCHES "pico")
    include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
elseif(PICO_BOARD MATCHES "pico2_w" OR PICO_BOARD MATCHES "pico2")
    include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ/FreeRTOS_Kernel_import.cmake)
endif()

# Define project
project(nadus-devils-flight LANGUAGES C CXX ASM)

# Initialize Pico SDK
pico_sdk_init()

# 1. Build external sensor & driver libraries
#add_subdirectory(external)

# 2. Build main firmware and link everything
add_subdirectory(src)