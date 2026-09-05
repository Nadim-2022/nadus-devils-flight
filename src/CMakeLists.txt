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
set(PICO_BOARD pico CACHE STRING "Board type")

# Source files
add_executable(${PROJECT_NAME}
        main.cpp
        receiver/Elrs.cpp

)


# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/../inc
        config
)

# Turns ON the DEBUG macro for all files in 'your_project_name'
target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG)


# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall
        -Wno-format # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile> # remove warnings about volatile compound expressions
        -Wno-psabi
)

# Linked libraries
target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_i2c
        hardware_dma
        FreeRTOS-Kernel-Heap4
        hardware_adc
        hardware_uart
        hardware_spi
)

pico_add_extra_outputs(${PROJECT_NAME})


# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)
