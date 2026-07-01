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

# Add optimization settings for different build types.
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -DNDEBUG")
# Set the project name
set(CMAKE_PROJECT_NAME 9C-HTP100-main)
set(JLINK_DEVICE "STM32G070CB" CACHE STRING "SEGGER J-Link target device")
set(JLINK_INTERFACE "SWD" CACHE STRING "SEGGER J-Link target interface")
set(JLINK_SPEED_KHZ "4000" CACHE STRING "SEGGER J-Link speed in kHz")
set(JLINK_GDB_PORT "2331" CACHE STRING "SEGGER J-Link GDB server port")
set(PRODUCT_MODEL "9C-U-HTP100" CACHE STRING "Target product model")#产品名称
set_property(CACHE PRODUCT_MODEL PROPERTY STRINGS "9C-U-HTP100S" "9C-U-HTP100")
set(DEFAULT_LANGUAGE_MODE "0" CACHE STRING "Default language mode: 0=Chinese, 1=English")
# Manually maintain the software version here. YYYYMMDD format is recommended.
set(SOFTWARE_VERSION "20260614")

if(PRODUCT_MODEL STREQUAL "9C-U-HTP100S")
        set(PRODUCT_MODEL_DEFINE PRODUCT_MODEL_9C_U_HTP100S)
elseif(PRODUCT_MODEL STREQUAL "9C-U-HTP100")
        set(PRODUCT_MODEL_DEFINE PRODUCT_MODEL_9C_U_HTP100)
else()
        message(FATAL_ERROR "Unsupported PRODUCT_MODEL: ${PRODUCT_MODEL}")
endif()

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)



# Core project settings
project(${CMAKE_PROJECT_NAME})
# Enable CMake support for ASM and C languages
enable_language(C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        #HardWare/Src/BQ27441_hal.c
        HardWare/Inc)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)
file(GLOB_RECURSE USER_SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/HardWare/Src/*.*"
)
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        ${USER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        ${CMAKE_CURRENT_SOURCE_DIR}/HardWare/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
        ${PRODUCT_MODEL_DEFINE}
        PRODUCT_MODEL_NAME="${PRODUCT_MODEL}"
        SOFTWARE_VERSION=${SOFTWARE_VERSION}
        DEFAULT_LANGUAGE_MODE=${DEFAULT_LANGUAGE_MODE}
        # LOG_SWITCH_OF_BQ27441
        # BQ27441_RA_TABLE_LOG
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)

# Keep float printf enabled for every build type.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u
        _printf_float
)
# Print final image size
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        COMMENT "Print image size"
)
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
        COMMENT "Converting ELF to HEX"
)

add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin
        COMMENT "Converting ELF to BIN"
)

set(STM32CUBE_BUNDLE_ROOTS "")
foreach(STM32CUBE_BUNDLE_ROOT IN ITEMS
        "$ENV{STM32CUBE_BUNDLE_PATH}"
        "$ENV{CUBE_BUNDLE_PATH}"
        "$ENV{HOME}/Library/Application Support/stm32cube/bundles"
        "$ENV{LOCALAPPDATA}/stm32cube/bundles"
        "$ENV{HOME}/.stm32cube/bundles"
)
    if(EXISTS "${STM32CUBE_BUNDLE_ROOT}")
        list(APPEND STM32CUBE_BUNDLE_ROOTS "${STM32CUBE_BUNDLE_ROOT}")
    endif()
endforeach()

set(JLINK_BIN_HINTS "")
foreach(STM32CUBE_BUNDLE_ROOT IN LISTS STM32CUBE_BUNDLE_ROOTS)
    file(GLOB JLINK_BIN_HINTS_FOR_ROOT
            "${STM32CUBE_BUNDLE_ROOT}/jlink-gdbserver/*/bin"
    )
    list(APPEND JLINK_BIN_HINTS ${JLINK_BIN_HINTS_FOR_ROOT})
endforeach()
list(SORT JLINK_BIN_HINTS COMPARE NATURAL ORDER DESCENDING)

find_program(JLINK_EXE NAMES JLinkExe HINTS ${JLINK_BIN_HINTS} NO_DEFAULT_PATH)
find_program(JLINK_GDB_SERVER_EXE NAMES JLinkGDBServerCLExe HINTS ${JLINK_BIN_HINTS} NO_DEFAULT_PATH)

if(JLINK_EXE)
    message(STATUS "J-Link Commander: ${JLINK_EXE}")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/jlink")
    file(GENERATE
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/jlink/flash-$<CONFIG>.jlink"
            CONTENT "r\nh\nloadfile \"$<TARGET_FILE:${CMAKE_PROJECT_NAME}>\"\nr\ng\nexit\n"
    )

    add_custom_target(jlink_flash
            COMMAND "${JLINK_EXE}"
            -NoGui 1
            -ExitOnError 1
            -device ${JLINK_DEVICE}
            -if ${JLINK_INTERFACE}
            -speed ${JLINK_SPEED_KHZ}
            -autoconnect 1
            -CommandFile "${CMAKE_CURRENT_BINARY_DIR}/jlink/flash-$<CONFIG>.jlink"
            DEPENDS ${CMAKE_PROJECT_NAME}
            USES_TERMINAL
            COMMENT "Flash ${CMAKE_PROJECT_NAME} with SEGGER J-Link"
    )
endif()

if(JLINK_GDB_SERVER_EXE)
    message(STATUS "J-Link GDB Server: ${JLINK_GDB_SERVER_EXE}")
    add_custom_target(jlink_gdbserver
            COMMAND "${JLINK_GDB_SERVER_EXE}"
            -if ${JLINK_INTERFACE}
            -device ${JLINK_DEVICE}
            -speed ${JLINK_SPEED_KHZ}
            -port ${JLINK_GDB_PORT}
            -swoport 2332
            -telnetport 2333
            -singlerun
            -nogui
            USES_TERMINAL
            COMMENT "Start SEGGER J-Link GDB Server on port ${JLINK_GDB_PORT}"
    )
endif()
