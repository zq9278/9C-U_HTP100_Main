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
set(CMAKE_PROJECT_NAME SLK-01_V1_0)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE USER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/HardWare/Src/*.*"
    # "${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER/SEGGER_RTT.c"
    # "${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER/SEGGER_SYSVIEW.c"
    # "${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER/SEGGER_RTT_printf.c"
    # "${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Sample/FreeRTOSV10/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c"
    # "${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.c"
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
    # ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Config
    # ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER
    # ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Sample/FreeRTOSV10
)

# # 定义函数：递归收集源文件和所有子目录路径
# function(collect_sources_and_include_dirs DIR)
#     # 初始化列表
#     set(LOCAL_INCLUDE_DIRS "")
#     set(LOCAL_SOURCES "")

#     # 收集所有源文件
#     file(GLOB_RECURSE SOURCES "${DIR}/*.c" "${DIR}/*.cpp" "${DIR}/*.s")
#     list(APPEND LOCAL_SOURCES ${SOURCES})

#     # 收集当前目录和子目录
#     file(GLOB SUBDIRS LIST_DIRECTORIES true "${DIR}/*")
#     foreach(SUBDIR ${SUBDIRS})
#         if(IS_DIRECTORY ${SUBDIR})
#             list(APPEND LOCAL_INCLUDE_DIRS ${SUBDIR}) # 添加当前目录
#             # 调试信息
#             message(STATUS "Adding include dir: ${SUBDIR}")
#             # 递归调用自身，处理嵌套目录
#             collect_sources_and_include_dirs(${SUBDIR})
#             list(APPEND LOCAL_INCLUDE_DIRS ${INCLUDE_DIRS})
#             list(APPEND LOCAL_SOURCES ${SOURCES})
#         endif()
#     endforeach()

#     # 去重
#     list(REMOVE_DUPLICATES LOCAL_INCLUDE_DIRS)
#     list(REMOVE_DUPLICATES LOCAL_SOURCES)

#     # 将结果返回到父作用域
#     set(INCLUDE_DIRS ${LOCAL_INCLUDE_DIRS} PARENT_SCOPE)
#     set(SOURCES ${LOCAL_SOURCES} PARENT_SCOPE)
# endfunction()

# # 收集 HardWare 文件夹
# collect_sources_and_include_dirs("${CMAKE_CURRENT_SOURCE_DIR}/HardWare")
# set(HARDWARE_SOURCES ${SOURCES})
# set(HARDWARE_INCLUDE_DIRS ${INCLUDE_DIRS})

# # 收集 SystemView 文件夹
# collect_sources_and_include_dirs("${CMAKE_CURRENT_SOURCE_DIR}/SystemView")
# set(SYSTEMVIEW_SOURCES ${SOURCES})
# set(SYSTEMVIEW_INCLUDE_DIRS ${INCLUDE_DIRS})

# # 合并所有源文件和包含路径
# list(APPEND ALL_SOURCES ${HARDWARE_SOURCES} ${SYSTEMVIEW_SOURCES})
# list(APPEND ALL_INCLUDE_DIRS ${HARDWARE_INCLUDE_DIRS} ${SYSTEMVIEW_INCLUDE_DIRS})

# # 调试输出
# message(STATUS "All sources: ${ALL_SOURCES}")
# message(STATUS "All include dirs: ${ALL_INCLUDE_DIRS}")

# # 添加到目标项目
# target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ALL_SOURCES})
# target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${ALL_INCLUDE_DIRS})



# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


# 显示代码大小
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "显示代码大小"
)
