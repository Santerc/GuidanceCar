cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Cretnas)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)



# Core project settings
project(${CMAKE_PROJECT_NAME})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

message("单片机课设循迹小车项目，其名为" ${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})



include_directories()
link_directories()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}

        )

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# ------ 添加源文件 ------
file(GLOB_RECURSE source_c CONFIGURE_DEPENDS
        User/*.cpp User/*.h
        BSP/*.cpp BSP/*.h
        Device/*.cpp Device/*.h
        Library/*.cpp Library/*.h
        Library/*.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${source_c}
)
# ------ 添加头文件目录 ------
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        User/include
        BSP/include
        Device/include
        third/CMSIS-DSP/1.15.0/include
        third/IMU/include
        third/fsm
        Library/algorithm

)



# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/third/IMU/lib
        # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/third/IMU/lib/libalgobsx.a
        # Add user defined libraries
)
