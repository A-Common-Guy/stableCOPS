cmake_minimum_required(VERSION 3.16)

project(stableCOPS LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)

add_library(stablecops_lely_project_options INTERFACE)
target_compile_features(stablecops_lely_project_options INTERFACE cxx_std_17)
target_compile_options(stablecops_lely_project_options INTERFACE
    -Wall
    -Wextra
    -Wpedantic
)

target_link_libraries(stablecops_lely_project_options INTERFACE PkgConfig::LELY_COAPP)
message(STATUS "Lely CANopen dependency found through pkg-config: liblely-coapp")

add_library(stablecops_ds402
    src/ds402/DriveController.cpp
    src/ds402/State.cpp
)
target_include_directories(stablecops_ds402 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(stablecops_ds402 PUBLIC stablecops_lely_project_options)

add_library(stablecops_runtime
    src/app/CanopenApplication.cpp
    src/lely/MotorDriver.cpp
)
target_include_directories(stablecops_runtime PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(stablecops_runtime PUBLIC stablecops_ds402)

add_executable(stablecops_master
    src/tools/stablecops_master.cpp
)
target_link_libraries(stablecops_master PRIVATE stablecops_runtime)
