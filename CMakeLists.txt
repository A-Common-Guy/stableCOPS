cmake_minimum_required(VERSION 3.16)

project(stableCOPS VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)
message(STATUS "Lely CANopen dependency found through pkg-config: liblely-coapp")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# When stableCOPS is the top-level project we build everything and install by
# default; when it is pulled into another build (add_subdirectory / FetchContent)
# only the library is built unless the parent opts back in.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(STABLECOPS_TOPLEVEL ON)
else()
    set(STABLECOPS_TOPLEVEL OFF)
endif()
option(STABLECOPS_BUILD_TOOLS "Build the stableCOPS CLI tools" ${STABLECOPS_TOPLEVEL})
option(STABLECOPS_BUILD_EXAMPLES "Build the stableCOPS examples" ${STABLECOPS_TOPLEVEL})
option(STABLECOPS_BUILD_TESTS "Build the stableCOPS unit tests" ${STABLECOPS_TOPLEVEL})
option(STABLECOPS_INSTALL "Generate stableCOPS install and CMake-package rules" ${STABLECOPS_TOPLEVEL})

# ---------------------------------------------------------------------------
# The library: one shared object that consumers link against.
#
# All sources live in a single target so the install/export is trivial and so
# the Lely dependency stays a private implementation detail. The public headers
# (MotorDrive.hpp and the ds402/app value types) do not include any Lely header,
# so consumers need neither the Lely headers nor an explicit Lely link.
# ---------------------------------------------------------------------------
add_library(stablecops SHARED
    src/log/Log.cpp
    src/ds402/DriveController.cpp
    src/ds402/Diagnostics.cpp
    src/ds402/State.cpp
    src/config/MotorConfig.cpp
    src/config/PdoMap.cpp
    src/app/CanopenApplication.cpp
    src/app/Bus.cpp
    src/app/MotorDrive.cpp
    src/app/RealtimeScheduling.cpp
    src/lely/MotorDriver.cpp
)
add_library(stableCOPS::stablecops ALIAS stablecops)

target_include_directories(stablecops
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_compile_features(stablecops PUBLIC cxx_std_17)
target_compile_options(stablecops PRIVATE -Wall -Wextra -Wpedantic)

# PRIVATE: because the library is shared and its public headers don't expose
# Lely/pthread types, these are not propagated to consumers. libstablecops.so
# records liblely-coapp as a runtime dependency (DT_NEEDED); it must be present
# on the target at load time.
target_link_libraries(stablecops PRIVATE PkgConfig::LELY_COAPP Threads::Threads)

set_target_properties(stablecops PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ---------------------------------------------------------------------------
# In-tree tools and examples (skipped by default when embedded as a subproject).
# ---------------------------------------------------------------------------
if(STABLECOPS_BUILD_TOOLS)
    add_executable(stablecops_master src/tools/stablecops_master.cpp)
    target_link_libraries(stablecops_master PRIVATE stablecops)

    add_executable(stablecops_commissiond src/tools/stablecops_commissiond.cpp)
    target_include_directories(stablecops_commissiond PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    target_link_libraries(stablecops_commissiond PRIVATE stablecops)
endif()

if(STABLECOPS_BUILD_TESTS)
    enable_testing()
    add_executable(stablecops_tests tests/stablecops_tests.cpp)
    target_include_directories(stablecops_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
    target_link_libraries(stablecops_tests PRIVATE stablecops)
    add_test(NAME stablecops_tests COMMAND stablecops_tests)
endif()

if(STABLECOPS_BUILD_EXAMPLES)
    # Every example uses only the Lely-free MotorDrive API.
    set(STABLECOPS_EXAMPLES
        pdo_feedback_monitor
        enable_and_hold
        csp_position
        csv_velocity
        cst_torque
        pp_move
        pv_velocity
        pt_torque
        multi_drive
    )
    foreach(example ${STABLECOPS_EXAMPLES})
        add_executable(${example} examples/${example}.cpp)
        set_target_properties(${example} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
        )
        target_link_libraries(${example} PRIVATE stablecops)
    endforeach()

    # Leg-specific tooling lives under leg/ to keep it separate from the generic
    # examples. zero_leg is CAN-only (no EtherCAT); the EtherCAT-backed leg tools
    # are built in the STABLECOPS_BUILD_ECAT block below.
    add_executable(zero_leg leg/examples/zero_leg.cpp)
    set_target_properties(zero_leg PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(zero_leg PRIVATE stablecops)

    # dump_config is CAN-only and read-only: it SDO-reads the configuration
    # parameters of both drives in a leg and prints them side by side.
    add_executable(dump_config leg/examples/dump_config.cpp)
    set_target_properties(dump_config PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(dump_config PRIVATE stablecops)
endif()

# ---------------------------------------------------------------------------
# Optional closed-chain leg support: the vendored neura_mapper_library plus the
# joint-space ankle control library (leg_control) built on top of it. CAN only,
# no EtherCAT required. Off by default because it pulls Eigen via FetchContent
# (if not already installed) and compiles the vendored mapper.
#   cmake ... -DSTABLECOPS_BUILD_MAPPER=ON
# Select the ankle hardware generation with -DSTABLECOPS_ANKLE_GENERATION=gen3-2
# (default gen3-2 for the new leg; gen3-1 is the older prismatic ankle).
# ---------------------------------------------------------------------------
option(STABLECOPS_BUILD_MAPPER "Build the closed-chain ankle mapper and leg control library" OFF)
set(STABLECOPS_ANKLE_GENERATION "gen3-2" CACHE STRING "Ankle LUT generation (gen3-1 or gen3-2)")

if(STABLECOPS_BUILD_MAPPER)
    find_package(Eigen3 QUIET NO_MODULE)
    if(NOT TARGET Eigen3::Eigen)
        message(STATUS "Eigen3 not found; fetching it for the mapper")
        include(FetchContent)
        set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
        set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
        set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(Eigen3
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 3.4.0)
        FetchContent_MakeAvailable(Eigen3)
    endif()

    set(MAPPER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/leg/4ne1_closed_chain_mapping/deployment)
    # Compile the mapper sources directly (its own CMakeLists is a standalone
    # project); this lets us control the ankle generation and Eigen source.
    add_library(leg_mapper STATIC
        ${MAPPER_DIR}/neura_mapper_library/src/mapper.cpp
        ${MAPPER_DIR}/src/lookup_table.cpp)
    target_include_directories(leg_mapper
        PUBLIC
            ${MAPPER_DIR}/neura_mapper_library/include
            ${MAPPER_DIR}/include
        PRIVATE
            ${MAPPER_DIR}/neura_mapper_library)
    target_link_libraries(leg_mapper PUBLIC Eigen3::Eigen)
    target_compile_features(leg_mapper PUBLIC cxx_std_17)
    if(STABLECOPS_ANKLE_GENERATION STREQUAL "gen3-2")
        target_compile_definitions(leg_mapper PRIVATE ANKLE_GEN3_2)
    elseif(NOT STABLECOPS_ANKLE_GENERATION STREQUAL "gen3-1")
        message(FATAL_ERROR "STABLECOPS_ANKLE_GENERATION must be gen3-1 or gen3-2")
    endif()

    # Joint-space ankle control over CAN: AnkleLeg owns the two MotorDrives of
    # one leg and runs the mapper-based PD torque loop (CST) on an internal
    # thread. All feedback is derived from the CAN drives through the mapper.
    add_library(leg_control STATIC
        leg/src/AnkleLeg.cpp)
    target_include_directories(leg_control
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/leg/include)
    target_link_libraries(leg_control PUBLIC leg_mapper stablecops)
    target_compile_features(leg_control PUBLIC cxx_std_17)
    target_compile_options(leg_control PRIVATE -Wall -Wextra -Wpedantic)

    if(STABLECOPS_BUILD_EXAMPLES)
        # Demo: both ankles (right on can0, left on can1) under interactive
        # joint-space PD torque control, CAN feedback only (no EtherCAT).
        add_executable(two_leg_demo leg/examples/two_leg_demo.cpp)
        set_target_properties(two_leg_demo PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
        target_link_libraries(two_leg_demo PRIVATE leg_control)
    endif()
endif()

# ---------------------------------------------------------------------------
# Optional EtherCAT integration (the vendored motion-faster library), for mixed
# CAN + EtherCAT applications - e.g. reading EtherCAT encoders while moving CAN
# motors. Off by default because motion-faster pulls SOEM + yaml-cpp over the
# network via FetchContent and requires CMake >= 3.28 (its own floor).
# Enable with: cmake --preset default -DSTABLECOPS_BUILD_ECAT=ON
# ---------------------------------------------------------------------------
option(STABLECOPS_BUILD_ECAT "Build the EtherCAT (motion-faster) examples" OFF)

if(STABLECOPS_BUILD_ECAT)
    # Build only motion-faster's library (ecat_core), not its own examples.
    set(MOTION_FASTER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory(mm/motion-faster)

    # EtherCAT-only: print the auxiliary (load) encoder of every EtherCAT drive.
    add_executable(motion_faster_encoder examples/motion_faster_encoder.cpp)
    set_target_properties(motion_faster_encoder PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(motion_faster_encoder PRIVATE ecat_core)

    # Combined: read EtherCAT encoders (motion-faster) while moving a CAN motor
    # (stableCOPS MotorDrive), both on their own RT/loop threads.
    add_executable(motion_faster_can_move examples/motion_faster_can_move.cpp)
    set_target_properties(motion_faster_can_move PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(motion_faster_can_move PRIVATE ecat_core stablecops)

    # Validation: replay a recorded joint trajectory on the two CAN drives (CSP)
    # while logging the foot pitch/roll measured by the two EtherCAT encoders
    # against the expected pitch/roll from the trajectory CSV.
    add_executable(validate_trajectory leg/examples/validate_trajectory.cpp)
    set_target_properties(validate_trajectory PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(validate_trajectory PRIVATE ecat_core stablecops)

    # Calibration: drive both CAN joints to their NVM zero and latch the two
    # EtherCAT encoder offsets there (saved to a file for capture_rom).
    add_executable(capture_ecat_zero leg/examples/capture_ecat_zero.cpp)
    set_target_properties(capture_ecat_zero PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(capture_ecat_zero PRIVATE ecat_core stablecops)

    # Range-of-motion: read the EtherCAT encoders only (CAN drives left
    # de-energised so the joint backdrives freely) and log centered roll/pitch.
    add_executable(capture_rom leg/examples/capture_rom.cpp)
    set_target_properties(capture_rom PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
    target_link_libraries(capture_rom PRIVATE ecat_core)

    # -----------------------------------------------------------------------
    # EtherCAT-validated mapper tools (the mapper itself is built in the
    # STABLECOPS_BUILD_MAPPER block above; these additionally read the EtherCAT
    # foot encoders for independent validation).
    #   cmake ... -DSTABLECOPS_BUILD_ECAT=ON -DSTABLECOPS_BUILD_MAPPER=ON
    # -----------------------------------------------------------------------
    if(STABLECOPS_BUILD_MAPPER)
        add_executable(validate_mapper leg/examples/validate_mapper.cpp)
        set_target_properties(validate_mapper PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
        target_link_libraries(validate_mapper PRIVATE leg_mapper ecat_core stablecops)

        # Interactive joint-space PD torque control through the mapper's torque
        # map (CST), validated with the EtherCAT encoders, telemetry to PlotJuggler.
        add_executable(ankle_pd_torque leg/examples/ankle_pd_torque.cpp)
        set_target_properties(ankle_pd_torque PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)
        target_link_libraries(ankle_pd_torque PRIVATE leg_mapper ecat_core stablecops)
    endif()
endif()

# ---------------------------------------------------------------------------
# Install + CMake package export, so a consumer can:
#   find_package(stableCOPS REQUIRED)
#   target_link_libraries(app PRIVATE stableCOPS::stablecops)
# ---------------------------------------------------------------------------
if(STABLECOPS_INSTALL)
install(TARGETS stablecops
    EXPORT stableCOPSTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT stableCOPSTargets
    FILE stableCOPSTargets.cmake
    NAMESPACE stableCOPS::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stableCOPS
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stableCOPSConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/stableCOPSConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stableCOPS
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/stableCOPSConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/stableCOPSConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/stableCOPSConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/stableCOPS
)

# Runtime data the library loads by path at boot (the generated master DCF, the
# PDO summary the cyclic layout is built from, and the motor profiles/EDS).
# Installed as sample data under the datadir; integrators point MotorConfig's
# master_dcf_path / summary_path at their own copies.
install(DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/dcf
        ${CMAKE_CURRENT_SOURCE_DIR}/generated
        ${CMAKE_CURRENT_SOURCE_DIR}/config
    DESTINATION ${CMAKE_INSTALL_DATADIR}/stablecops
)
endif()  # STABLECOPS_INSTALL
