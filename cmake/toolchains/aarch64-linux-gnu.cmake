# aarch64-linux-gnu cross toolchain for stableCOPS (robot/ARM builds).
#
# Requirements on the build host:
#   - the cross compiler: apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#   - an aarch64 sysroot containing the ARM builds of the runtime
#     dependencies, most importantly liblely-coapp (+ its .pc files). The
#     simplest way to get one is to copy /usr and /lib from the robot's
#     rootfs (rsync, or mount its image) into a directory on the host.
#
# Point the build at the sysroot either way:
#   export STABLECOPS_ARM_SYSROOT=/path/to/aarch64-sysroot
#   cmake --preset arm64
# or
#   cmake --preset arm64 -DCMAKE_SYSROOT=/path/to/aarch64-sysroot

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

if(NOT CMAKE_SYSROOT AND DEFINED ENV{STABLECOPS_ARM_SYSROOT})
    set(CMAKE_SYSROOT $ENV{STABLECOPS_ARM_SYSROOT})
endif()

# Libraries/headers/packages only from the sysroot; programs only from the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Make pkg-config (liblely-coapp lookup) resolve against the sysroot instead
# of the host: -I/-L results are rewritten under CMAKE_SYSROOT.
if(CMAKE_SYSROOT)
    set(ENV{PKG_CONFIG_DIR} "")
    set(ENV{PKG_CONFIG_LIBDIR}
        "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CMAKE_SYSROOT}/usr/lib/pkgconfig:${CMAKE_SYSROOT}/usr/share/pkgconfig:${CMAKE_SYSROOT}/usr/local/lib/pkgconfig")
    set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CMAKE_SYSROOT}")
endif()
