set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER /opt/cross-pi-gcc/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /opt/cross-pi-gcc/bin/arm-linux-gnueabihf-g++)
set(CMAKE_SYSROOT /opt/raspi-rootfs)

# https://github.com/eProsima/Fast-DDS/issues/1262
set(CMAKE_CXX_FLAGS "-latomic")

set(CMAKE_FIND_ROOT_PATH /home/pi/)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(PYTHON_SOABI cpython-39-arm-linux-gnueabihf)

# https://github.com/foonathan/memory/pull/60
set(CMAKE_CROSSCOMPILING_EMULATOR /usr/bin/qemu-arm-static)
