#!/bin/bash

if [ -d "build" ]; then
    sudo rm -r build
fi
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-rpi3-linux-gnu.cmake
cmake --build build -j

scp build/main rpi:~