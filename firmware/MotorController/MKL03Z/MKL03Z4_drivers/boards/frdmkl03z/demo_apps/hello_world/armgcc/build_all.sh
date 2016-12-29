#!/bin/sh
echo
echo "--------------------- DEBUG BUILD ------------------------"
echo
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -Wno-deprecated -DCMAKE_BUILD_TYPE=Debug  .
make -j4
echo
echo "--------------------- RELEASE BUILD ------------------------"
echo
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -Wno-deprecated -DCMAKE_BUILD_TYPE=Release  .
make -j4
