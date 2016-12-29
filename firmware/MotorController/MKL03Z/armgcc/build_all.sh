#!/bin/sh
echo
echo "--------------------- DEBUG BUILD ------------------------"
echo
cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "Unix Makefiles" -Wno-deprecated --no-warn-unused-cli -DCMAKE_BUILD_TYPE=Debug  .
make -j4
echo
echo "--------------------- RELEASE BUILD ------------------------"
echo
cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "Unix Makefiles" -Wno-deprecated --no-warn-unused-cli -DCMAKE_BUILD_TYPE=Release  .
make -j4
