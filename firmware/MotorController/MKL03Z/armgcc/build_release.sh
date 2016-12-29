#!/bin/sh
cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "Unix Makefiles" -Wno-deprecated -DCMAKE_BUILD_TYPE=Release  .
make -j4
