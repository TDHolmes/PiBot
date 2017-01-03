# Overview

This folder is for the motor controller co-processor on the PyqoBot project. This processor is designed to offload
much of the processing required to keep track of motor speed, accelleration, and position from the encoders. The folder
structure is as follows:

    MotorController
    |- MKL03Z4_drivers
    |- armgcc
    |  |- debug
    |  |- release

The `armgcc` folder contains the build system for this firmware. The `debug` and `release` folders contained compiled .elf
files. `debug` has extra assertions and debug prints as well as a lower compiler optimization setting. `release` does not
have these debug options enabled. The MKL03Z4 contains NXP driver code and linker scripts that are used in this project. All 
other source files in the main `MotorController` folder is project source code specific to this project.

# Build Instructions

This project utilizes cmake to generate a makefile which then is compiled. The files required to compile this firmware is in the folder 
`./armgcc'. The important files for the buildsystem are as follows:

    * CMakeLists.txt: File that sets GCC compiler and linker flags as well as specifying which source files to compile.
    * armgcc.cmake: Main cmake file that sets up the compiler and environment.
    * build.py: python command line script for compiling the firmware

Cmake can be called from the commandline by using some form of the following command:

    cmake -DCMAKE_TOOLCHAIN_FILE="armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release  .

Alternatively, and preferred, call build.py to do this for you. The commandline options are as follows:

    ./build.py -h
    usage: build.py [-h] [-f FLASH_SIZE] [-s] [-g GCC_DIR] build

    positional arguments:
      build                 Type of build to run. Options are: 'debug', 'release',
                            'all', and 'clean'.

    optional arguments:
      -h, --help            show this help message and exit
      -f FLASH_SIZE, --flash-size FLASH_SIZE
                            total flash size available on the chip in Kibibytes.
                            Defaults to 32k.
      -s, --show-usage      Outputs size information for individual symbols in the
                            builds output elf file.
      -g GCC_DIR, --gcc-dir GCC_DIR
                            Directory for armgcc compiler.


