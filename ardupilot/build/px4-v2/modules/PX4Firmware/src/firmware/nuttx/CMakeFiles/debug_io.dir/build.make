# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jj/ardupilot/modules/PX4Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware

# Utility rule file for debug_io.

# Include the progress variables for this target.
include src/firmware/nuttx/CMakeFiles/debug_io.dir/progress.make

src/firmware/nuttx/CMakeFiles/debug_io: src/firmware/nuttx/firmware_nuttx
src/firmware/nuttx/CMakeFiles/debug_io: src/firmware/nuttx/.gdbinit
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gdb /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/modules/px4iofirmware/px4io-v2

debug_io: src/firmware/nuttx/CMakeFiles/debug_io
debug_io: src/firmware/nuttx/CMakeFiles/debug_io.dir/build.make

.PHONY : debug_io

# Rule to build all files generated by this target.
src/firmware/nuttx/CMakeFiles/debug_io.dir/build: debug_io

.PHONY : src/firmware/nuttx/CMakeFiles/debug_io.dir/build

src/firmware/nuttx/CMakeFiles/debug_io.dir/clean:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/debug_io.dir/cmake_clean.cmake
.PHONY : src/firmware/nuttx/CMakeFiles/debug_io.dir/clean

src/firmware/nuttx/CMakeFiles/debug_io.dir/depend:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/ardupilot/modules/PX4Firmware /home/jj/ardupilot/modules/PX4Firmware/src/firmware/nuttx /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/CMakeFiles/debug_io.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/nuttx/CMakeFiles/debug_io.dir/depend

