# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/jj/MyProject/ardupilot/modules/PX4Firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware

# Utility rule file for build_firmware_px4fmu-v2.

# Include the progress variables for this target.
include src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/progress.make

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2: src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4


src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4: src/firmware/nuttx/firmware_nuttx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx-px4fmu-v2-apm.px4"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-objcopy -O binary /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && /usr/bin/python /home/jj/MyProject/ardupilot/modules/PX4Firmware/Tools/px_mkfw.py --prototype /home/jj/MyProject/ardupilot/modules/PX4Firmware/Images/px4fmu-v2.prototype --git_identity /home/jj/MyProject/ardupilot/modules/PX4Firmware --parameter_xml /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/parameters.xml --airframe_xml /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/airframes.xml --image /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin > /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4

build_firmware_px4fmu-v2: src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2
build_firmware_px4fmu-v2: src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4
build_firmware_px4fmu-v2: src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/build.make

.PHONY : build_firmware_px4fmu-v2

# Rule to build all files generated by this target.
src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/build: build_firmware_px4fmu-v2

.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/build

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/clean:
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/build_firmware_px4fmu-v2.dir/cmake_clean.cmake
.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/clean

src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/depend:
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/MyProject/ardupilot/modules/PX4Firmware /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/firmware/nuttx /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/nuttx/CMakeFiles/build_firmware_px4fmu-v2.dir/depend

