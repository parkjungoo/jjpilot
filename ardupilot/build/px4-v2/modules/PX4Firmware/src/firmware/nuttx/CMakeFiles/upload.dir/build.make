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

# Utility rule file for upload.

# Include the progress variables for this target.
include src/firmware/nuttx/CMakeFiles/upload.dir/progress.make

src/firmware/nuttx/CMakeFiles/upload: src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "uploading /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4"
	/usr/bin/python /home/jj/ardupilot/modules/PX4Firmware/Tools/px_uploader.py --port "/dev/serial/by-id/*3D_Robotics*,/dev/serial/by-id/*_PX4_*,/dev/serial/by-id/*_Autopilot*,/dev/serial/by-id/*Bitcraze*" /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4

src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4: src/firmware/nuttx/firmware_nuttx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating nuttx-px4fmu-v2-apm.px4"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-objcopy -O binary /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && /usr/bin/python /home/jj/ardupilot/modules/PX4Firmware/Tools/px_mkfw.py --prototype /home/jj/ardupilot/modules/PX4Firmware/Images/px4fmu-v2.prototype --git_identity /home/jj/ardupilot/modules/PX4Firmware --parameter_xml /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/parameters.xml --airframe_xml /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/airframes.xml --image /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/firmware_nuttx.bin > /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4

upload: src/firmware/nuttx/CMakeFiles/upload
upload: src/firmware/nuttx/nuttx-px4fmu-v2-apm.px4
upload: src/firmware/nuttx/CMakeFiles/upload.dir/build.make

.PHONY : upload

# Rule to build all files generated by this target.
src/firmware/nuttx/CMakeFiles/upload.dir/build: upload

.PHONY : src/firmware/nuttx/CMakeFiles/upload.dir/build

src/firmware/nuttx/CMakeFiles/upload.dir/clean:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx && $(CMAKE_COMMAND) -P CMakeFiles/upload.dir/cmake_clean.cmake
.PHONY : src/firmware/nuttx/CMakeFiles/upload.dir/clean

src/firmware/nuttx/CMakeFiles/upload.dir/depend:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/ardupilot/modules/PX4Firmware /home/jj/ardupilot/modules/PX4Firmware/src/firmware/nuttx /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/firmware/nuttx/CMakeFiles/upload.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/firmware/nuttx/CMakeFiles/upload.dir/depend

