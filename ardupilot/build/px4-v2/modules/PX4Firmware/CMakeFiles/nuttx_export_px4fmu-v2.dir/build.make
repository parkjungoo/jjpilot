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

# Utility rule file for nuttx_export_px4fmu-v2.

# Include the progress variables for this target.
include CMakeFiles/nuttx_export_px4fmu-v2.dir/progress.make

CMakeFiles/nuttx_export_px4fmu-v2: nuttx_export_px4fmu-v2.stamp


nuttx_export_px4fmu-v2.stamp: px4fmu-v2.export
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating nuttx_export_px4fmu-v2.stamp"
	/bin/rm -rf /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx-export
	/usr/bin/unzip -q /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2.export -d /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX
	/usr/bin/touch nuttx_export_px4fmu-v2.stamp

px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/include/board.h
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/include/nsh_romfsimg.h
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/nsh/Make.defs
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/nsh/appconfig
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/nsh/defconfig
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/nsh/setenv.sh
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/scripts/ld.script
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/src/Makefile
px4fmu-v2.export: /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2/src/empty.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating px4fmu-v2.export"
	/bin/echo Configuring NuttX for px4fmu-v2
	/usr/bin/make --no-print-directory -C/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx -r --quiet distclean
	/bin/cp -r /home/jj/MyProject/ardupilot/modules/PX4Firmware/nuttx-configs/px4fmu-v2 /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx/configs
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx/tools && /bin/sh ./configure.sh px4fmu-v2/nsh
	/bin/echo Exporting NuttX for px4fmu-v2
	/usr/bin/make --no-print-directory --quiet -C /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx -j4 -r CONFIG_ARCH_BOARD=px4fmu-v2 export > /dev/null
	/bin/cp -r /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2/NuttX/nuttx/nuttx-export.zip /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/px4fmu-v2.export

nuttx_export_px4fmu-v2: CMakeFiles/nuttx_export_px4fmu-v2
nuttx_export_px4fmu-v2: nuttx_export_px4fmu-v2.stamp
nuttx_export_px4fmu-v2: px4fmu-v2.export
nuttx_export_px4fmu-v2: CMakeFiles/nuttx_export_px4fmu-v2.dir/build.make

.PHONY : nuttx_export_px4fmu-v2

# Rule to build all files generated by this target.
CMakeFiles/nuttx_export_px4fmu-v2.dir/build: nuttx_export_px4fmu-v2

.PHONY : CMakeFiles/nuttx_export_px4fmu-v2.dir/build

CMakeFiles/nuttx_export_px4fmu-v2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nuttx_export_px4fmu-v2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nuttx_export_px4fmu-v2.dir/clean

CMakeFiles/nuttx_export_px4fmu-v2.dir/depend:
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/MyProject/ardupilot/modules/PX4Firmware /home/jj/MyProject/ardupilot/modules/PX4Firmware /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles/nuttx_export_px4fmu-v2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/nuttx_export_px4fmu-v2.dir/depend
