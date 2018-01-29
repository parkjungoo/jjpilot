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

# Include any dependencies generated for this target.
include src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/depend.make

# Include the progress variables for this target.
include src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/progress.make

# Include the compile flags for this target's objects.
include src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/flags.make

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/flags.make
src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj: /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/mtd.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/systemcmds__mtd.dir/mtd.c.obj   -c /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/mtd.c

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/systemcmds__mtd.dir/mtd.c.i"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/mtd.c > CMakeFiles/systemcmds__mtd.dir/mtd.c.i

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/systemcmds__mtd.dir/mtd.c.s"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/mtd.c -o CMakeFiles/systemcmds__mtd.dir/mtd.c.s

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.requires:

.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.requires

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.provides: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.requires
	$(MAKE) -f src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/build.make src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.provides.build
.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.provides

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.provides.build: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj


src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/flags.make
src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj: /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/24xxxx_mtd.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj   -c /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/24xxxx_mtd.c

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.i"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/24xxxx_mtd.c > CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.i

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.s"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd/24xxxx_mtd.c -o CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.s

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.requires:

.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.requires

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.provides: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.requires
	$(MAKE) -f src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/build.make src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.provides.build
.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.provides

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.provides.build: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj


# Object files for target systemcmds__mtd
systemcmds__mtd_OBJECTS = \
"CMakeFiles/systemcmds__mtd.dir/mtd.c.obj" \
"CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj"

# External object files for target systemcmds__mtd
systemcmds__mtd_EXTERNAL_OBJECTS =

src/systemcmds/mtd/libsystemcmds__mtd.a: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj
src/systemcmds/mtd/libsystemcmds__mtd.a: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj
src/systemcmds/mtd/libsystemcmds__mtd.a: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/build.make
src/systemcmds/mtd/libsystemcmds__mtd.a: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libsystemcmds__mtd.a"
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__mtd.dir/cmake_clean_target.cmake
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/systemcmds__mtd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/build: src/systemcmds/mtd/libsystemcmds__mtd.a

.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/build

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/requires: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/mtd.c.obj.requires
src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/requires: src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/24xxxx_mtd.c.obj.requires

.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/requires

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/clean:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd && $(CMAKE_COMMAND) -P CMakeFiles/systemcmds__mtd.dir/cmake_clean.cmake
.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/clean

src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/depend:
	cd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/ardupilot/modules/PX4Firmware /home/jj/ardupilot/modules/PX4Firmware/src/systemcmds/mtd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd /home/jj/ardupilot/build/px4-v2/modules/PX4Firmware/src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/systemcmds/mtd/CMakeFiles/systemcmds__mtd.dir/depend

