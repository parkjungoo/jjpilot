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

# Include any dependencies generated for this target.
include src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/depend.make

# Include the progress variables for this target.
include src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/progress.make

# Include the compile flags for this target's objects.
include src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/flags.make

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/flags.make
src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj: /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/fmu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj -c /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/fmu.cpp

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drivers__px4fmu.dir/fmu.cpp.i"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/fmu.cpp > CMakeFiles/drivers__px4fmu.dir/fmu.cpp.i

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drivers__px4fmu.dir/fmu.cpp.s"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/fmu.cpp -o CMakeFiles/drivers__px4fmu.dir/fmu.cpp.s

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.requires:

.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.requires

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.provides: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.requires
	$(MAKE) -f src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/build.make src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.provides.build
.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.provides

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.provides.build: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj


src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/flags.make
src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj: /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/px4fmu_params.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj   -c /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/px4fmu_params.c

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.i"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/px4fmu_params.c > CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.i

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.s"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && /opt/gcc-arm-none-eabi-4_9-2015q3/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu/px4fmu_params.c -o CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.s

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.requires:

.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.requires

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.provides: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.requires
	$(MAKE) -f src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/build.make src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.provides.build
.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.provides

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.provides.build: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj


# Object files for target drivers__px4fmu
drivers__px4fmu_OBJECTS = \
"CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj" \
"CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj"

# External object files for target drivers__px4fmu
drivers__px4fmu_EXTERNAL_OBJECTS =

src/drivers/px4fmu/libdrivers__px4fmu.a: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj
src/drivers/px4fmu/libdrivers__px4fmu.a: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj
src/drivers/px4fmu/libdrivers__px4fmu.a: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/build.make
src/drivers/px4fmu/libdrivers__px4fmu.a: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libdrivers__px4fmu.a"
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4fmu.dir/cmake_clean_target.cmake
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drivers__px4fmu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/build: src/drivers/px4fmu/libdrivers__px4fmu.a

.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/build

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/requires: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/fmu.cpp.obj.requires
src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/requires: src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/px4fmu_params.c.obj.requires

.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/requires

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/clean:
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu && $(CMAKE_COMMAND) -P CMakeFiles/drivers__px4fmu.dir/cmake_clean.cmake
.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/clean

src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/depend:
	cd /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jj/MyProject/ardupilot/modules/PX4Firmware /home/jj/MyProject/ardupilot/modules/PX4Firmware/src/drivers/px4fmu /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu /home/jj/MyProject/ardupilot/build/px4-v2/modules/PX4Firmware/src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/drivers/px4fmu/CMakeFiles/drivers__px4fmu.dir/depend

