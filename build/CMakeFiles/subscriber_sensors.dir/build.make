# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/joao/PESTA/control_system/v3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joao/PESTA/control_system/v3/build

# Include any dependencies generated for this target.
include CMakeFiles/subscriber_sensors.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/subscriber_sensors.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/subscriber_sensors.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/subscriber_sensors.dir/flags.make

CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o: CMakeFiles/subscriber_sensors.dir/flags.make
CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o: ../subscriber_sensors.cpp
CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o: CMakeFiles/subscriber_sensors.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joao/PESTA/control_system/v3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o -MF CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o.d -o CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o -c /home/joao/PESTA/control_system/v3/subscriber_sensors.cpp

CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joao/PESTA/control_system/v3/subscriber_sensors.cpp > CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.i

CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joao/PESTA/control_system/v3/subscriber_sensors.cpp -o CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.s

CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o: CMakeFiles/subscriber_sensors.dir/flags.make
CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o: /home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp
CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o: CMakeFiles/subscriber_sensors.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joao/PESTA/control_system/v3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o -MF CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o.d -o CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o -c /home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp

CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp > CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.i

CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp -o CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.s

# Object files for target subscriber_sensors
subscriber_sensors_OBJECTS = \
"CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o" \
"CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o"

# External object files for target subscriber_sensors
subscriber_sensors_EXTERNAL_OBJECTS =

subscriber_sensors: CMakeFiles/subscriber_sensors.dir/subscriber_sensors.cpp.o
subscriber_sensors: CMakeFiles/subscriber_sensors.dir/home/joao/PESTA/2._Node_initialization_and_startup/platform_linux.cpp.o
subscriber_sensors: CMakeFiles/subscriber_sensors.dir/build.make
subscriber_sensors: /usr/local/lib/libuavcan.a
subscriber_sensors: CMakeFiles/subscriber_sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joao/PESTA/control_system/v3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable subscriber_sensors"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/subscriber_sensors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/subscriber_sensors.dir/build: subscriber_sensors
.PHONY : CMakeFiles/subscriber_sensors.dir/build

CMakeFiles/subscriber_sensors.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/subscriber_sensors.dir/cmake_clean.cmake
.PHONY : CMakeFiles/subscriber_sensors.dir/clean

CMakeFiles/subscriber_sensors.dir/depend:
	cd /home/joao/PESTA/control_system/v3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joao/PESTA/control_system/v3 /home/joao/PESTA/control_system/v3 /home/joao/PESTA/control_system/v3/build /home/joao/PESTA/control_system/v3/build /home/joao/PESTA/control_system/v3/build/CMakeFiles/subscriber_sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/subscriber_sensors.dir/depend

