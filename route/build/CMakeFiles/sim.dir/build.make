# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /code/build

# Include any dependencies generated for this target.
include CMakeFiles/sim.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sim.dir/flags.make

CMakeFiles/sim.dir/sim.cpp.o: CMakeFiles/sim.dir/flags.make
CMakeFiles/sim.dir/sim.cpp.o: ../sim.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sim.dir/sim.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sim.dir/sim.cpp.o -c /code/sim.cpp

CMakeFiles/sim.dir/sim.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sim.dir/sim.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/sim.cpp > CMakeFiles/sim.dir/sim.cpp.i

CMakeFiles/sim.dir/sim.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sim.dir/sim.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/sim.cpp -o CMakeFiles/sim.dir/sim.cpp.s

CMakeFiles/sim.dir/sim.cpp.o.requires:

.PHONY : CMakeFiles/sim.dir/sim.cpp.o.requires

CMakeFiles/sim.dir/sim.cpp.o.provides: CMakeFiles/sim.dir/sim.cpp.o.requires
	$(MAKE) -f CMakeFiles/sim.dir/build.make CMakeFiles/sim.dir/sim.cpp.o.provides.build
.PHONY : CMakeFiles/sim.dir/sim.cpp.o.provides

CMakeFiles/sim.dir/sim.cpp.o.provides.build: CMakeFiles/sim.dir/sim.cpp.o


# Object files for target sim
sim_OBJECTS = \
"CMakeFiles/sim.dir/sim.cpp.o"

# External object files for target sim
sim_EXTERNAL_OBJECTS =

sim: CMakeFiles/sim.dir/sim.cpp.o
sim: CMakeFiles/sim.dir/build.make
sim: libopt.so
sim: libsimulation.so
sim: s2geometry/libs2.so
sim: /usr/lib/x86_64-linux-gnu/libssl.so
sim: /usr/lib/x86_64-linux-gnu/libcrypto.so
sim: /nomad/builds/release/lib/libnomad.so
sim: CMakeFiles/sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sim.dir/build: sim

.PHONY : CMakeFiles/sim.dir/build

CMakeFiles/sim.dir/requires: CMakeFiles/sim.dir/sim.cpp.o.requires

.PHONY : CMakeFiles/sim.dir/requires

CMakeFiles/sim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sim.dir/clean

CMakeFiles/sim.dir/depend:
	cd /code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /code /code /code/build /code/build /code/build/CMakeFiles/sim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sim.dir/depend

