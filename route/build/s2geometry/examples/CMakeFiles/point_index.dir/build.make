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
include s2geometry/examples/CMakeFiles/point_index.dir/depend.make

# Include the progress variables for this target.
include s2geometry/examples/CMakeFiles/point_index.dir/progress.make

# Include the compile flags for this target's objects.
include s2geometry/examples/CMakeFiles/point_index.dir/flags.make

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o: s2geometry/examples/CMakeFiles/point_index.dir/flags.make
s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o: /s2geometry/doc/examples/point_index.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o"
	cd /code/build/s2geometry/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/point_index.dir/point_index.cc.o -c /s2geometry/doc/examples/point_index.cc

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_index.dir/point_index.cc.i"
	cd /code/build/s2geometry/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /s2geometry/doc/examples/point_index.cc > CMakeFiles/point_index.dir/point_index.cc.i

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_index.dir/point_index.cc.s"
	cd /code/build/s2geometry/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /s2geometry/doc/examples/point_index.cc -o CMakeFiles/point_index.dir/point_index.cc.s

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.requires:

.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.requires

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.provides: s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.requires
	$(MAKE) -f s2geometry/examples/CMakeFiles/point_index.dir/build.make s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.provides.build
.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.provides

s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.provides.build: s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o


# Object files for target point_index
point_index_OBJECTS = \
"CMakeFiles/point_index.dir/point_index.cc.o"

# External object files for target point_index
point_index_EXTERNAL_OBJECTS =

s2geometry/examples/point_index: s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o
s2geometry/examples/point_index: s2geometry/examples/CMakeFiles/point_index.dir/build.make
s2geometry/examples/point_index: s2geometry/libs2testing.a
s2geometry/examples/point_index: s2geometry/libs2.so
s2geometry/examples/point_index: /usr/lib/x86_64-linux-gnu/libssl.so
s2geometry/examples/point_index: /usr/lib/x86_64-linux-gnu/libcrypto.so
s2geometry/examples/point_index: s2geometry/examples/CMakeFiles/point_index.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable point_index"
	cd /code/build/s2geometry/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_index.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
s2geometry/examples/CMakeFiles/point_index.dir/build: s2geometry/examples/point_index

.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/build

s2geometry/examples/CMakeFiles/point_index.dir/requires: s2geometry/examples/CMakeFiles/point_index.dir/point_index.cc.o.requires

.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/requires

s2geometry/examples/CMakeFiles/point_index.dir/clean:
	cd /code/build/s2geometry/examples && $(CMAKE_COMMAND) -P CMakeFiles/point_index.dir/cmake_clean.cmake
.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/clean

s2geometry/examples/CMakeFiles/point_index.dir/depend:
	cd /code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /code /s2geometry/doc/examples /code/build /code/build/s2geometry/examples /code/build/s2geometry/examples/CMakeFiles/point_index.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : s2geometry/examples/CMakeFiles/point_index.dir/depend
