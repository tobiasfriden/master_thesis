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
include CMakeFiles/opt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/opt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/opt.dir/flags.make

CMakeFiles/opt.dir/opt.cpp.o: CMakeFiles/opt.dir/flags.make
CMakeFiles/opt.dir/opt.cpp.o: ../opt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/opt.dir/opt.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opt.dir/opt.cpp.o -c /code/opt.cpp

CMakeFiles/opt.dir/opt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opt.dir/opt.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /code/opt.cpp > CMakeFiles/opt.dir/opt.cpp.i

CMakeFiles/opt.dir/opt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opt.dir/opt.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /code/opt.cpp -o CMakeFiles/opt.dir/opt.cpp.s

CMakeFiles/opt.dir/opt.cpp.o.requires:

.PHONY : CMakeFiles/opt.dir/opt.cpp.o.requires

CMakeFiles/opt.dir/opt.cpp.o.provides: CMakeFiles/opt.dir/opt.cpp.o.requires
	$(MAKE) -f CMakeFiles/opt.dir/build.make CMakeFiles/opt.dir/opt.cpp.o.provides.build
.PHONY : CMakeFiles/opt.dir/opt.cpp.o.provides

CMakeFiles/opt.dir/opt.cpp.o.provides.build: CMakeFiles/opt.dir/opt.cpp.o


# Object files for target opt
opt_OBJECTS = \
"CMakeFiles/opt.dir/opt.cpp.o"

# External object files for target opt
opt_EXTERNAL_OBJECTS =

libopt.so: CMakeFiles/opt.dir/opt.cpp.o
libopt.so: CMakeFiles/opt.dir/build.make
libopt.so: libsimulation.so
libopt.so: /nomad/builds/release/lib/libnomad.so
libopt.so: s2geometry/libs2.so
libopt.so: /usr/lib/x86_64-linux-gnu/libssl.so
libopt.so: /usr/lib/x86_64-linux-gnu/libcrypto.so
libopt.so: CMakeFiles/opt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libopt.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/opt.dir/build: libopt.so

.PHONY : CMakeFiles/opt.dir/build

CMakeFiles/opt.dir/requires: CMakeFiles/opt.dir/opt.cpp.o.requires

.PHONY : CMakeFiles/opt.dir/requires

CMakeFiles/opt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/opt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/opt.dir/clean

CMakeFiles/opt.dir/depend:
	cd /code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /code /code /code/build /code/build /code/build/CMakeFiles/opt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/opt.dir/depend

