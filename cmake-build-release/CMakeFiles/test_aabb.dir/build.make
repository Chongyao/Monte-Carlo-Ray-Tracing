# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/zcy/Downloads/clion-2018.3.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zcy/Downloads/clion-2018.3.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release

# Include any dependencies generated for this target.
include CMakeFiles/test_aabb.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_aabb.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_aabb.dir/flags.make

CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o: CMakeFiles/test_aabb.dir/flags.make
CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o: ../src/test/test_aabb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o -c /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/src/test/test_aabb.cc

CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/src/test/test_aabb.cc > CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.i

CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/src/test/test_aabb.cc -o CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.s

# Object files for target test_aabb
test_aabb_OBJECTS = \
"CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o"

# External object files for target test_aabb
test_aabb_EXTERNAL_OBJECTS =

bin/test_aabb: CMakeFiles/test_aabb.dir/src/test/test_aabb.cc.o
bin/test_aabb: CMakeFiles/test_aabb.dir/build.make
bin/test_aabb: lib/libMC_Source.a
bin/test_aabb: CMakeFiles/test_aabb.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/test_aabb"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_aabb.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_aabb.dir/build: bin/test_aabb

.PHONY : CMakeFiles/test_aabb.dir/build

CMakeFiles/test_aabb.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_aabb.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_aabb.dir/clean

CMakeFiles/test_aabb.dir/depend:
	cd /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release /home/zcy/Documents/projects/Monte-Carlo-Ray-Tracing/cmake-build-release/CMakeFiles/test_aabb.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_aabb.dir/depend

