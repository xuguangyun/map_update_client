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
CMAKE_SOURCE_DIR = /root/workspace/carto_client/build/abseil/src/abseil

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/workspace/carto_client/build/abseil/src/abseil-build

# Include any dependencies generated for this target.
include absl/container/CMakeFiles/absl_container.dir/depend.make

# Include the progress variables for this target.
include absl/container/CMakeFiles/absl_container.dir/progress.make

# Include the compile flags for this target's objects.
include absl/container/CMakeFiles/absl_container.dir/flags.make

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o: absl/container/CMakeFiles/absl_container.dir/flags.make
absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o: absl/container/absl_container_header_only_dummy.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/workspace/carto_client/build/abseil/src/abseil-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o"
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o -c /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container/absl_container_header_only_dummy.cc

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.i"
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container/absl_container_header_only_dummy.cc > CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.i

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.s"
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container/absl_container_header_only_dummy.cc -o CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.s

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.requires:

.PHONY : absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.requires

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.provides: absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.requires
	$(MAKE) -f absl/container/CMakeFiles/absl_container.dir/build.make absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.provides.build
.PHONY : absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.provides

absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.provides.build: absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o


# Object files for target absl_container
absl_container_OBJECTS = \
"CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o"

# External object files for target absl_container
absl_container_EXTERNAL_OBJECTS =

absl/container/libabsl_container.a: absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o
absl/container/libabsl_container.a: absl/container/CMakeFiles/absl_container.dir/build.make
absl/container/libabsl_container.a: absl/container/CMakeFiles/absl_container.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/workspace/carto_client/build/abseil/src/abseil-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libabsl_container.a"
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && $(CMAKE_COMMAND) -P CMakeFiles/absl_container.dir/cmake_clean_target.cmake
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/absl_container.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
absl/container/CMakeFiles/absl_container.dir/build: absl/container/libabsl_container.a

.PHONY : absl/container/CMakeFiles/absl_container.dir/build

absl/container/CMakeFiles/absl_container.dir/requires: absl/container/CMakeFiles/absl_container.dir/absl_container_header_only_dummy.cc.o.requires

.PHONY : absl/container/CMakeFiles/absl_container.dir/requires

absl/container/CMakeFiles/absl_container.dir/clean:
	cd /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container && $(CMAKE_COMMAND) -P CMakeFiles/absl_container.dir/cmake_clean.cmake
.PHONY : absl/container/CMakeFiles/absl_container.dir/clean

absl/container/CMakeFiles/absl_container.dir/depend:
	cd /root/workspace/carto_client/build/abseil/src/abseil-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/workspace/carto_client/build/abseil/src/abseil /root/workspace/carto_client/build/abseil/src/abseil/absl/container /root/workspace/carto_client/build/abseil/src/abseil-build /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container /root/workspace/carto_client/build/abseil/src/abseil-build/absl/container/CMakeFiles/absl_container.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : absl/container/CMakeFiles/absl_container.dir/depend

