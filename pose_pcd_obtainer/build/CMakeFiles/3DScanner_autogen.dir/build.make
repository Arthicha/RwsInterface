# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/student/RoViProject/pose_pcd_obtainer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/RoViProject/pose_pcd_obtainer/build

# Utility rule file for 3DScanner_autogen.

# Include the progress variables for this target.
include CMakeFiles/3DScanner_autogen.dir/progress.make

CMakeFiles/3DScanner_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/RoViProject/pose_pcd_obtainer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target 3DScanner"
	/usr/bin/cmake -E cmake_autogen /home/student/RoViProject/pose_pcd_obtainer/build/CMakeFiles/3DScanner_autogen.dir/AutogenInfo.json Release

3DScanner_autogen: CMakeFiles/3DScanner_autogen
3DScanner_autogen: CMakeFiles/3DScanner_autogen.dir/build.make

.PHONY : 3DScanner_autogen

# Rule to build all files generated by this target.
CMakeFiles/3DScanner_autogen.dir/build: 3DScanner_autogen

.PHONY : CMakeFiles/3DScanner_autogen.dir/build

CMakeFiles/3DScanner_autogen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3DScanner_autogen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3DScanner_autogen.dir/clean

CMakeFiles/3DScanner_autogen.dir/depend:
	cd /home/student/RoViProject/pose_pcd_obtainer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/RoViProject/pose_pcd_obtainer /home/student/RoViProject/pose_pcd_obtainer /home/student/RoViProject/pose_pcd_obtainer/build /home/student/RoViProject/pose_pcd_obtainer/build /home/student/RoViProject/pose_pcd_obtainer/build/CMakeFiles/3DScanner_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3DScanner_autogen.dir/depend
