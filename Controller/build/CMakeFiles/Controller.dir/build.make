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
CMAKE_COMMAND = /home/zubuntu/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/zubuntu/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build

# Include any dependencies generated for this target.
include CMakeFiles/Controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Controller.dir/flags.make

CMakeFiles/Controller.dir/src/controller.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/controller.cpp.o: ../src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Controller.dir/src/controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/controller.cpp.o -c /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/controller.cpp

CMakeFiles/Controller.dir/src/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/controller.cpp > CMakeFiles/Controller.dir/src/controller.cpp.i

CMakeFiles/Controller.dir/src/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/controller.cpp -o CMakeFiles/Controller.dir/src/controller.cpp.s

CMakeFiles/Controller.dir/src/RwsInterface.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/src/RwsInterface.cpp.o: ../src/RwsInterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Controller.dir/src/RwsInterface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/src/RwsInterface.cpp.o -c /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/RwsInterface.cpp

CMakeFiles/Controller.dir/src/RwsInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/src/RwsInterface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/RwsInterface.cpp > CMakeFiles/Controller.dir/src/RwsInterface.cpp.i

CMakeFiles/Controller.dir/src/RwsInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/src/RwsInterface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/src/RwsInterface.cpp -o CMakeFiles/Controller.dir/src/RwsInterface.cpp.s

CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o: CMakeFiles/Controller.dir/flags.make
CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o: Controller_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o -c /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/Controller_autogen/mocs_compilation.cpp

CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/Controller_autogen/mocs_compilation.cpp > CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.i

CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/Controller_autogen/mocs_compilation.cpp -o CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.s

# Object files for target Controller
Controller_OBJECTS = \
"CMakeFiles/Controller.dir/src/controller.cpp.o" \
"CMakeFiles/Controller.dir/src/RwsInterface.cpp.o" \
"CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o"

# External object files for target Controller
Controller_EXTERNAL_OBJECTS =

Controller: CMakeFiles/Controller.dir/src/controller.cpp.o
Controller: CMakeFiles/Controller.dir/src/RwsInterface.cpp.o
Controller: CMakeFiles/Controller.dir/Controller_autogen/mocs_compilation.cpp.o
Controller: CMakeFiles/Controller.dir/build.make
Controller: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Controller: /usr/lib/x86_64-linux-gnu/libGLX.so
Controller: /usr/lib/x86_64-linux-gnu/libGLU.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libyaobi.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libpqp.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_csgjs.a
Controller: /usr/lib/x86_64-linux-gnu/libfcl.so
Controller: /usr/lib/x86_64-linux-gnu/liblua5.3.so
Controller: /usr/lib/x86_64-linux-gnu/libm.so
Controller: /usr/lib/x86_64-linux-gnu/libassimp.so
Controller: /usr/lib/x86_64-linux-gnu/libdl.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_system.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Controller: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
Controller: /home/zubuntu/Projects/RobWork/Build/RobWorkStudio/libs/release/libsdurws_robworkstudioapp.so
Controller: /home/zubuntu/Projects/RobWork/Build/RobWorkStudio/libs/release/libsdurws_workcelleditor.so
Controller: /usr/lib/x86_64-linux-gnu/libsdurws_luaeditor.so
Controller: /home/zubuntu/Projects/RobWork/Build/RobWorkStudio/libs/release/libsdurws.so
Controller: /home/zubuntu/Projects/RobWork/Build/RobWorkStudio/libs/release/libqtpropertybrowser.a
Controller: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Controller: /usr/lib/x86_64-linux-gnu/libGLX.so
Controller: /usr/lib/x86_64-linux-gnu/libGLU.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libyaobi.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libpqp.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_csgjs.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_pathplanners.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_pathoptimization.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_simulation.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_opengl.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_assembly.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_task.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_calibration.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_csg.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_control.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_proximitystrategies.so
Controller: /usr/lib/x86_64-linux-gnu/libfcl.so
Controller: /usr/lib/x86_64-linux-gnu/liblua5.3.so
Controller: /usr/lib/x86_64-linux-gnu/libm.so
Controller: /usr/lib/x86_64-linux-gnu/libassimp.so
Controller: /usr/lib/x86_64-linux-gnu/libdl.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_system.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Controller: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
Controller: /usr/lib/x86_64-linux-gnu/libglut.so
Controller: /usr/lib/x86_64-linux-gnu/libpython3.6m.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_algorithms.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_csgjs.a
Controller: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
Controller: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libpqp.a
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_plugin.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_graspplanning.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_loaders.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_graphics.so
Controller: /usr/lib/x86_64-linux-gnu/libxerces-c.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_pathplanning.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_invkin.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_trajectory.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_proximity.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_models.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_sensor.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_geometry.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_kinematics.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_math.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_common.so
Controller: /home/zubuntu/Projects/RobWork/Build/RW/libs/release/libsdurw_core.so
Controller: /usr/lib/gcc/x86_64-linux-gnu/7/libgomp.so
Controller: /usr/lib/x86_64-linux-gnu/libpthread.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_system.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_thread.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
Controller: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
Controller: /usr/lib/x86_64-linux-gnu/libpython2.7.so
Controller: /usr/lib/x86_64-linux-gnu/libOpenGL.so
Controller: /usr/lib/x86_64-linux-gnu/libGLX.so
Controller: /usr/lib/x86_64-linux-gnu/libGLU.so
Controller: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
Controller: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
Controller: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
Controller: CMakeFiles/Controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Controller.dir/build: Controller

.PHONY : CMakeFiles/Controller.dir/build

CMakeFiles/Controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Controller.dir/clean

CMakeFiles/Controller.dir/depend:
	cd /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build /home/zubuntu/Projects/sdu_courses/rovis/project/workspace/Controller/build/CMakeFiles/Controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Controller.dir/depend
