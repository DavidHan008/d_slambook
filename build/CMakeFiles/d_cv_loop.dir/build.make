# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/davidhan/holo_builder/output/bin/cmake

# The command to remove a file.
RM = /home/davidhan/holo_builder/output/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/davidhan/davidhan_project/d_slambook

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/davidhan/davidhan_project/d_slambook/build

# Include any dependencies generated for this target.
include CMakeFiles/d_cv_loop.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/d_cv_loop.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/d_cv_loop.dir/flags.make

CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o: CMakeFiles/d_cv_loop.dir/flags.make
CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o: ../cv_loop.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/davidhan/davidhan_project/d_slambook/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o -c /home/davidhan/davidhan_project/d_slambook/cv_loop.cpp

CMakeFiles/d_cv_loop.dir/cv_loop.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/d_cv_loop.dir/cv_loop.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/davidhan/davidhan_project/d_slambook/cv_loop.cpp > CMakeFiles/d_cv_loop.dir/cv_loop.cpp.i

CMakeFiles/d_cv_loop.dir/cv_loop.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/d_cv_loop.dir/cv_loop.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/davidhan/davidhan_project/d_slambook/cv_loop.cpp -o CMakeFiles/d_cv_loop.dir/cv_loop.cpp.s

# Object files for target d_cv_loop
d_cv_loop_OBJECTS = \
"CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o"

# External object files for target d_cv_loop
d_cv_loop_EXTERNAL_OBJECTS =

d_cv_loop: CMakeFiles/d_cv_loop.dir/cv_loop.cpp.o
d_cv_loop: CMakeFiles/d_cv_loop.dir/build.make
d_cv_loop: /usr/local/lib/libopencv_dnn.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_ml.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_objdetect.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_shape.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_stitching.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_superres.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_videostab.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_viz.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_calib3d.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_features2d.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_flann.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_highgui.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_photo.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_video.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_videoio.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_imgcodecs.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_imgproc.so.3.4.6
d_cv_loop: /usr/local/lib/libopencv_core.so.3.4.6
d_cv_loop: CMakeFiles/d_cv_loop.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/davidhan/davidhan_project/d_slambook/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable d_cv_loop"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/d_cv_loop.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/d_cv_loop.dir/build: d_cv_loop

.PHONY : CMakeFiles/d_cv_loop.dir/build

CMakeFiles/d_cv_loop.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/d_cv_loop.dir/cmake_clean.cmake
.PHONY : CMakeFiles/d_cv_loop.dir/clean

CMakeFiles/d_cv_loop.dir/depend:
	cd /home/davidhan/davidhan_project/d_slambook/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/davidhan/davidhan_project/d_slambook /home/davidhan/davidhan_project/d_slambook /home/davidhan/davidhan_project/d_slambook/build /home/davidhan/davidhan_project/d_slambook/build /home/davidhan/davidhan_project/d_slambook/build/CMakeFiles/d_cv_loop.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/d_cv_loop.dir/depend

