# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/build

# Utility rule file for test_test_servo.

# Include the progress variables for this target.
include CMakeFiles/test_test_servo.dir/progress.make

CMakeFiles/test_test_servo: ../test_servo
	cd /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp && /opt/ros/fuerte/share/rosunit/bin/rosunit --name=test_servo --time-limit=60.0 /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/test_servo

test_test_servo: CMakeFiles/test_test_servo
test_test_servo: CMakeFiles/test_test_servo.dir/build.make
.PHONY : test_test_servo

# Rule to build all files generated by this target.
CMakeFiles/test_test_servo.dir/build: test_test_servo
.PHONY : CMakeFiles/test_test_servo.dir/build

CMakeFiles/test_test_servo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_test_servo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_test_servo.dir/clean

CMakeFiles/test_test_servo.dir/depend:
	cd /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/build/CMakeFiles/test_test_servo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_test_servo.dir/depend

