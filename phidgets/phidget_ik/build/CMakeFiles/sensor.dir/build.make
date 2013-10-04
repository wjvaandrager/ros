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
CMAKE_SOURCE_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build

# Include any dependencies generated for this target.
include CMakeFiles/sensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensor.dir/flags.make

CMakeFiles/sensor.dir/src/sensor.o: CMakeFiles/sensor.dir/flags.make
CMakeFiles/sensor.dir/src/sensor.o: ../src/sensor.cpp
CMakeFiles/sensor.dir/src/sensor.o: ../manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp_c_api/manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /opt/ros/fuerte/share/rospy/manifest.xml
CMakeFiles/sensor.dir/src/sensor.o: /opt/ros/fuerte/share/roscpp/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/sensor.dir/src/sensor.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/sensor.dir/src/sensor.o -c /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/src/sensor.cpp

CMakeFiles/sensor.dir/src/sensor.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensor.dir/src/sensor.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/src/sensor.cpp > CMakeFiles/sensor.dir/src/sensor.i

CMakeFiles/sensor.dir/src/sensor.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensor.dir/src/sensor.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/src/sensor.cpp -o CMakeFiles/sensor.dir/src/sensor.s

CMakeFiles/sensor.dir/src/sensor.o.requires:
.PHONY : CMakeFiles/sensor.dir/src/sensor.o.requires

CMakeFiles/sensor.dir/src/sensor.o.provides: CMakeFiles/sensor.dir/src/sensor.o.requires
	$(MAKE) -f CMakeFiles/sensor.dir/build.make CMakeFiles/sensor.dir/src/sensor.o.provides.build
.PHONY : CMakeFiles/sensor.dir/src/sensor.o.provides

CMakeFiles/sensor.dir/src/sensor.o.provides.build: CMakeFiles/sensor.dir/src/sensor.o

# Object files for target sensor
sensor_OBJECTS = \
"CMakeFiles/sensor.dir/src/sensor.o"

# External object files for target sensor
sensor_EXTERNAL_OBJECTS =

../bin/sensor: CMakeFiles/sensor.dir/src/sensor.o
../bin/sensor: CMakeFiles/sensor.dir/build.make
../bin/sensor: CMakeFiles/sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/sensor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sensor.dir/build: ../bin/sensor
.PHONY : CMakeFiles/sensor.dir/build

CMakeFiles/sensor.dir/requires: CMakeFiles/sensor.dir/src/sensor.o.requires
.PHONY : CMakeFiles/sensor.dir/requires

CMakeFiles/sensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensor.dir/clean

CMakeFiles/sensor.dir/depend:
	cd /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_ik/build/CMakeFiles/sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensor.dir/depend
