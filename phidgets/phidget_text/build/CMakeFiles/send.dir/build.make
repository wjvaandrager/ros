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
CMAKE_SOURCE_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build

# Include any dependencies generated for this target.
include CMakeFiles/send.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/send.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/send.dir/flags.make

CMakeFiles/send.dir/send_string.o: CMakeFiles/send.dir/flags.make
CMakeFiles/send.dir/send_string.o: ../send_string.cc
CMakeFiles/send.dir/send_string.o: ../manifest.xml
CMakeFiles/send.dir/send_string.o: /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp_c_api/manifest.xml
CMakeFiles/send.dir/send_string.o: /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidgetspp/manifest.xml
CMakeFiles/send.dir/send_string.o: /opt/ros/fuerte/share/roslang/manifest.xml
CMakeFiles/send.dir/send_string.o: /opt/ros/fuerte/share/roscpp/manifest.xml
CMakeFiles/send.dir/send_string.o: /opt/ros/fuerte/share/std_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/send.dir/send_string.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/send.dir/send_string.o -c /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/send_string.cc

CMakeFiles/send.dir/send_string.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/send.dir/send_string.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/send_string.cc > CMakeFiles/send.dir/send_string.i

CMakeFiles/send.dir/send_string.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/send.dir/send_string.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/send_string.cc -o CMakeFiles/send.dir/send_string.s

CMakeFiles/send.dir/send_string.o.requires:
.PHONY : CMakeFiles/send.dir/send_string.o.requires

CMakeFiles/send.dir/send_string.o.provides: CMakeFiles/send.dir/send_string.o.requires
	$(MAKE) -f CMakeFiles/send.dir/build.make CMakeFiles/send.dir/send_string.o.provides.build
.PHONY : CMakeFiles/send.dir/send_string.o.provides

CMakeFiles/send.dir/send_string.o.provides.build: CMakeFiles/send.dir/send_string.o

# Object files for target send
send_OBJECTS = \
"CMakeFiles/send.dir/send_string.o"

# External object files for target send
send_EXTERNAL_OBJECTS =

../send: CMakeFiles/send.dir/send_string.o
../send: CMakeFiles/send.dir/build.make
../send: CMakeFiles/send.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../send"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/send.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/send.dir/build: ../send
.PHONY : CMakeFiles/send.dir/build

CMakeFiles/send.dir/requires: CMakeFiles/send.dir/send_string.o.requires
.PHONY : CMakeFiles/send.dir/requires

CMakeFiles/send.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/send.dir/cmake_clean.cmake
.PHONY : CMakeFiles/send.dir/clean

CMakeFiles/send.dir/depend:
	cd /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build /home/pc_willem/ros/dbl-ros-pkg-dev/drivers/phidgets/phidget_text/build/CMakeFiles/send.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/send.dir/depend

