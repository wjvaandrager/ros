cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
#ADD_DEFINITIONS(-DDEBUG)

# ROS stuff
rosbuild_init()

execute_process(COMMAND rosversion ros OUTPUT_VARIABLE ROS_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
IF (NOT ${ROS_VERSION} MATCHES [0-9]*\\.[0-9]*\\.[0-9])
  execute_process(COMMAND rosversion roscpp OUTPUT_VARIABLE ROS_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
ENDIF()  

# DBL stuff
execute_process(COMMAND rospack find dbl_repos OUTPUT_VARIABLE WORKSPACE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)

IF (${ROS_VERSION} VERSION_LESS 1.6.2)
  execute_process(COMMAND rospack cflags-only-I tinyxml OUTPUT_VARIABLE TINYXML_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
  SET(TINYXML_INCLUDE_DIR ${TINYXML_INCLUDE_DIR}/tinyxml)
  execute_process(COMMAND rospack cflags-only-other tinyxml OUTPUT_VARIABLE TINYXML_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
ELSE()
  SET(TINYXML_INCLUDE_DIR /usr/include)
  SET(TINYXML_CFLAGS -DTIXML_NEW_TYPES)
ENDIF()

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -fPIC)

# Check for libftdipp
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(LIBFTDIPP QUIET libftdipp)
IF (LIBFTDIPP_FOUND)
  SET(FTDI_SOURCE src/LxFTDI.cpp)
  SET(FTDI_LIBS ${LIBFTDIPP_LDFLAGS})

  message("-- using FTDI libs: ${FTDI_LIBS}")
ENDIF()


SET(TARGET threemxl)
rosbuild_add_library(${TARGET} src/CDxlROSPacketHandler.cpp ${FTDI_SOURCE})
TARGET_LINK_LIBRARIES(${TARGET} -Wl,-whole-archive)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/hardware/dynamixel/dynamixel.cmake)
INCLUDE (${WORKSPACE_DIR}/dbl/platform/io/logging/stdlogging.cmake)
TARGET_LINK_LIBRARIES(${TARGET} -Wl,-no-whole-archive)

IF (${ROS_VERSION} VERSION_LESS 1.6.2)
ELSE()
  TARGET_LINK_LIBRARIES(${TARGET} -ltinyxml ${FTDI_LIBS})
ENDIF()

rosbuild_add_executable(gripper src/gripper.cpp)
ADD_DEPENDENCIES(gripper threemxl)
TARGET_LINK_LIBRARIES(gripper -L${PROJECT_SOURCE_DIR}/lib -Wl,-rpath,${PROJECT_SOURCE_DIR}/lib -lthreemxl)

IF (${ROS_VERSION} VERSION_LESS 1.6.2)
ELSE()
  TARGET_LINK_LIBRARIES(gripper -ltinyxml)
ENDIF()

rosbuild_add_executable(console src/console.cpp)
ADD_DEPENDENCIES(console threemxl)
TARGET_LINK_LIBRARIES(console -L${PROJECT_SOURCE_DIR}/lib -Wl,-rpath,${PROJECT_SOURCE_DIR}/lib -lthreemxl -lreadline)

IF (${ROS_VERSION} VERSION_LESS 1.6.2)
ELSE()
  TARGET_LINK_LIBRARIES(console -ltinyxml)
ENDIF()

#rosbuild_add_executable(benchmark src/benchmark.cpp)
#ADD_DEPENDENCIES(benchmark threemxl)
#TARGET_LINK_LIBRARIES(benchmark -L${PROJECT_SOURCE_DIR}/lib -Wl,-rpath,${PROJECT_SOURCE_DIR}/lib -#lthreemxl)

#IF (${ROS_VERSION} VERSION_LESS 1.6.2)
#ELSE()
#  TARGET_LINK_LIBRARIES(benchmark -ltinyxml)
#ENDIF()

#rosbuild_add_executable(syncreadtest src/syncreadtest.cpp)
#ADD_DEPENDENCIES(syncreadtest threemxl)
#TARGET_LINK_LIBRARIES(syncreadtest -L${PROJECT_SOURCE_DIR}/lib -Wl,-rpath,${PROJECT_SOURCE_DIR}/lib -lthreemxl)

#IF (${ROS_VERSION} VERSION_LESS 1.6.2)
#ELSE()
#  TARGET_LINK_LIBRARIES(benchmark -ltinyxml)
#ENDIF()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
