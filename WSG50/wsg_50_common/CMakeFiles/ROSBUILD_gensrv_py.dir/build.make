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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common

# Utility rule file for ROSBUILD_gensrv_py.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_gensrv_py.dir/progress.make

CMakeFiles/ROSBUILD_gensrv_py: src/wsg_50_common/srv/__init__.py

src/wsg_50_common/srv/__init__.py: src/wsg_50_common/srv/_Conf.py
src/wsg_50_common/srv/__init__.py: src/wsg_50_common/srv/_Incr.py
src/wsg_50_common/srv/__init__.py: src/wsg_50_common/srv/_Move.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/wsg_50_common/srv/__init__.py"
	/opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py --initpy /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Conf.srv /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Incr.srv /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Move.srv

src/wsg_50_common/srv/_Conf.py: srv/Conf.srv
src/wsg_50_common/srv/_Conf.py: /opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py
src/wsg_50_common/srv/_Conf.py: /opt/ros/indigo/share/roslib/cmake/../../../lib/roslib/gendeps
src/wsg_50_common/srv/_Conf.py: manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/wsg_50_common/srv/_Conf.py"
	/opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Conf.srv

src/wsg_50_common/srv/_Incr.py: srv/Incr.srv
src/wsg_50_common/srv/_Incr.py: /opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py
src/wsg_50_common/srv/_Incr.py: /opt/ros/indigo/share/roslib/cmake/../../../lib/roslib/gendeps
src/wsg_50_common/srv/_Incr.py: manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/wsg_50_common/srv/_Incr.py"
	/opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Incr.srv

src/wsg_50_common/srv/_Move.py: srv/Move.srv
src/wsg_50_common/srv/_Move.py: /opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py
src/wsg_50_common/srv/_Move.py: /opt/ros/indigo/share/roslib/cmake/../../../lib/roslib/gendeps
src/wsg_50_common/srv/_Move.py: manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating src/wsg_50_common/srv/_Move.py"
	/opt/ros/indigo/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/srv/Move.srv

ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py
ROSBUILD_gensrv_py: src/wsg_50_common/srv/__init__.py
ROSBUILD_gensrv_py: src/wsg_50_common/srv/_Conf.py
ROSBUILD_gensrv_py: src/wsg_50_common/srv/_Incr.py
ROSBUILD_gensrv_py: src/wsg_50_common/srv/_Move.py
ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py.dir/build.make
.PHONY : ROSBUILD_gensrv_py

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_py.dir/build: ROSBUILD_gensrv_py
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/build

CMakeFiles/ROSBUILD_gensrv_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/clean

CMakeFiles/ROSBUILD_gensrv_py.dir/depend:
	cd /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common /home/clonecomputer/catkin_ws/src/wsg50/wsg_50_common/CMakeFiles/ROSBUILD_gensrv_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/depend

