# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/250/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/250/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocomp/robocomp/components/grupo-4-robotica/rooms

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug

# Utility rule file for ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/progress.make

ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src: src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/CommonBehavior.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/CommonBehavior.ice"
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src && robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/CommonBehavior.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/GenericBase.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/GenericBase.ice"
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src && robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/GenericBase.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/Lidar3D.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/Lidar3D.ice"
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src && robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/Lidar3D.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/Lidar3D.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "BU robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/OmniRobot.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/OmniRobot.ice"
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src && robocompdsl /home/robocomp/robocomp//interfaces/IDSLs/OmniRobot.idsl /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src/OmniRobot.ice
.PHONY : ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/build: ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src
.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/build

src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/clean:
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/clean

src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/depend:
	cd /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocomp/robocomp/components/grupo-4-robotica/rooms /home/robocomp/robocomp/components/grupo-4-robotica/rooms/src /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src /home/robocomp/robocomp/components/grupo-4-robotica/rooms/cmake-build-debug/src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ICES__home_robocomp_robocomp_components_grupo-4-robotica_rooms_src.dir/depend

