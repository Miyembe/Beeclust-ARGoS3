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
CMAKE_SOURCE_DIR = /home/oer/Seongin/Software/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/oer/Seongin/Software/argos3-examples

# Include any dependencies generated for this target.
include phero_sensor/CMakeFiles/footbot_pheroset.dir/depend.make

# Include the progress variables for this target.
include phero_sensor/CMakeFiles/footbot_pheroset.dir/progress.make

# Include the compile flags for this target's objects.
include phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o: phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make
phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o: phero_sensor/ci_footbot_phero_sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o -c /home/oer/Seongin/Software/argos3-examples/phero_sensor/ci_footbot_phero_sensor.cpp

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.i"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oer/Seongin/Software/argos3-examples/phero_sensor/ci_footbot_phero_sensor.cpp > CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.i

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.s"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oer/Seongin/Software/argos3-examples/phero_sensor/ci_footbot_phero_sensor.cpp -o CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.s

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.requires:

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.provides: phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.requires
	$(MAKE) -f phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.provides.build
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.provides

phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.provides.build: phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o


phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o: phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make
phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o: phero_sensor/footbot_phero.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o -c /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero.cpp

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.i"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero.cpp > CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.i

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.s"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero.cpp -o CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.s

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.requires:

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.provides: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.requires
	$(MAKE) -f phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.provides.build
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.provides

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.provides.build: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o


phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o: phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make
phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o: phero_sensor/footbot_phero_sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o -c /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero_sensor.cpp

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.i"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero_sensor.cpp > CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.i

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.s"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_phero_sensor.cpp -o CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.s

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.requires:

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.provides: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.requires
	$(MAKE) -f phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.provides.build
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.provides

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.provides.build: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o


phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o: phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make
phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o: phero_sensor/phero_sensor_equipped_entity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o -c /home/oer/Seongin/Software/argos3-examples/phero_sensor/phero_sensor_equipped_entity.cpp

phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.i"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oer/Seongin/Software/argos3-examples/phero_sensor/phero_sensor_equipped_entity.cpp > CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.i

phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.s"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oer/Seongin/Software/argos3-examples/phero_sensor/phero_sensor_equipped_entity.cpp -o CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.s

phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.requires:

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.provides: phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.requires
	$(MAKE) -f phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.provides.build
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.provides

phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.provides.build: phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o


phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o: phero_sensor/CMakeFiles/footbot_pheroset.dir/flags.make
phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o: phero_sensor/footbot_pheroset_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o -c /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_pheroset_autogen/mocs_compilation.cpp

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.i"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_pheroset_autogen/mocs_compilation.cpp > CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.i

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.s"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oer/Seongin/Software/argos3-examples/phero_sensor/footbot_pheroset_autogen/mocs_compilation.cpp -o CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.s

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.requires:

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.provides: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.provides

phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.provides.build: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o


# Object files for target footbot_pheroset
footbot_pheroset_OBJECTS = \
"CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o" \
"CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o" \
"CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o" \
"CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o" \
"CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o"

# External object files for target footbot_pheroset
footbot_pheroset_EXTERNAL_OBJECTS =

phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/build.make
phero_sensor/libfootbot_pheroset.so: phero_sensor/CMakeFiles/footbot_pheroset.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/oer/Seongin/Software/argos3-examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libfootbot_pheroset.so"
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/footbot_pheroset.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
phero_sensor/CMakeFiles/footbot_pheroset.dir/build: phero_sensor/libfootbot_pheroset.so

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/build

phero_sensor/CMakeFiles/footbot_pheroset.dir/requires: phero_sensor/CMakeFiles/footbot_pheroset.dir/ci_footbot_phero_sensor.cpp.o.requires
phero_sensor/CMakeFiles/footbot_pheroset.dir/requires: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero.cpp.o.requires
phero_sensor/CMakeFiles/footbot_pheroset.dir/requires: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_phero_sensor.cpp.o.requires
phero_sensor/CMakeFiles/footbot_pheroset.dir/requires: phero_sensor/CMakeFiles/footbot_pheroset.dir/phero_sensor_equipped_entity.cpp.o.requires
phero_sensor/CMakeFiles/footbot_pheroset.dir/requires: phero_sensor/CMakeFiles/footbot_pheroset.dir/footbot_pheroset_autogen/mocs_compilation.cpp.o.requires

.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/requires

phero_sensor/CMakeFiles/footbot_pheroset.dir/clean:
	cd /home/oer/Seongin/Software/argos3-examples/phero_sensor && $(CMAKE_COMMAND) -P CMakeFiles/footbot_pheroset.dir/cmake_clean.cmake
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/clean

phero_sensor/CMakeFiles/footbot_pheroset.dir/depend:
	cd /home/oer/Seongin/Software/argos3-examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oer/Seongin/Software/argos3-examples /home/oer/Seongin/Software/argos3-examples/phero_sensor /home/oer/Seongin/Software/argos3-examples /home/oer/Seongin/Software/argos3-examples/phero_sensor /home/oer/Seongin/Software/argos3-examples/phero_sensor/CMakeFiles/footbot_pheroset.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : phero_sensor/CMakeFiles/footbot_pheroset.dir/depend

