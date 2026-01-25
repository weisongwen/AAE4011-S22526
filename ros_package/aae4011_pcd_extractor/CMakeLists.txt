################################################################################
# CMakeLists.txt - Build Configuration File for ROS Package
################################################################################
#
# This file tells the catkin build system how to compile your ROS package.
# CMake is a cross-platform build system, and catkin is ROS's build system
# built on top of CMake.
#
# This file defines:
# - Minimum CMake version required
# - Package name
# - Dependencies (find_package)
# - Include directories (where to find header files)
# - Executables to build (add_executable)
# - Libraries to link (target_link_libraries)
#
# Official documentation: http://wiki.ros.org/catkin/CMakeLists.txt
#
################################################################################

################################################################################
# 1. CMAKE VERSION REQUIREMENT
################################################################################
# Specify the minimum version of CMake required to build this package.
# ROS Kinetic and later require at least CMake 3.0.2
# This ensures we have access to modern CMake features.
cmake_minimum_required(VERSION 3.0.2)

################################################################################
# 2. PROJECT NAME
################################################################################
# Define the name of this project. This should match the package name in
# package.xml. The project name is used internally by CMake.
project(aae4011_pcd_extractor)

################################################################################
# 3. C++ STANDARD (Optional but recommended)
################################################################################
# Set C++ standard to C++14 (or C++11 for older ROS versions)
# This enables modern C++ features like auto, lambda functions, smart pointers
# PCL requires at least C++11, C++14 is recommended for better compatibility
add_compile_options(-std=c++14)

################################################################################
# 4. FIND REQUIRED PACKAGES
################################################################################
# find_package() locates and configures external dependencies.
# For ROS packages, we use catkin's special find_package with COMPONENTS.
#
# COMPONENTS lists all ROS packages that this package depends on.
# This must match the dependencies listed in package.xml!
#
# What happens during find_package():
# 1. CMake searches for <package>Config.cmake files
# 2. Loads package configuration (include paths, libraries, etc.)
# 3. Sets variables like ${catkin_INCLUDE_DIRS}, ${catkin_LIBRARIES}
# 4. If REQUIRED is specified and package not found, build fails
#
# Note: We use 'catkin' as the main package name and list ROS packages
# as COMPONENTS. This is the standard way for catkin packages.

find_package(catkin REQUIRED COMPONENTS
  roscpp              # C++ ROS client library (core functionality)
  sensor_msgs         # Standard sensor message definitions (PointCloud2)
  pcl_ros             # PCL-ROS integration (conversions, utilities)
  pcl_conversions     # Low-level ROS<->PCL conversions
)

# Find PCL (Point Cloud Library) separately
# PCL is not a ROS package, so we find it outside of catkin COMPONENTS
# This locates the system-installed PCL library (usually via apt-get)
# REQUIRED means the build will fail if PCL is not found
find_package(PCL REQUIRED)

# Debug output (optional - uncomment to see what find_package found)
# message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
# message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
# message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

################################################################################
# 5. CATKIN PACKAGE CONFIGURATION
################################################################################
# catkin_package() is a catkin-specific macro that configures this package
# for use by other packages. It generates CMake configuration files.
#
# This section defines what this package exports to other packages that
# depend on it. For a simple executable node (not a library), we usually
# just need to specify CATKIN_DEPENDS.
#
# Parameters explained:
# - INCLUDE_DIRS: Header file directories to export (for libraries)
# - LIBRARIES: Libraries to export (for library packages)
# - CATKIN_DEPENDS: ROS packages that downstream packages also need
# - DEPENDS: Non-ROS system dependencies (e.g., PCL, Eigen)
#
# For this package: We only build an executable (not a library), so we
# mainly need to declare our catkin dependencies.

catkin_package(
  # INCLUDE_DIRS include  # Uncomment if you have a public header directory
  # LIBRARIES aae4011_pcd_extractor  # Uncomment if you build a library
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    pcl_ros 
    pcl_conversions
  DEPENDS 
    PCL  # System dependency on Point Cloud Library
)

################################################################################
# 6. SPECIFY INCLUDE DIRECTORIES
################################################################################
# Tell the compiler where to find header files (.h, .hpp).
# This is like adding -I flags to g++.
#
# Order matters: catkin includes are typically first, then system includes.
#
# ${catkin_INCLUDE_DIRS}: Header files from ROS packages (roscpp, sensor_msgs, etc.)
# ${PCL_INCLUDE_DIRS}: Header files from PCL library

include_directories(
  # include  # Uncomment if you have a local include/ directory for headers
  ${catkin_INCLUDE_DIRS}  # ROS package headers
  ${PCL_INCLUDE_DIRS}     # PCL headers
)

# Link directories for PCL (where to find PCL library files)
# This is like adding -L flags to g++
link_directories(${PCL_LIBRARY_DIRS})

# Add PCL definitions (compile flags required by PCL)
# This might include things like -DPCL_NO_PRECOMPILE
add_definitions(${PCL_DEFINITIONS})

################################################################################
# 7. DECLARE EXECUTABLES
################################################################################
# add_executable() tells CMake to build an executable program.
#
# Syntax: add_executable(executable_name source_file1 source_file2 ...)
#
# This creates a target that:
# 1. Compiles all specified source files
# 2. Links them together
# 3. Produces an executable binary
#
# The executable will be created in: devel/lib/<package_name>/
# and can be run with: rosrun <package_name> <executable_name>

add_executable(pcd_extractor_node 
  src/pcd_extractor_node.cpp  # Main source file
)

# If you have multiple source files, list them all:
# add_executable(pcd_extractor_node
#   src/pcd_extractor_node.cpp
#   src/helper_functions.cpp
#   src/file_utils.cpp
# )

################################################################################
# 8. SPECIFY LIBRARIES TO LINK
################################################################################
# target_link_libraries() links external libraries to your executable.
# This is like adding -l flags to g++.
#
# Your executable needs to link against:
# 1. ROS libraries (from catkin packages)
# 2. PCL libraries
# 3. System libraries (pthread, etc.)
#
# ${catkin_LIBRARIES}: All libraries from packages in find_package(catkin ...)
#                       Includes libroscpp.so, libsensor_msgs.so, etc.
# ${PCL_LIBRARIES}: All PCL libraries (pcl_common, pcl_io, pcl_filters, etc.)
#
# Order sometimes matters: list the target first, then its dependencies.

target_link_libraries(pcd_extractor_node
  ${catkin_LIBRARIES}  # ROS libraries
  ${PCL_LIBRARIES}     # PCL libraries
)

################################################################################
# 9. ADD DEPENDENCIES (Optional but recommended)
################################################################################
# add_dependencies() ensures proper build order when you have custom messages,
# services, or actions.
#
# This tells CMake that our executable depends on message generation targets
# from catkin packages. It prevents compilation errors when messages are
# being generated in parallel.
#
# ${catkin_EXPORTED_TARGETS}: All message generation targets from dependencies
#
# For this simple package, this line ensures sensor_msgs are built before
# our node (in case we're building from scratch in a fresh workspace).

add_dependencies(pcd_extractor_node 
  ${catkin_EXPORTED_TARGETS}
)

# If you have custom messages in this package, you'd also add:
# add_dependencies(pcd_extractor_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

################################################################################
# 10. INSTALL TARGETS (Optional - for releasing the package)
################################################################################
# install() rules specify what gets copied when you run "catkin_make install"
# This is important for releasing your package or deploying to robots.
#
# Typically you install:
# - Executables (to lib/<package_name>/)
# - Libraries (to lib/)
# - Headers (to include/<package_name>/)
# - Launch files, config files (to share/<package_name>/)
#
# Uncomment these if you want to create an installable package:

# Install executable
# install(TARGETS pcd_extractor_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# Install other files (config, data, etc.)
# install(DIRECTORY config/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )

################################################################################
# 11. TESTING (Optional)
################################################################################
# If you want to add unit tests using rostest/gtest:
#
# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(test_pcd_extractor test/test_pcd_extractor.test test/test_pcd_extractor.cpp)
#   target_link_libraries(test_pcd_extractor ${catkin_LIBRARIES})
# endif()

################################################################################
# SUMMARY FOR STUDENTS
################################################################################
#
# Build Process Overview:
# 1. cmake reads this CMakeLists.txt
# 2. Finds all required packages and their libraries
# 3. Configures include paths and library paths
# 4. Generates Makefiles
# 5. make compiles source files into object files (.o)
# 6. make links object files with libraries into executable
# 7. Executable placed in devel/lib/<package_name>/
#
# Key CMake Variables (automatically set by find_package):
# - ${catkin_INCLUDE_DIRS}: Paths to ROS package headers
# - ${catkin_LIBRARIES}: ROS libraries to link against
# - ${PCL_INCLUDE_DIRS}: Paths to PCL headers
# - ${PCL_LIBRARIES}: PCL libraries to link against
# - ${CATKIN_PACKAGE_BIN_DESTINATION}: Where to install executables
#
# Essential Commands in Order:
# 1. cmake_minimum_required()  - Set required CMake version
# 2. project()                 - Name your project
# 3. find_package()            - Find dependencies
# 4. catkin_package()          - Configure catkin exports
# 5. include_directories()     - Specify header locations
# 6. add_executable()          - Define what to build
# 7. target_link_libraries()   - Link libraries to executable
# 8. add_dependencies()        - Set build order
#
# Common Mistakes:
# ✗ Forgetting to add a package to find_package() when using it in code
# ✗ Not listing source files in add_executable()
# ✗ Wrong order of include_directories() call
# ✗ Forgetting ${} around variable names
# ✗ Package name mismatch with package.xml
#
# Debugging Tips:
# - If build fails with "No such file or directory" for a header:
#   → Check find_package() and include_directories()
# - If build fails with "undefined reference" during linking:
#   → Check target_link_libraries()
# - Use message(STATUS "Variable: ${VARIABLE}") to debug
# - Check build output in terminal for specific errors
#
# After modifying CMakeLists.txt:
# $ cd ~/catkin_ws
# $ catkin_make
# $ source devel/setup.bash
#
################################################################################
