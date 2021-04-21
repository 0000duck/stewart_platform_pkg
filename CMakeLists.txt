cmake_minimum_required(VERSION 3.5)
project(stewart_platform_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

IF (NOT DEFINED BUILD_VERSION)
	SET(BUILD_VERSION "not set")
ENDIF()
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(stewart_platform_interfaces REQUIRED)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Include Cpp "include" directory and Qt directories
include_directories(
  include/stewart_platform_pkg
  ${rclcpp_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(actuator_node 
  src/actuator_node.cpp
  src/canbus.cpp
)
ament_target_dependencies(
  actuator_node
  rclcpp
  stewart_platform_interfaces
)

add_executable(hmi_node 
  "src/main.cpp"
  "src/hmi_node1.cpp"
  "src/qml.qrc"
  "include/stewart_platform_pkg/hmi_node1.h"
  "include/stewart_platform_pkg/qt_executor.cpp"

)
ament_target_dependencies(
  hmi_node
  rclcpp
  rmw_implementation
  stewart_platform_interfaces
)


target_link_libraries(hmi_node
  Qt5::Core
  Qt5::Quick
)

# Install Cpp executables
install(TARGETS
  actuator_node
  hmi_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/kinematics_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
