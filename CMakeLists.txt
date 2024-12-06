cmake_minimum_required(VERSION 3.8)
project(fast_lio_color)

# Set the default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Define project directory for includes
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Conditional compilation flags
if(ISAAC_SIM)
  add_compile_definitions(ISAAC_SIM)
endif()
if(USE_LIVOX)
  add_compile_definitions(USE_LIVOX)
endif()

# Detect CPU architecture and set OpenMP-related flags
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("Core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("Core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
if(USE_LIVOX)
  find_package(livox_ros_driver2 REQUIRED)
endif()
find_package(rosidl_default_generators REQUIRED)

# Third-party libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

set(COLOR_MAPPING_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/fast_lio_color_mapping"
 ) 
# Include directories
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
  ${COLOR_MAPPING_DIRS}
)

# ROS 2 Message definitions
set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# Define executable
add_executable(fastlio_color
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
  src/LensDistortionCorrection.cpp
)

# OpenMP support
if(OpenMP_FOUND)
  target_compile_options(fastlio_color PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(fastlio_color OpenMP::OpenMP_CXX)
endif()

# Link libraries and include directories
target_include_directories(fastlio_color PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS}
)

# Link ROS 2 dependencies and other libraries (plain signature)
target_link_libraries(fastlio_color
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

ament_target_dependencies(fastlio_color
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)

# Handle ROS 2 compatibility for older distributions
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_color
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_color ${cpp_typesupport_target})
endif()

# Installation rules
install(TARGETS fastlio_color
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()