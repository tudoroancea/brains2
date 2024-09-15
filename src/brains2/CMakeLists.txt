cmake_minimum_required(VERSION 3.8)
project(brains2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall -Wextra -Wpedantic -pthread -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -pthread")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

####################################################################################################
# dependencies
####################################################################################################

# find ROS 2 dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

# find other libraries: eigen3, nlohmann json
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

####################################################################################################
# custom message and service types
####################################################################################################
set(msg_files
  "msg/Acceleration.msg"
  "msg/Controls.msg"
  "msg/Map.msg"
  "msg/Pose.msg"
  "msg/TrackEstimate.msg"
  "msg/Velocity.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

####################################################################################################
# common
####################################################################################################

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_add_library(brains2_common SHARED src/common/cone_color.cpp src/common/marker_color.cpp src/common/math.cpp src/common/tracks.cpp)
target_link_libraries(brains2_common Eigen3::Eigen nlohmann_json::nlohmann_json)

####################################################################################################
# sim
####################################################################################################
ament_auto_add_library(brains2_sim SHARED src/sim/sim.cpp src/sim/models.c)
target_link_libraries(brains2_sim Eigen3::Eigen)
target_include_directories(brains2_sim PUBLIC src/sim)
####################################################################################################
# control
####################################################################################################

####################################################################################################
# estimation
####################################################################################################

####################################################################################################
# create package
####################################################################################################

ament_auto_package(INSTALL_TO_SHARE)
