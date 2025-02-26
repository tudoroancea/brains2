cmake_minimum_required(VERSION 3.8)
project(brains2)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG -Wall -Wextra -Wpedantic -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g -DDEBUG -Wall -Wextra -Wpedantic -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL} -Os -DNDEBUG")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)
add_compile_options(--system-header-prefix=brains2/external/)

add_compile_definitions(TRACK_DATABASE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../track_database")
add_compile_definitions(CAR_CONSTANTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../car_constants.yaml")

set(CMAKE_TESTING_ENABLED ON)

####################################################################################################
# dependencies
####################################################################################################

# find ROS 2 dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

# find other libraries: eigen3, nlohmann json
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
# the following assumes acados has already been installed (automatically done in build.sh)
find_package(acados REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/../acados/cmake)
find_package(casadi REQUIRED)
find_package(fatrop REQUIRED)

if (CMAKE_TESTING_ENABLED)
  enable_testing()
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()


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

file(GLOB_RECURSE brains2_common_source_files src/common/*.cpp)
ament_auto_add_library(brains2_common SHARED ${brains2_common_source_files})
target_link_libraries(brains2_common Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)

if (CMAKE_TESTING_ENABLED)
  ament_add_gtest(test_track_database tests/common/test_track_database.cpp)
  target_link_libraries(test_track_database brains2_common)

  ament_add_gtest(test_tracks tests/common/test_tracks.cpp)
  target_link_libraries(test_tracks brains2_common)

  ament_add_gtest(test_spline_fittings tests/estimation/spline_fitting/test_spline_fitting.cpp)
  target_link_libraries(test_spline_fittings brains2_common brains2_estimation)
endif()

####################################################################################################
# sim
####################################################################################################
file(GLOB_RECURSE generated_sources src/sim/generated/*.c)
message(STATUS "Generated sources: ${generated_sources}")
ament_auto_add_library(brains2_sim SHARED src/sim/sim.cpp ${generated_sources})
target_link_libraries(brains2_sim Eigen3::Eigen acados)
target_include_directories(brains2_sim PUBLIC src/sim)

ament_auto_add_executable(sim_node src/sim/sim_node.cpp)
target_link_libraries(sim_node
  "${cpp_typesupport_target}" # for custom message and service types defined in this package
  yaml-cpp::yaml-cpp
  brains2_common
  brains2_sim
)

# if (CMAKE_TESTING_ENABLED)
#   ament_add_gtest(test_sim tests/common/test_sim.cpp)
#   target_link_libraries(test_sim brains2_common brains2_sim)
# endif()

####################################################################################################
# control
####################################################################################################

ament_auto_add_library(brains2_control SHARED src/control/controller.cpp)
target_link_libraries(brains2_control m casadi fatrop)

ament_auto_add_executable(control_node src/control/control_node.cpp)
target_link_libraries(control_node
  "${cpp_typesupport_target}" # for custom message and service types defined in this package
  yaml-cpp::yaml-cpp
  brains2_common
  brains2_control
  brains2_sim
)

if (CMAKE_TESTING_ENABLED)
  ament_add_gtest(test_controller tests/control/test_controller.cpp)
  target_link_libraries(test_controller brains2_common brains2_control)
endif()

####################################################################################################
# estimation
####################################################################################################
ament_auto_add_library(brains2_estimation SHARED
    src/estimation/center_line_estimation.cpp
)
target_link_libraries(brains2_estimation OsqpEigen::OsqpEigen osqp::osqp)

ament_auto_add_executable(brains2_be_spline_test src/estimation/spline_fitting/work_spline.cpp)
target_link_libraries(brains2_be_spline_test brains2_common brains2_estimation)

ament_auto_add_executable(fake_track_estimation_node src/estimation/fake_track_estimation_node.cpp)
target_link_libraries(fake_track_estimation_node
  "${cpp_typesupport_target}" # for custom message and service types defined in this package
  brains2_common
)

####################################################################################################
# SLAM
####################################################################################################
ament_auto_add_executable(fake_slam_node src/slam/fake_slam_node.cpp)
target_link_libraries(fake_slam_node
  "${cpp_typesupport_target}" # for custom message and service types defined in this package
  yaml-cpp::yaml-cpp
  brains2_common
)

####################################################################################################
# create package
####################################################################################################

ament_auto_package(INSTALL_TO_SHARE)
