cmake_minimum_required(VERSION 3.15)
project(brains2)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_DEBUG} -O2")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)
add_compile_options(--system-header-prefix=brains2/external/)

add_compile_definitions(TRACK_DATABASE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../track_database")
add_compile_definitions(CAR_CONSTANTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../car_constants.yaml")

set(BUILD_TESTING ON)

####################################################################################################
# dependencies
####################################################################################################

# find ROS 2 dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

# find other libraries
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
# the following assumes acados has already been installed (automatically done in build.sh)
find_package(acados REQUIRED PATHS ${CMAKE_INSTALL_PREFIX}/../acados/cmake)
find_package(casadi REQUIRED)
find_package(fatrop REQUIRED)

if (BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()


####################################################################################################
# custom message and service types
####################################################################################################
set(msg_files
  "msg/Acceleration.msg"
  "msg/ControllerDebugInfo.msg"
  "msg/Controls.msg"
  "msg/FSM.msg"
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
rosidl_get_typesupport_target(brains2_msg ${PROJECT_NAME} rosidl_typesupport_cpp)
message(STATUS "brains2_msg: ${brains2_msg}")

####################################################################################################
# common
####################################################################################################

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

file(GLOB_RECURSE brains2_common_source_files CONFIGURE_DEPENDS src/common/*.cpp)
ament_auto_add_library(brains2_common SHARED ${brains2_common_source_files})
target_link_libraries(brains2_common Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)

if(BUILD_TESTING)
  ament_add_gtest(test_track_database tests/common/test_track_database.cpp)
  target_link_libraries(test_track_database brains2_common)

  ament_add_gtest(test_track tests/common/test_track.cpp)
  target_link_libraries(test_track brains2_common)

  ament_add_gtest(test_spline_fittings tests/estimation/spline_fitting/test_spline_fitting.cpp)
  target_link_libraries(test_spline_fittings brains2_common brains2_estimation)
endif()

####################################################################################################
# sim
####################################################################################################
file(GLOB_RECURSE brains2_sim_generated_sources CONFIGURE_DEPENDS src/sim/generated/*.c)
message(STATUS "Generated sim sources: ${brains2_sim_generated_sources}")
ament_auto_add_library(brains2_sim SHARED src/sim/sim.cpp ${brains2_sim_generated_sources})
target_link_libraries(brains2_sim Eigen3::Eigen acados)
target_include_directories(brains2_sim PUBLIC src/sim)

ament_auto_add_executable(sim_node src/sim/sim_node.cpp)
target_link_libraries(sim_node
  ${brains2_msg}
  brains2_common
  brains2_sim
  yaml-cpp::yaml-cpp
)

####################################################################################################
# control
####################################################################################################

ament_auto_add_library(brains2_control SHARED src/control/high_level_controller.cpp src/control/low_level_controller.cpp)
target_link_libraries(brains2_control
  brains2_common
  brains2_sim
  m
  casadi
  fatrop
)

ament_auto_add_executable(control_node src/control/control_node.cpp)
target_link_libraries(control_node
  ${brains2_msg}
  brains2_common
  brains2_control
  yaml-cpp::yaml-cpp
)

if(BUILD_TESTING)
  ament_add_gtest(test_high_level_controller tests/control/test_high_level_controller.cpp)
  target_link_libraries(test_high_level_controller brains2_common brains2_control)
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
  ${brains2_msg}
  brains2_common
)

####################################################################################################
# SLAM
####################################################################################################
ament_auto_add_executable(fake_slam_node src/slam/fake_slam_node.cpp)
target_link_libraries(fake_slam_node
  ${brains2_msg}
  yaml-cpp::yaml-cpp
  brains2_common
)

####################################################################################################
# coordination
####################################################################################################
ament_auto_add_executable(coordination_node src/coordination/coordination_node.cpp)
target_link_libraries(coordination_node
  ${brains2_msg}
  brains2_common
)


####################################################################################################
# linting
####################################################################################################
# NOTE: this must happen before the invocation of ament_package()
if(BUILD_TESTING)
  # find all files to exclude from lint
  # NOTE: all the command line invocations done by ament will actually be performed via subprocess.Popen
  # (see https://github.com/ament/ament_cmake/blob/1b66f6c25f7d6ed98b3aadafe57af0df6ce91573/ament_cmake_test/ament_cmake_test/__init__.py#L201)
  # which does not seem to support shell extensions such as globbing. We therefore have to manually
  # specify the files to exclude.
  file(GLOB_RECURSE FILES_EXCLUDED_FROM_LINT CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/src/sim/generated/*
    ${CMAKE_SOURCE_DIR}/src/sim/generated/dyn6_model/*
    ${CMAKE_SOURCE_DIR}/src/sim/generated/kin6_model/*
    ${CMAKE_SOURCE_DIR}/include/brains2/external/*
    ${CMAKE_SOURCE_DIR}/imgui/*
    ${CMAKE_SOURCE_DIR}/imgui/backends/*
  )
  message(STATUS "FILES_EXCLUDED_FROM_LINT: ${FILES_EXCLUDED_FROM_LINT}")

  # find files to lint
  # same note as above
  file(GLOB_RECURSE FILES_TO_LINT CONFIGURE_DEPENDS
    *.cpp
    *.hpp
    *.c
    *.h
  )
  list(REMOVE_ITEM FILES_TO_LINT ${FILES_EXCLUDED_FROM_LINT})
  message(STATUS "FILES_TO_LINT: ${FILES_TO_LINT}")

  # check copyright headers
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright(EXCLUDE ${FILES_EXCLUDED_FROM_LINT})

  # find clang-format config file (at the root of the workspace)
  set(CLANG_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../../.clang-format")
  get_filename_component(CLANG_CONFIG_FILE "${CLANG_CONFIG_FILE}" ABSOLUTE)
  message(STATUS "CLANG_CONFIG_FILE: ${CLANG_CONFIG_FILE}")

  # check formatting with clang-format
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${CLANG_CONFIG_FILE}" "${FILES_TO_LINT}")

  # lint with cpplint
  # NOTE: we use a custom implementation of ament_cmake_cpplint that supports
  # providing several patterns/files in EXCLUDE
  include(cmake/ament_cpplint.cmake)
  ament_cpplint(EXCLUDE "${FILES_EXCLUDED_FROM_LINT}" FILTERS "-readability/todo")
endif()


####################################################################################################
# tools
####################################################################################################

# Fetch Dear ImGui
include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)

# Fetch ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(implot)

# Find SDL2
find_package(SDL2 REQUIRED)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Create imgui library
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC SDL2::SDL2 OpenGL::GL)
if(APPLE)
    target_compile_definitions(imgui_lib PUBLIC GL_SILENCE_DEPRECATION)
endif()

# Create implot library
add_library(implot_lib STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)
target_include_directories(implot_lib PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot_lib PUBLIC imgui_lib)

# Create the MPC tuning GUI executable
add_executable(mpc_tuning_gui src/tools/mpc_tuning_gui.cpp)
target_link_libraries(mpc_tuning_gui PRIVATE
    imgui_lib
    implot_lib
    Eigen3::Eigen
    brains2_common
    brains2_control
    brains2_sim
)


####################################################################################################
# create package
####################################################################################################

ament_auto_package(INSTALL_TO_SHARE)
