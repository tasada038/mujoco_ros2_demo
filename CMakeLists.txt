cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenGL REQUIRED)

# GLFW (sudo apt install libglfw3-dev). Searched directly instead of find_package(glfw3),
# so that a stale glfw3Config.cmake (e.g. in /usr/local) cannot break the build.
find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h REQUIRED)
find_library(GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
message(STATUS "GLFW: ${GLFW_INCLUDE_DIR}  ${GLFW_LIBRARY}")

# ---------------------------------------------------------------------------
# MuJoCo C library
#   Default: the one shipped inside the Python package (pip install mujoco).
#   Override: colcon build --cmake-args -DMUJOCO_DIR=/path/to/mujoco
#             (a directory containing include/mujoco/mujoco.h and libmujoco.so*)
# ---------------------------------------------------------------------------
if(NOT MUJOCO_DIR)
  execute_process(
    COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
    OUTPUT_VARIABLE MUJOCO_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS ${MUJOCO_DIR}/include REQUIRED)
file(GLOB MUJOCO_LIBRARY ${MUJOCO_DIR}/libmujoco.so* ${MUJOCO_DIR}/lib/libmujoco.so*)
list(GET MUJOCO_LIBRARY 0 MUJOCO_LIBRARY)
message(STATUS "MuJoCo: ${MUJOCO_INCLUDE_DIR}  ${MUJOCO_LIBRARY}")
get_filename_component(MUJOCO_LIB_DIR ${MUJOCO_LIBRARY} DIRECTORY)

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------
add_library(mujoco_view_component SHARED src/mujoco_view_component.cpp)
target_include_directories(mujoco_view_component PRIVATE include ${MUJOCO_INCLUDE_DIR} ${GLFW_INCLUDE_DIR})
target_link_libraries(mujoco_view_component ${MUJOCO_LIBRARY} ${GLFW_LIBRARY} OpenGL::GL)
ament_target_dependencies(mujoco_view_component
  rclcpp rclcpp_components ament_index_cpp sensor_msgs geometry_msgs std_msgs tf2_ros)
set_target_properties(mujoco_view_component PROPERTIES INSTALL_RPATH "${MUJOCO_LIB_DIR}")
rclcpp_components_register_node(mujoco_view_component
  PLUGIN "mujoco_ros2::MujocoViewComponent"
  EXECUTABLE mujoco_viewer)

add_library(ant_walk_component SHARED src/ant_walk_component.cpp)
target_include_directories(ant_walk_component PRIVATE include)
ament_target_dependencies(ant_walk_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(ant_walk_component
  PLUGIN "mujoco_ros2::AntWalkComponent"
  EXECUTABLE ant_walker)

install(TARGETS mujoco_view_component ant_walk_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch models rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
