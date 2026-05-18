cmake_minimum_required(VERSION 3.8)
project(artgmap_vcs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics window system)
find_package(TGUI REQUIRED)

# =========================
# Include directories
# =========================
include_directories(
  include
)

# =========================
# Executable
# =========================
add_executable(artgmap_vcs_node
  src/main.cpp
  src/Map.cpp
  src/MapRosConection.cpp
  src/Astar.cpp
  src/GridMap.cpp
  src/MapFusion.cpp
  src/MapViewer.cpp
  src/MenuBar.cpp
  src/DragWindow.cpp
  src/FileManager.cpp
  src/RightClickMapMenu.cpp
  src/RobotCreator.cpp
  src/SaveMenu.cpp
  src/ViewController.cpp
  src/UnicicleWmr.cpp
  src/tinyfiledialogs.c
)

target_include_directories(artgmap_vcs_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# =========================
# Link libraries
# =========================
ament_target_dependencies(artgmap_vcs_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp   # ← THIS WAS MISSING
)

target_link_libraries(artgmap_vcs_node
  TGUI::TGUI
  sfml-graphics
  sfml-window
  sfml-system
)

# =========================
# Install
# =========================
install(TARGETS
  artgmap_vcs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets/
)

# =========================
# Testing
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
