cmake_minimum_required(VERSION 3.8)
project(laneletvisualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Find dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)                                       # :contentReference[oaicite:3]{index=3}
find_package(rclcpp REQUIRED)                                           # your ROS client library
find_package(lanelet2_core REQUIRED)                                    # core Lanelet2 types :contentReference[oaicite:4]{index=4}
find_package(lanelet2_io REQUIRED)                                      # map I/O (.osm) :contentReference[oaicite:5]{index=5}
find_package(lanelet2_projection REQUIRED)                              # coordinate projection :contentReference[oaicite:6]{index=6}
find_package(visualization_msgs REQUIRED)

# -----------------------------------------------------------------------------
# Executable
# -----------------------------------------------------------------------------
add_executable(laneletviz src/laneletviz.cpp)

# include your own headers
target_include_directories(laneletviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# require C++17
target_compile_features(laneletviz PUBLIC cxx_std_17)

# Link against ROS 2 and Lanelet2
ament_target_dependencies(laneletviz
  rclcpp
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  visualization_msgs
)

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------
install(TARGETS laneletviz
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# Testing (unchanged)
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
