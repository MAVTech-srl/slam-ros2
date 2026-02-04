cmake_minimum_required(VERSION 3.8)
project(multistereo_planning_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

# nlohmann-json is header-only. On Ubuntu: sudo apt install nlohmann-json3-dev
find_package(nlohmann_json REQUIRED)

# ---- Library as component
add_library(multistereo_planning_cloud_component SHARED
  src/multistereo_planning_cloud_node.cpp
)

target_compile_features(multistereo_planning_cloud_component PUBLIC cxx_std_17)

target_include_directories(multistereo_planning_cloud_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(multistereo_planning_cloud_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)

target_link_libraries(multistereo_planning_cloud_component
  ${OpenCV_LIBS}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

# Register component (class name must match the one in your .cpp)
# In our code: multicam_depth::MultiStereoPlanningCloudNode
rclcpp_components_register_nodes(multistereo_planning_cloud_component
  "multicam_depth::MultiStereoPlanningCloudNode"
)

# (Optional) executable that runs the component standalone (handy for debug)
add_executable(multistereo_planning_cloud_node
  src/multistereo_planning_cloud_node.cpp
)
target_compile_features(multistereo_planning_cloud_node PUBLIC cxx_std_17)
ament_target_dependencies(multistereo_planning_cloud_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(multistereo_planning_cloud_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

# ---- Install
install(TARGETS
  multistereo_planning_cloud_component
  multistereo_planning_cloud_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
