cmake_minimum_required(VERSION 3.5)
project(picam)

# enable C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(omxcam REQUIRED)
include_directories(${OMXCAM_INCLUDE_DIRS})
link_directories(${OMXCAM_LIBRARY_DIRS})

add_definitions(-DOMX_SKIP64BIT)

add_executable(picam_node src/picam_node.cpp)
ament_target_dependencies(picam_node rclcpp sensor_msgs)

install(TARGETS
  picam_node
  DESTINATION lib/${PROJECT_NAME}
)
target_link_libraries(picam_node ${OMXCAM_LIBRARIES})

ament_package()
