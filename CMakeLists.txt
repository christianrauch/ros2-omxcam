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

find_package(PkgConfig)
pkg_search_module(AVCODEC REQUIRED libavcodec)
include_directories(${AVCODEC_INCLUDE_DIRS})
link_directories(${AVCODEC_LIBDIR})
pkg_search_module(AVUTIL REQUIRED libavutil)
include_directories(${AVUTIL_INCLUDE_DIRS})
link_directories(${AVUTIL_LIBDIR})

add_definitions(-DOMX_SKIP64BIT)

add_executable(picam_node src/picam_node.cpp)
ament_target_dependencies(picam_node rclcpp sensor_msgs)
target_link_libraries(picam_node ${OMXCAM_LIBRARIES})

add_executable(video_node src/video_node.cpp)
ament_target_dependencies(video_node rclcpp sensor_msgs)
target_link_libraries(video_node ${OMXCAM_LIBRARIES})
target_link_libraries(video_node ${AVCODEC_LIBRARIES} ${AVUTIL_LIBRARIES})

install(TARGETS
  picam_node video_node
  DESTINATION lib/${PROJECT_NAME}
)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

ament_package()
