cmake_minimum_required(VERSION 3.8)
project(aruco_pose_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(aruco_pose_publisher_node src/aruco_pose_publisher_node.cpp)
ament_target_dependencies(aruco_pose_publisher_node rclcpp geometry_msgs std_msgs)
target_include_directories(aruco_pose_publisher_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_pose_publisher_node ${OpenCV_LIBS})

install(TARGETS aruco_pose_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
