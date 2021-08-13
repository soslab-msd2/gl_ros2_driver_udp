cmake_minimum_required(VERSION 3.5)
project(gl_ros2_driver_udp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
    include
)


file(GLOB serial_src "include/serial/*.cc")
if (NOT TARGET dl_driver)
add_library(serial SHARED ${serial_src})
endif()
if (NOT TARGET gl_driver)
add_library(gl_driver SHARED include/gl_driver.cpp)
endif()


add_executable(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    gl_driver
    serial
)
ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp
    sensor_msgs
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()