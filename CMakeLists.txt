cmake_minimum_required(VERSION 3.8)
project(global_map_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(libpointmatcher_ros REQUIRED)
find_package(norlab_icp_mapper_ros REQUIRED)
find_package(libpointmatcher REQUIRED)

include_directories(
    ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    libpointmatcher_ros
    norlab_icp_mapper_ros
)
target_link_libraries(${PROJECT_NAME}_node
    ${libpointmatcher_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
