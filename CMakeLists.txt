cmake_minimum_required(VERSION 3.8)
project(voris_log)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4.4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
include_directories(include)
if (TARGET tf2_geometry_msgs::tf2_geometry_msgs)
    get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else ()
    set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif ()

add_executable(pose_log src/message_filter/pose_node.cpp src/message_filter/pose_register.cpp)
target_include_directories(pose_log PRIVATE include)
ament_target_dependencies(pose_log geometry_msgs message_filters rclcpp)


add_executable(posestamped_cb src/PoseStamped/posestamped_cb.cpp)
ament_target_dependencies(posestamped_cb geometry_msgs rclcpp)

add_executable(posearray_cb src/PoseArray/posearray_cb.cpp)
ament_target_dependencies(posearray_cb geometry_msgs rclcpp)


add_executable(stereo_save src/Image/stereo_img_save.cpp)
ament_target_dependencies(stereo_save image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(stereo_save ${OpenCV_LIBS})

install(TARGETS
        pose_log
        posestamped_cb
        posearray_cb
        stereo_save
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()


ament_package()
