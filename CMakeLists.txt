cmake_minimum_required(VERSION 3.8)
project(voris_log)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# You should set the PYTHONPATH to your own python site-packages path
#set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

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
find_package(OpenCV 4.5.4 EXACT)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
include_directories(include)


if (TARGET tf2_geometry_msgs::tf2_geometry_msgs)
    get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else ()
    set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif ()


rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SaveImages.srv"
    #"srv/Test.srv"
        DEPENDENCIES builtin_interfaces std_msgs)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


add_executable(pose_log src/message_filter/pose_node.cpp src/message_filter/pose_register.cpp)
target_include_directories(pose_log PRIVATE include)
ament_target_dependencies(pose_log geometry_msgs message_filters rclcpp)


add_executable(posestamped_cb src/PoseStamped/posestamped_cb.cpp)
ament_target_dependencies(posestamped_cb geometry_msgs rclcpp)

add_executable(posearray_cb src/PoseArray/posearray_cb.cpp)
ament_target_dependencies(posearray_cb geometry_msgs rclcpp)


add_executable(stereo_save src/Image/stereo_img_save.cpp )
target_include_directories(stereo_save PRIVATE include)
ament_target_dependencies(stereo_save image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(stereo_save ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(camera_img_save src/Image/single_image_save.cpp )
target_include_directories(camera_img_save PRIVATE include)
ament_target_dependencies(camera_img_save image_transport cv_bridge rclcpp sensor_msgs)
target_link_libraries(camera_img_save ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(stereo_view src/Image/stereo_image_view.cpp )
target_include_directories(stereo_view PRIVATE include)
ament_target_dependencies(stereo_view image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(stereo_view ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(images_acquisition src/handle_node/image_acquisition.cpp )
target_include_directories(images_acquisition PRIVATE include)
ament_target_dependencies(images_acquisition message_filters rclcpp std_srvs)
target_link_libraries(images_acquisition "${cpp_typesupport_target}")

add_executable(stereo_tools src/stereo_tools/stereo_tools.cpp )
target_include_directories(stereo_tools PRIVATE include)
ament_target_dependencies(stereo_tools image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(stereo_tools ${OpenCV_LIBS} "${cpp_typesupport_target}")

install(TARGETS
        pose_log
        posestamped_cb
        posearray_cb
        stereo_save
        stereo_view
        images_acquisition
        camera_img_save
        stereo_tools
        DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(
        PROGRAMS
        scripts/noise_image.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()


ament_package()
