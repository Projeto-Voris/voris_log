cmake_minimum_required(VERSION 3.8)
project(voris_log)

# Padrão C++17 e Flags de Compilação
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# -----------------------------------------------------------------------------
# Encontrar Dependências
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(marine_acoustic_msgs REQUIRED)

# Inclui a pasta local de headers para todos os alvos abaixo
include_directories(include)

# -----------------------------------------------------------------------------
# Geração de Mensagens/Serviços Customizados
# -----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SaveImages.srv"
    DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# -----------------------------------------------------------------------------
# Nós / Executáveis Tradicionais
# -----------------------------------------------------------------------------

# Dados / Pose / Odom
add_executable(pose_log src/data/pose_node.cpp src/data/pose_register.cpp)
ament_target_dependencies(pose_log geometry_msgs message_filters rclcpp)

add_executable(posestamped_cb src/data/posestamped_cb.cpp)
ament_target_dependencies(posestamped_cb geometry_msgs rclcpp)

add_executable(odom_node src/data/odom_node.cpp)
ament_target_dependencies(odom_node geometry_msgs nav_msgs rclcpp)

add_executable(posearray_cb src/data/posearray_cb.cpp)
ament_target_dependencies(posearray_cb geometry_msgs rclcpp)

# Imagens e Visão
add_executable(save_dt_images src/image/save_images_dt.cpp)
ament_target_dependencies(save_dt_images image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(save_dt_images ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(multicam_save src/image/multicam_save.cpp)
ament_target_dependencies(multicam_save image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(multicam_save ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(rescale src/image/rescale.cpp)
ament_target_dependencies(rescale image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(rescale ${OpenCV_LIBS})

add_executable(camera_img_save src/image/single_image_save.cpp)
ament_target_dependencies(camera_img_save image_transport cv_bridge rclcpp sensor_msgs)
target_link_libraries(camera_img_save ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(stereo_view src/image/stereo_image_view.cpp)
ament_target_dependencies(stereo_view image_transport cv_bridge message_filters rclcpp sensor_msgs std_srvs)
target_link_libraries(stereo_view ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(images_acquisition src/image/image_acquisition.cpp)
ament_target_dependencies(images_acquisition message_filters rclcpp std_srvs)
target_link_libraries(images_acquisition "${cpp_typesupport_target}")

add_executable(stereo_tools src/stereo_tools/stereo_tools.cpp)
ament_target_dependencies(stereo_tools image_transport cv_bridge message_filters rclcpp sensor_msgs)
target_link_libraries(stereo_tools ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(undistort_img src/image/undistort_image.cpp)
ament_target_dependencies(undistort_img image_transport cv_bridge rclcpp sensor_msgs)
target_link_libraries(undistort_img ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(stereo_rectify src/image/stereo_rectify.cpp)
ament_target_dependencies(stereo_rectify image_transport cv_bridge rclcpp sensor_msgs message_filters)
target_link_libraries(stereo_rectify ${OpenCV_LIBS} "${cpp_typesupport_target}")

add_executable(compressed2raw src/image/compressed2raw.cpp)
ament_target_dependencies(compressed2raw rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(compressed2raw ${OpenCV_LIBS} "${cpp_typesupport_target}")

# -----------------------------------------------------------------------------
# Componentes (Shared Libraries compartilhadas pelo rclcpp_components)
# -----------------------------------------------------------------------------

# Image Processor Component
add_library(image_processor_component SHARED src/image/image_processor.cpp)
ament_target_dependencies(image_processor_component rclcpp rclcpp_components sensor_msgs image_transport cv_bridge OpenCV)
rclcpp_components_register_node(image_processor_component PLUGIN "voris_log::ImageProcessorNode" EXECUTABLE image_processor_node)

# Stereo Save Component
add_library(stereo_save_component SHARED src/image/stereo_img_save.cpp)
target_link_libraries(stereo_save_component ${OpenCV_LIBS} "${cpp_typesupport_target}")
ament_target_dependencies(stereo_save_component rclcpp rclcpp_components sensor_msgs image_transport cv_bridge message_filters nav_msgs OpenCV)
rclcpp_components_register_node(stereo_save_component PLUGIN "voris_log::ImageSaver" EXECUTABLE stereo_save_node)

# DVL Component
add_library(dvl_node_component SHARED src/data/dvl_node.cpp)
ament_target_dependencies(dvl_node_component rclcpp rclcpp_components geometry_msgs marine_acoustic_msgs tf2_geometry_msgs tf2_ros)
rclcpp_components_register_node(dvl_node_component PLUGIN "dvl_log::DVLNode" EXECUTABLE dvl_node)

# Stereo Sonar / PCL Component
add_library(stereo_sonar_component SHARED src/image/stereo_sonar.cpp)
target_include_directories(stereo_sonar_component PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(stereo_sonar_component ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(stereo_sonar_component rclcpp rclcpp_components sensor_msgs message_filters cv_bridge pcl_conversions std_srvs nav_msgs)
rclcpp_components_register_node(stereo_sonar_component PLUGIN "voris_log::DataSaverNode" EXECUTABLE stereo_sonar_node)

add_library(dvl2mavros_node_component SHARED src/data/dvl_mavros_node.cpp)
ament_target_dependencies(dvl2mavros_node_component rclcpp rclcpp_components geometry_msgs marine_acoustic_msgs nav_msgs tf2_geometry_msgs tf2_ros)
rclcpp_components_register_node(dvl2mavros_node_component PLUGIN "voris_log::DVL2MavrosNode" EXECUTABLE dvl_mavros_node)
# -----------------------------------------------------------------------------
# Regras de Instalação (Install)
# -----------------------------------------------------------------------------

# Bibliotecas de Componentes
install(TARGETS 
    image_processor_component 
    stereo_sonar_component 
    dvl_node_component 
    dvl2mavros_node_component
    stereo_save_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executáveis Comuns
install(TARGETS
    pose_log
    posestamped_cb
    posearray_cb
    odom_node
    stereo_view
    save_dt_images
    images_acquisition
    camera_img_save
    stereo_tools
    rescale
    multicam_save
    undistort_img
    stereo_rectify
    compressed2raw
  DESTINATION lib/${PROJECT_NAME}
)

# Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Scripts Python
install(PROGRAMS
    scripts/noise_image.py
    scripts/point_cloud_las.py
    scripts/point_cloud_csv.py
    scripts/mp4_converter.py
    scripts/camerainfo_publisher.py
    scripts/sim_keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testes e Finalização do Pacote
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()