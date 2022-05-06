cmake_minimum_required(VERSION 3.10)

option(BUILD_DEPTHAI "Build all relevant dependencies" OFF)

include(FetchContent)
if(BUILD_DEPTHAI)
    # FetchContent_Declare(
    #       opencv
    #       GIT_REPOSITORY https://github.com/opencv/opencv.git
    #       GIT_TAG        4.4.0
    # 	)

    FetchContent_Declare(
            depthai-core
            GIT_REPOSITORY https://github.com/luxonis/depthai-core.git
            GIT_TAG tof_rgb_mono
    )

    #FetchContent_MakeAvailable(opencv)
    FetchContent_MakeAvailable(depthai-core)
    project(cr_dai_ros)
else()
    project(cr_dai_ros)
    find_package(depthai REQUIRED)
endif()

cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_STANDARD 17)

if(NOT TARGET cr-dai-tools)
    message("Building as catkin workspace; adding tools dir")
    add_subdirectory(../dai-tools dai-tools)
endif()

find_package(catkin COMPONENTS roscpp rosbag sensor_msgs cv_bridge tf camera_info_manager image_transport tf2 catkin
        dynamic_reconfigure geometry_msgs std_msgs message_generation tf2_ros)

find_package(ament_cmake)

SET(BUILD_ROS OFF)
if(ament_cmake_FOUND)
    SET(BUILD_ROS ON)

    #find_package(rmw_cyclonedds_cpp REQUIRED)
    #set(RMW_IMPLEMENTATION rmw_cyclonedds_cpp)
    #find_package(rmw_fastrtps_cpp REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)


    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(stereo_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(vision_msgs REQUIRED)
    find_package(camera_info_manager REQUIRED)

    include_directories(${ament_INCLUDE_DIRS}})

    #    find_package(rmw_connext_cpp REQUIRED)
    find_package(std_msgs REQUIRED)
    #find_package(std_interfaces REQUIRED)
#
#    rosidl_generate_interfaces(${PROJECT_NAME}
#            ./msg/CameraMetadata.msg
#            DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
#            LIBRARY_NAME ${PROJECT_NAME}
#    )

    ament_export_dependencies(rosidl_default_runtime cv_bridge
            image_transport
            rclcpp
            sensor_msgs
            stereo_msgs
            std_msgs
            tf2 tf2_ros
            vision_msgs camera_info_manager)

    add_compile_options(-DHAS_ROS2=1)
elseif(catkin_FOUND)
    SET(BUILD_ROS ON)
    message("Building ROS tools...")

    add_message_files (
            FILES
            CameraMetadata.msg
    )

    generate_messages (
            DEPENDENCIES
            geometry_msgs sensor_msgs std_msgs dynamic_reconfigure
    )
    generate_dynamic_reconfigure_options(
            cfg/StereoDepth.cfg
            cfg/CameraControl.cfg
            cfg/DeviceControl.cfg
    )

    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME}
            CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs dynamic_reconfigure tf2_ros
    )

    include_directories(include)
    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${cr_dai_ros_INCLUDE_DIRS})

    add_compile_options(-DHAS_ROS1=1)
else()
    message("Not building ROS tools...")
endif()

if(BUILD_ROS)
    include_directories(include)
    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${cr_dai_ros_INCLUDE_DIRS})

    add_library(cr-dai-rosnode-lib STATIC src/PipelinePublisher.cc src/ImagePublisher.cc src/ToFDepthPublisher.cc src/IMUPublisher.cc src/NNPublisher.cc ./src/DepthAICameraInfoManager.cpp ./src/virtual_camera_info_manager.cc ./src/ros_impl.cc)
    target_link_libraries(cr-dai-rosnode-lib cr-dai-tools ${catkin_LIBRARIES} depthai::core)
    target_include_directories(cr-dai-rosnode-lib PUBLIC include ${depthai_INCLUDE_DIRS})

    if(NOT "${cr_dai_ros_EXPORTED_TARGETS}" STREQUAL "")
        add_dependencies(cr-dai-rosnode-lib ${cr_dai_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()

    add_executable(autonode apps/autonode.cc)
    target_link_libraries(autonode cr-dai-rosnode-lib cr-dai-tools)

    install(TARGETS autonode
            DESTINATION bin)

endif()

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(depthai_bridge)
    ament_export_dependencies(${dependencies})

    ament_target_dependencies(cr-dai-rosnode-lib cv_bridge
            image_transport
            rclcpp
            sensor_msgs tf2 tf2_ros
            stereo_msgs
            std_msgs
            vision_msgs camera_info_manager)

    #rosidl_target_interfaces(DepthaiCameraInfoManager cr_dai_ros "rosidl_typesupport_cpp")
    #rosidl_target_interfaces(autonode ${PROJECT_NAME} rosidl_typesupport_cpp)
    #rosidl_target_interfaces(autonode ${PROJECT_NAME} rosidl_typesupport_fastrtps_cpp)

    ament_package()
elseif(catkin_FOUND)
endif()