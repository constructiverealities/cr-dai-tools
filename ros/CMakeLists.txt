project(cr_dai_ros)

find_package(depthai REQUIRED)
if(NOT TARGET cr-dai-tools)
    message("Building as catkin workspace; adding tools dir")
    add_subdirectory(../dai-tools dai-tools)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp rosbag sensor_msgs cv_bridge tf camera_info_manager image_transport tf2 catkin
        dynamic_reconfigure geometry_msgs)

if(catkin_FOUND)
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
            CATKIN_DEPENDS geometry_msgs message_runtime sensor_msgs std_msgs dynamic_reconfigure
    )

    include_directories(include)
    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${cr_dai_ros_INCLUDE_DIRS})

    add_library(DepthaiCameraInfoManager STATIC ./src/DepthAICameraInfoManager.cpp ./src/virtual_camera_info_manager.cc)
    target_link_libraries(DepthaiCameraInfoManager depthai::core ${catkin_LIBRARIES})
    target_include_directories(DepthaiCameraInfoManager PUBLIC include ${depthai_INCLUDE_DIRS})

    add_library(cr-dai-rosnode-lib STATIC src/PipelinePublisher.cc src/ImagePublisher.cc src/ToFDepthPublisher.cc src/IMUPublisher.cc src/NNPublisher.cc)
    target_link_libraries(cr-dai-rosnode-lib cr-dai-tools ${catkin_LIBRARIES} DepthaiCameraInfoManager)
    message(${cr_dai_ros_EXPORTED_TARGETS})
    add_dependencies(cr-dai-rosnode-lib ${cr_dai_ros_EXPORTED_TARGETS})

    add_executable(autonode apps/autonode.cc)
    target_link_libraries(autonode cr-dai-rosnode-lib cr-dai-tools)

    install(TARGETS autonode
            DESTINATION bin)

else()
    message("Not building ROS tools...")
endif()