cmake_minimum_required(VERSION 3.10)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_CXX_STANDARD 17)

if(NOT TARGET cr-dai-tools)
    message("Building as catkin workspace; adding tools dir")
    add_subdirectory(../dai-tools dai-tools)
endif()
project(cr_dai_ros)
if(NOT TARGET depthai::core)
    find_package(depthai REQUIRED)
endif()

SET(ROS_DEPENDENCIES sensor_msgs tf2 geometry_msgs std_msgs tf2_ros camera_info_manager )
SET(ROS1_DEPENDENCIES ${ROS_DEPENDENCIES} catkin roscpp dynamic_reconfigure)
SET(ROS2_DEPENDENCIES ${ROS_DEPENDENCIES})

find_package(catkin)
find_package(ament_cmake)

SET(BUILD_ROS OFF)
if(ament_cmake_FOUND)
    SET(BUILD_ROS ON)

    find_package(rosidl_default_generators REQUIRED)
    foreach(d ${ROS2_DEPENDENCIES})
        find_package(${d} REQUIRED)
    endforeach()

    include_directories(${ament_INCLUDE_DIRS}})
#
#    rosidl_generate_interfaces(${PROJECT_NAME}
#            ./msg/CameraMetadata.msg
#            DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
#            LIBRARY_NAME ${PROJECT_NAME}
#    )

    ament_export_dependencies(rosidl_default_runtime ${ROS2_DEPENDENCIES})

    add_compile_options(-DHAS_ROS2=1)
elseif(catkin_FOUND)
    SET(BUILD_ROS ON)
    message("Building ROS tools... ${CATKIN_WORKSPACES}")

    SET(ROS_LIBRARIES)
    set(CMAKE_FIND_DEBUG_MODE TRUE)
    foreach(d ${ROS1_DEPENDENCIES})
        message("Looking for '${d}' ${${d}_FOUND}")
        find_package(${d} REQUIRED)
        message("Found ${${d}_FOUND} ${${d}_LIBRARIES}")
        LIST(APPEND ROS_LIBRARIES ${${d}_LIBRARIES})
    endforeach()

    set(CATKIN_WORKSPACES "$ENV{ROS_PATH}")
    string(REPLACE "\\" "/" CATKIN_WORKSPACES "${CATKIN_WORKSPACES}")

    add_message_files (
            FILES
            CameraMetadata.msg
    )

    generate_messages (
            DEPENDENCIES
           std_msgs dynamic_reconfigure
    )
    message(" ${cr_dai_ros_EXPORTED_TARGETS}")
    generate_dynamic_reconfigure_options(
            cfg/StereoDepth.cfg
            cfg/CameraControl.cfg
            cfg/DeviceControl.cfg
    )

    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME}
            CATKIN_DEPENDS ${ROS1_DEPENDENCIES}
    )

    include_directories(${CATKIN_WORKSPACES}/include)
    include_directories(include)
    include_directories(${catkin_INCLUDE_DIRS})
    include_directories(${cr_dai_ros_INCLUDE_DIRS})

    add_compile_options(-DHAS_ROS1=1)

    set(ROS_EXPORT_TARGETS ${cr_dai_ros_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    message("ROS libraries: ${ROS_LIBRARIES} ${ROS_EXPORT_TARGETS}")
    message("ROS includes: ${catkin_INCLUDE_DIRS} cr_dai_ros: ${cr_dai_ros_INCLUDE_DIRS} ros: ${CATKIN_WORKSPACES}/include")
    set(CMAKE_FIND_DEBUG_MODE FALSE)
else()
    message("Not building ROS tools...")
endif()

if(BUILD_ROS)
    include_directories(include)

    add_compile_options(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
    add_library(cr-dai-rosnode-lib STATIC src/PipelinePublisher.cc src/ImagePublisher.cc src/ToFDepthPublisher.cc
            src/IMUPublisher.cc src/NNPublisher.cc ./src/DepthAICameraInfoManager.cpp ./src/virtual_camera_info_manager.cc
            ./src/ros_impl.cc ./src/DepthPublisher.cc ./src/tx.cc)
    target_link_libraries(cr-dai-rosnode-lib cr-dai-tools ${ROS_LIBRARIES} depthai::core)
    target_include_directories(cr-dai-rosnode-lib PUBLIC include ${depthai_INCLUDE_DIRS})
    if(NOT "${ROS_EXPORT_TARGETS}" STREQUAL "")
        add_dependencies(cr-dai-rosnode-lib ${ROS_EXPORT_TARGETS})
    endif()

    add_executable(autonode apps/autonode.cc)
    target_link_libraries(autonode cr-dai-rosnode-lib cr-dai-tools)

    install(TARGETS autonode cr-dai-rosnode-lib
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            PUBLIC_HEADER DESTINATION include
            )
endif()

if(ament_cmake_FOUND)
    ament_export_include_directories(include)
    ament_export_libraries(depthai_bridge)
    ament_export_dependencies(${dependencies})

    ament_target_dependencies(cr-dai-rosnode-lib ${ROS2_DEPENDENCIES} rclcpp)

    ament_package()
elseif(catkin_FOUND)
endif()
