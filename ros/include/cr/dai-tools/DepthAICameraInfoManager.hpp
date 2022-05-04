#pragma once

#include "ros/ros.h"
#include "virtual_camera_info_manager.h"
#include <depthai/device/Device.hpp>
#include <sensor_msgs/CameraInfo.h>

class ROS_HELPER_EXPORT DepthaiCameraInfoManager : public shim::camera_info_manager::CameraInfoManager {

public:
    static void spin();
    ros::Timer spin_timer;

    dai::Device &device;
    enum dai::CameraBoardSocket socket;
    bool loadCalibrationFlash(const std::string &flashURL, const std::string &cname) override;

    bool saveCalibrationFlash(const sensor_msgs::CameraInfo &new_info, const std::string &flashURL,
                              const std::string &cname) override;

    DepthaiCameraInfoManager(dai::Device &device, enum dai::CameraBoardSocket socket, ros::NodeHandle nh,
                             const std::string &cname="camera", const std::string &url="flash:///");
public:
    std::string cname() const { return camera_name_; }
    static std::shared_ptr<DepthaiCameraInfoManager> get(dai::Device &device, enum dai::CameraBoardSocket socket, ros::NodeHandle nh,
            const std::string &cname="camera", const std::string &url="flash:///");
    static std::shared_ptr<DepthaiCameraInfoManager> get(dai::Device &device, enum dai::CameraBoardSocket socket);

};
