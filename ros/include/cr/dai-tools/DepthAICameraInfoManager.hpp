#pragma once

#include "cr/dai-tools/ros_headers.h"
#include "virtual_camera_info_manager.h"
#include <depthai/device/Device.hpp>

class ROS_HELPER_EXPORT DepthaiCameraInfoManager : public shim::camera_info_manager::CameraInfoManager {
protected:
    void spin();
    ros_impl::Timer spin_timer;
    dai::CalibrationHandler& calibrationHandler;

    tf2_ros::TransformBroadcaster broadcaster;
    dai::Device &device;
    enum dai::CameraBoardSocket socket;

    int width; int height;
    dai::Point2f topLeftPixelId;
    dai::Point2f bottomRightPixelId;
    bool loadCalibrationFlash(const std::string &flashURL, const std::string &cname) override;

    bool saveCalibrationFlash(const ros_impl::sensor_msgs::CameraInfo &new_info, const std::string &flashURL,
                              const std::string &cname) override;

    DepthaiCameraInfoManager(dai::Device &device, dai::CalibrationHandler& calibrationHandler, enum dai::CameraBoardSocket socket, ros_impl::Node nh,
                             const std::string &cname="camera", const std::string &url="flash:///", int width = -1, int height = -1,
                             dai::Point2f topLeftPixelId = { },
                             dai::Point2f bottomRightPixelId = { });

public:
    dai::Device &Device() { return device; }
    enum dai::CameraBoardSocket Socket() { return socket; }
    std::string cname() const { return camera_name_; }
    static std::shared_ptr<DepthaiCameraInfoManager> get(dai::Device &device, dai::CalibrationHandler& calibrationHandler, enum dai::CameraBoardSocket socket, ros_impl::Node nh,
            const std::string &cname="camera", const std::string &url="flash:///",int width = -1, int height = -1,
                                                         dai::Point2f topLeftPixelId = { },
                                                         dai::Point2f bottomRightPixelId = { });
    static std::shared_ptr<DepthaiCameraInfoManager> get(dai::Device &device, enum dai::CameraBoardSocket socket);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

};
