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
    std::shared_ptr<dai::Device> device;
    enum dai::CameraBoardSocket socket;


    dai::Point2f topLeftPixelId;
    dai::Point2f bottomRightPixelId;
    bool setCameraInfoService(ros_impl::sensor_msgs::SetCameraInfo::Request &req, ros_impl::sensor_msgs::SetCameraInfo::Response &rsp) override;

    bool loadCalibrationFlash(const std::string &flashURL, const std::string &cname) override;

    bool saveCalibrationFlash(const ros_impl::sensor_msgs::CameraInfo &new_info, const std::string &flashURL,
                              const std::string &cname) override;

    DepthaiCameraInfoManager(const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationHandler, enum dai::CameraBoardSocket socket, ros_impl::Node nh,
                             const std::string &cname="camera", const std::string &url="flash:///", int width = -1, int height = -1,
                             dai::Point2f topLeftPixelId = { },
                             dai::Point2f bottomRightPixelId = { });
   int width; int height;
public:
    ros_impl::sensor_msgs::CameraInfo getCameraInfo(void) override;
    void setSize(int width, int height);
public:
    ros_impl::sensor_msgs::CameraInfo getCameraInfo(bool isRectified);
    dai::Device &Device() { return *device; }
    enum dai::CameraBoardSocket Socket() { return socket; }
    std::string cname() const { return camera_name_; }
    static std::shared_ptr<DepthaiCameraInfoManager> get(const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationHandler, enum dai::CameraBoardSocket socket, ros_impl::Node nh,
            const std::string &cname="camera", const std::string &url="flash:///",int width = -1, int height = -1,
                                                         dai::Point2f topLeftPixelId = { },
                                                         dai::Point2f bottomRightPixelId = { });
    static std::shared_ptr<DepthaiCameraInfoManager> get(const std::shared_ptr<dai::Device> &device, enum dai::CameraBoardSocket socket);


};
