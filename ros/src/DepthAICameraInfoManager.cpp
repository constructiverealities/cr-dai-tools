#include "cr/dai-tools/ros_headers.h"
#include <memory>
#include "cr/dai-tools/DepthAICameraInfoManager.hpp"
#include "cr/dai-tools/Tx.h"

template<typename D_T>
static void copy(D_T &d, const std::vector<float> &s) {
    if (s.size() == 0) return;
    d.resize(s.size());
    for (size_t i = 0; i < s.size(); i++) {
        d[i] = s[i];
    }
}

template<typename D_T>
static void copy(D_T &d, const std::vector<std::vector<float>> &s) {
    if (s.size() == 0) return;
    for (size_t i = 0; i < d.size(); i++) {
        d[i] = s[i / s[0].size()][i % s[0].size()];
    }
}

static void copy(std::vector<std::vector<float>> &d, const tf2::Matrix3x3& s) {
    d.resize(3);
    for(int i = 0;i < 3;i++) {
        d[i].resize(3);
        for(int j = 0;j < 3;j++) {
            d[i][j] = s[i][j];
        }
    }
}

template<typename D_T>
static void copy43(D_T &d, const std::vector<std::vector<float>> &s) {
    if (s.size() == 0) return;
    for (int i = 0; i < 9; i++) {
        int y = i % s[0].size();
        int x = i / s[0].size();
        d[x * 4 + y] = s[x][y];
    }
}

static std::map<std::string, std::shared_ptr<DepthaiCameraInfoManager>> managers;

std::shared_ptr<DepthaiCameraInfoManager> DepthaiCameraInfoManager::get(dai::Device &device, enum dai::CameraBoardSocket socket) {
    std::string key = device.getMxId() + std::to_string((int)socket);
    return managers[key];
}



std::shared_ptr<DepthaiCameraInfoManager>
DepthaiCameraInfoManager::get(dai::Device &device, dai::CalibrationHandler& calibrationHandler, dai::CameraBoardSocket socket, ros_impl::Node nh,
                              const std::string &cname, const std::string &url, int width, int height,
                              dai::Point2f topLeftPixelId,
                              dai::Point2f bottomRightPixelId) {
    std::string key = device.getMxId() + std::to_string((int)socket);
    if(managers.find(key) == managers.end() || managers[key] == 0) {
        managers[key] = ::std::shared_ptr<DepthaiCameraInfoManager>(new DepthaiCameraInfoManager(device, calibrationHandler, socket, nh, cname, url, width, height, topLeftPixelId, bottomRightPixelId));
    }
    return managers[key];
}

static tf2_ros::Buffer& buffer(ros_impl::Node nh) {
    std::unique_ptr<tf2_ros::Buffer> buffer_ptr;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    if(!buffer_ptr) {
#if HAS_ROS2
        buffer_ptr = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
#else
        buffer_ptr = std::make_unique<tf2_ros::Buffer>();
#endif
        listener = std::make_unique<tf2_ros::TransformListener>(*buffer_ptr);
    }

    return *buffer_ptr;
}

DepthaiCameraInfoManager::DepthaiCameraInfoManager(dai::Device &device, dai::CalibrationHandler& calibrationHandler, dai::CameraBoardSocket socket,
                                                   ros_impl::Node nh, const std::string &cname, const std::string &url, int width, int height,
                                                   dai::Point2f topLeftPixelId,
                                                   dai::Point2f bottomRightPixelId)
        : shim::camera_info_manager::CameraInfoManager(nh, cname, url),
          device(device), calibrationHandler(calibrationHandler), socket(socket), width(width), height(height), topLeftPixelId(topLeftPixelId),
#if HAS_ROS2
          broadcaster(nh),
#endif
          bottomRightPixelId(bottomRightPixelId) {
    ROS_IMPL_INFO(nh, "Creating DAI camera info manager at %s/%s", ros_impl::Namespace(nh), cname.c_str());
    spin_timer = ros_impl::create_wall_timer(nh, .1, [this]() { this->spin(); });
    //spin_timer = nh.createTimer(ros::Duration(0.1),
}

static dai::CameraBoardSocket get_next_socket(dai::CameraBoardSocket socket) {
    switch(socket) {
        case dai::CameraBoardSocket::AUTO: return dai::CameraBoardSocket::LEFT;
        case dai::CameraBoardSocket::LEFT: return dai::CameraBoardSocket::RIGHT;
        case dai::CameraBoardSocket::RIGHT: return dai::CameraBoardSocket::RGB;
        case dai::CameraBoardSocket::RGB: return dai::CameraBoardSocket::CAM_D;
        default:
            return (dai::CameraBoardSocket)((int)socket + 1);
    }
}

static dai::CameraBoardSocket get_next_populated_socket(dai::Device &device, dai::CameraBoardSocket socket) {
    auto next_socket = get_next_socket(socket);
    std::string key = device.getMxId() + std::to_string((int)next_socket);
    if(managers[key]) {
        return next_socket;
    }
    if(next_socket > dai::CameraBoardSocket::CAM_H)
        return dai::CameraBoardSocket::CAM_H;
    return get_next_populated_socket(device, next_socket);
}

std::map<std::string, ros_impl::geometry_msgs::TransformStamped> device_transforms;
static void transmit_eeprom(const ros_impl::Node& n, dai::Device &device, dai::CalibrationHandler& calibrationData) {
    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

//        if(manager->Device().getMxId() != device.getMxId())
//            continue;

        auto next_socket = get_next_populated_socket(device, kv.second->Socket());
        auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket);
        if(!daiInfo)
            continue;
        auto cname = daiInfo->cname();
        try {
            auto extrinsics = calibrationData.getCameraExtrinsics( next_socket, kv.second->Socket());
            auto pose_msg = cr::dai_tools::tx::daiExtrinsics2Tx(extrinsics, cname, kv.second->cname());
            pose_msg.header.stamp = ros_impl::now(n);

            device_transforms[pose_msg.child_frame_id] = pose_msg;
            //tfBroadcaster().sendTransform(pose_msg);
            ROS_IMPL_INFO(n, "Broadcast tx between %s and %s", cname.c_str(), kv.second->cname().c_str());
        } catch(std::runtime_error& e) {
            ROS_IMPL_WARN(n, "Could not broadcast tx between %s and %s, %s", cname.c_str(), kv.second->cname().c_str(), e.what());
        }
    }
}

bool saveAllCalibrationData(const ros_impl::Node& n, dai::Device &device, dai::CalibrationHandler& calibrationHandler) {
    auto now = std::chrono::high_resolution_clock::now();
    calibrationHandler.eepromToJsonFile("camera_info_backup_" + std::to_string(now.time_since_epoch().count()) + ".json");

    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

        if(manager->Device().getMxId() != device.getMxId())
            continue;

        auto socket = manager->Socket();
        auto cname = manager->getCameraInfo().header.frame_id;
        auto new_info = manager->getCameraInfo();
        auto next_socket = get_next_populated_socket(device, socket);
        ROS_IMPL_INFO(n, "Saving calibration for %s Fx %f Fy %f Cx %f Cy %f %d %d", cname.c_str(), new_info.K[0],
                 new_info.K[4],
                 new_info.K[2], new_info.K[5], (int) socket, (int) next_socket);

        std::vector<std::vector<float>> intrinsics;
        intrinsics.resize(3);
        for (int i = 0; i < 3; i++) {
            intrinsics[i].resize(3);
            for (int j = 0; j < 3; j++) {
                intrinsics[i][j] = new_info.K[i * 3 + j];
            }
        }
        calibrationHandler.setCameraIntrinsics(socket, intrinsics, new_info.width, new_info.height);
        std::vector<float> D;
        for (auto &d : new_info.D) D.push_back(d);
        D.resize(14);
        calibrationHandler.setDistortionCoefficients(socket, D);

        if(auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket)) {
            auto ci = daiInfo->getCameraInfo();
            auto thisFrame = new_info.header.frame_id;
            auto nextFrame = ci.header.frame_id;
            assert(thisFrame != "" && nextFrame != "");
            std::string error_msg;

            auto lookupTime = ros_impl::Time(0);
            /// NOTE: lookupTransform is target, source. setCameraExtrinsics is source, target.
            if(buffer(n).canTransform(nextFrame, thisFrame, lookupTime), &error_msg) {
                //auto tx = buffer.lookupTransform(nextFrame, thisFrame, ros::Time(0));
                auto tx = buffer(n).lookupTransform(thisFrame, nextFrame, lookupTime);
                tf2::Matrix3x3 rot;
                rot.setRotation(tf2::Quaternion(tx.transform.rotation.x,
                                               tx.transform.rotation.y,
                                               tx.transform.rotation.z,
                                               tx.transform.rotation.w));
                std::vector<std::vector<float>> dai_rot;
                std::vector<float> translation = { (float)tx.transform.translation.x * 100.f,
                                                   (float)tx.transform.translation.y * 100.f,
                                                   (float)tx.transform.translation.z * 100.f};

                ROS_IMPL_INFO(n, "Saving extrinsics between %s and %s [%f %f %f] [%f %f %f %f]", cname.c_str(), nextFrame.c_str(),
                         translation[0], translation[1], translation[2], tx.transform.rotation.x,
                         tx.transform.rotation.y, tx.transform.rotation.z, tx.transform.rotation.w);
                copy(dai_rot, rot);
                calibrationHandler.setCameraExtrinsics(socket, static_cast<dai::CameraBoardSocket>(next_socket), dai_rot, translation, translation);
            } else {
                ROS_IMPL_WARN(n, "Could not find transform between %s and %s (%s)", nextFrame.c_str(), thisFrame.c_str(), error_msg.c_str());
                return false;
            }
        }

    }

    calibrationHandler.eepromToJsonFile("camera_info_" + std::to_string(now.time_since_epoch().count()) + ".json");
    try{
        transmit_eeprom(n, device, calibrationHandler);
        return device.flashCalibration(calibrationHandler);
    } catch(std::runtime_error& e) {
        ROS_IMPL_WARN(n, "%s", e.what());
        return false;
    }
}

static bool transmit_transforms = true;
void DepthaiCameraInfoManager::spin() {
    if(transmit_transforms) {
        for(auto& kv : device_transforms) {
            kv.second.header.stamp = ros_impl::now(nh_);
            broadcaster.sendTransform(kv.second);
        }
    }
}

bool DepthaiCameraInfoManager::setCameraInfoService(ros_impl::sensor_msgs::SetCameraInfo::Request &req, ros_impl::sensor_msgs::SetCameraInfo::Response &rsp) {
    if(transmit_transforms) {
        transmit_transforms = false;
        buffer(nh_).clear();
        ROS_IMPL_WARN(nh_, "set_camera_info invoked; turning off transmit mode on transforms and clearing applicable buffers");
    }
    return CameraInfoManager::setCameraInfoService(req, rsp);
}

bool
DepthaiCameraInfoManager::saveCalibrationFlash(const ros_impl::sensor_msgs::CameraInfo &_new_info, const std::string &flashURL,
                                               const std::string &cname) {
    auto now = std::chrono::high_resolution_clock::now();
    calibrationHandler.eepromToJsonFile("camera_info_backup_" + std::to_string(now.time_since_epoch().count()) + ".json");

    ros_impl::sensor_msgs::CameraInfo new_info = _new_info;
    new_info.header.frame_id = cname;
    this->cam_info_ = new_info;

    return saveAllCalibrationData(nh_, device, calibrationHandler);
}

bool DepthaiCameraInfoManager::loadCalibrationFlash(const std::string &flashURL, const std::string &cname) {
    auto saveData = calibrationHandler.getEepromData();
    auto camera_data = saveData.cameraData[socket];

    ros_impl::sensor_msgs::CameraInfo cameraInfo = {};
    cameraInfo.distortion_model = "rational_polynomial";
    cameraInfo.width = camera_data.width;
    cameraInfo.height = camera_data.height;

    cameraInfo.R[0] = cameraInfo.R[4] = cameraInfo.R[8] = 1;
    cameraInfo.header.frame_id = cname;
    cameraInfo.header.stamp = ros_impl::now(nh_);

    try {
        auto intrinsics = calibrationHandler.getCameraIntrinsics(socket, width, height);

        copy(cameraInfo.K, camera_data.intrinsicMatrix);
        ROS_IMPL_INFO(nh_, "Loading calibration for %s Fx %f Fy %f Cx %f Cy %f", cname.c_str(), cameraInfo.K[0],
                      cameraInfo.K[4],
                      cameraInfo.K[2], cameraInfo.K[5]);
        copy(cameraInfo.D, camera_data.distortionCoeff);
        cameraInfo.D.resize(8);
        copy43(cameraInfo.P, camera_data.intrinsicMatrix);
        assert(cname != "");

        if(socket == calibrationHandler.getStereoLeftCameraId()) {
            auto baseline_m = calibrationHandler.getBaselineDistance(calibrationHandler.getStereoRightCameraId(), calibrationHandler.getStereoLeftCameraId()) * .01;
            cameraInfo.P[3] = - cameraInfo.P[0] * baseline_m;
            copy(cameraInfo.R, saveData.stereoRectificationData.rectifiedRotationRight);
        } else if (socket == calibrationHandler.getStereoRightCameraId()) {
            copy(cameraInfo.R, saveData.stereoRectificationData.rectifiedRotationLeft);
        }
        transmit_eeprom(nh_, device, calibrationHandler);

        this->setCameraInfo(cameraInfo);
        return true;
    } catch(std::exception& e) {
        ROS_IMPL_WARN(nh_, "Could not load calibration: %s %s", cname.c_str(), e.what());
        this->setCameraInfo(cameraInfo);
        return true;
    }
}
