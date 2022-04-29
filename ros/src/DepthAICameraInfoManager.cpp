#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include "cr/dai-tools/DepthAICameraInfoManager.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

static void copy(std::vector<std::vector<float>> &d, const tf::Matrix3x3& s) {
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

static tf2_ros::Buffer& tfBuffer() {
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener tfListener(buffer);
    return buffer;
}

static tf2_ros::TransformBroadcaster& tfBroadcaster() {
    static tf2_ros::TransformBroadcaster broadcaster;
    return broadcaster;
}

std::shared_ptr<DepthaiCameraInfoManager> DepthaiCameraInfoManager::get(dai::Device &device, enum dai::CameraBoardSocket socket) {
    std::string key = device.getMxId() + std::to_string((int)socket);
    return managers[key];
}

std::shared_ptr<DepthaiCameraInfoManager>
DepthaiCameraInfoManager::get(dai::Device &device, dai::CameraBoardSocket socket, ros::NodeHandle nh,
                              const std::string &cname, const std::string &url) {
    tfBuffer();
    std::string key = device.getMxId() + std::to_string((int)socket);
    if(managers.find(key) == managers.end()) {
        managers[key] = std::shared_ptr<DepthaiCameraInfoManager>(new DepthaiCameraInfoManager(device, socket, nh, cname, url));
    }
    return managers[key];
}

DepthaiCameraInfoManager::DepthaiCameraInfoManager(dai::Device &device, dai::CameraBoardSocket socket,
                                                   ros::NodeHandle nh, const std::string &cname, const std::string &url)
        : shim::camera_info_manager::CameraInfoManager(nh, cname, url),
          device(device), socket(socket) {
    ROS_INFO("Creating DAI camera info manager at %s/%s", nh.getNamespace().c_str(), cname.c_str());
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

std::map<std::string, geometry_msgs::TransformStamped> device_transforms;
static void transmit_eeprom(dai::Device &device, dai::CalibrationHandler& calibrationData) {
    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

        if(manager->device.getMxId() != device.getMxId())
            continue;

        auto next_socket = get_next_populated_socket(device, kv.second->socket);
        auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket);
        if(!daiInfo)
            continue;
        auto cname = daiInfo->cname();
        try {
            auto extrinsics = calibrationData.getCameraExtrinsics( next_socket, kv.second->socket);
            //auto extrinsics = calibrationData.getCameraExtrinsics( kv.second->socket, next_socket);
            geometry_msgs::TransformStamped pose_msg = {};
            pose_msg.header.seq = 1;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.frame_id = cname;
            pose_msg.child_frame_id = kv.second->cname();

            pose_msg.transform.translation.x = extrinsics[0][3] * .01;
            pose_msg.transform.translation.y = extrinsics[1][3] * .01;
            pose_msg.transform.translation.z = extrinsics[2][3] * .01;
            tf::Matrix3x3 m2;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    m2[i][j] = extrinsics[i][j];
                }
            }
            tf::Quaternion q;
            m2.getRotation(q);
            pose_msg.transform.rotation.x = q.x();
            pose_msg.transform.rotation.y = q.y();
            pose_msg.transform.rotation.z = q.z();
            pose_msg.transform.rotation.w = q.w();

            device_transforms[pose_msg.child_frame_id] = pose_msg;
            //tfBroadcaster().sendTransform(pose_msg);
            ROS_INFO("Broadcast tx between %s and %s", cname.c_str(), kv.second->cname().c_str());
        } catch(std::runtime_error& e) {
            ROS_WARN("Could not broadcast tx between %s and %s, %s", cname.c_str(), kv.second->cname().c_str(), e.what());
        }
    }
}

bool saveAllCalibrationData(dai::Device &device) {
    auto calibration = device.readCalibration();
    auto calibrationData = calibration.getEepromData();

    auto now = std::chrono::high_resolution_clock::now();
    calibration.eepromToJsonFile("camera_info_backup_" + std::to_string(now.time_since_epoch().count()) + ".json");

    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

        if(manager->device.getMxId() != device.getMxId())
            continue;

        auto socket = manager->socket;
        auto cname = manager->getCameraInfo().header.frame_id;
        auto new_info = manager->getCameraInfo();
        auto next_socket = get_next_populated_socket(device, socket);
        ROS_INFO("Saving calibration for %s Fx %f Fy %f Cx %f Cy %f %d %d", cname.c_str(), new_info.K[0], new_info.K[4], new_info.K[2], new_info.K[5], (int)socket, (int)next_socket);

        std::vector<std::vector<float>> intrinsics;
        intrinsics.resize(3);
        for (int i = 0; i < 3; i++) {
            intrinsics[i].resize(3);
            for (int j = 0; j < 3; j++) {
                intrinsics[i][j] = new_info.K[i * 3 + j];
            }
        }
        calibration.setCameraIntrinsics(socket, intrinsics, new_info.width, new_info.height);
        std::vector<float> D;
        for (auto &d : new_info.D) D.push_back(d);
        D.resize(14);
        calibration.setDistortionCoefficients(socket, D);

        if(auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket)) {
            auto ci = daiInfo->getCameraInfo();
            auto thisFrame = new_info.header.frame_id;
            auto nextFrame = ci.header.frame_id;
            assert(thisFrame != "" && nextFrame != "");
            std::string error_msg;

            /// NOTE: lookupTransform is target, source. setCameraExtrinsics is source, target.
            if(tfBuffer().canTransform(nextFrame, thisFrame, ros::Time(0), &error_msg)) {
                //auto tx = tfBuffer().lookupTransform(nextFrame, thisFrame, ros::Time(0));
                auto tx = tfBuffer().lookupTransform(thisFrame, nextFrame, ros::Time(0));
                tf::Matrix3x3 rot;
                rot.setRotation(tf::Quaternion(tx.transform.rotation.x,
                                               tx.transform.rotation.y,
                                               tx.transform.rotation.z,
                                               tx.transform.rotation.w));
                std::vector<std::vector<float>> dai_rot;
                std::vector<float> translation = { (float)tx.transform.translation.x * 100.f,
                                                   (float)tx.transform.translation.y * 100.f,
                                                   (float)tx.transform.translation.z * 100.f};

                ROS_INFO("Saving extrinsics between %s and %s [%f %f %f] [%f %f %f %f]", cname.c_str(), nextFrame.c_str(),
                         translation[0], translation[1], translation[2], tx.transform.rotation.x,
                         tx.transform.rotation.y, tx.transform.rotation.z, tx.transform.rotation.w);
                copy(dai_rot, rot);
                calibration.setCameraExtrinsics(socket, static_cast<dai::CameraBoardSocket>(next_socket), dai_rot, translation, translation);
            } else {
                ROS_WARN("%s", error_msg.c_str());
                return false;
            }
        }

    }

    calibration.eepromToJsonFile("camera_info_" + std::to_string(now.time_since_epoch().count()) + ".json");
    try{
        transmit_eeprom(device, calibration);
        return device.flashCalibration(calibration);
    } catch(std::runtime_error& e) {
        ROS_WARN("%s", e.what());
        return false;
    }
}

void DepthaiCameraInfoManager::spin() {
    static double last = 0;
    auto now = ros::Time::now().toSec();
    if(now - last < .1) {
        return;
    }
    last = now;
    ros::V_string nodes;
    ros::master::getNodes(nodes);
    bool has_broadcast_node = false;
    for(auto& node : nodes) {
        if(node == "/broadcast_tf_graph") {
            has_broadcast_node = true;
        }
    }

    if(!has_broadcast_node) {
        for(auto& kv : device_transforms) {
            kv.second.header.stamp = ros::Time::now();
            tfBroadcaster().sendTransform(kv.second);
        }
    }
}

bool
DepthaiCameraInfoManager::saveCalibrationFlash(const sensor_msgs::CameraInfo &_new_info, const std::string &flashURL,
                                               const std::string &cname) {
    auto calibration = device.readCalibration();
    auto calibrationData = calibration.getEepromData();

    auto now = std::chrono::high_resolution_clock::now();
    calibration.eepromToJsonFile("camera_info_backup_" + std::to_string(now.time_since_epoch().count()) + ".json");

    sensor_msgs::CameraInfo new_info = _new_info;
    new_info.header.frame_id = cname;
    this->cam_info_ = new_info;

    return saveAllCalibrationData(device);
}

bool DepthaiCameraInfoManager::loadCalibrationFlash(const std::string &flashURL, const std::string &cname) {
    auto calibrationData = device.readCalibration();
    auto saveData = calibrationData.getEepromData();
    auto camera_data = saveData.cameraData[socket];

    sensor_msgs::CameraInfo cameraInfo = { };
    cameraInfo.distortion_model = "rational_polynomial";
    cameraInfo.width = camera_data.width;
    copy(cameraInfo.K, camera_data.intrinsicMatrix);
    ROS_INFO("Loading calibration for %s Fx %f Fy %f Cx %f Cy %f", cname.c_str(), cameraInfo.K[0], cameraInfo.K[4], cameraInfo.K[2], cameraInfo.K[5]);
    copy(cameraInfo.D, camera_data.distortionCoeff);
    cameraInfo.D.resize(8);

    copy43(cameraInfo.P, camera_data.intrinsicMatrix);
    cameraInfo.R[0] = cameraInfo.R[4] = cameraInfo.R[8] = 1;
    cameraInfo.height = camera_data.height;
    if(cameraInfo.height == 0) cameraInfo.height = 172;
    if(cameraInfo.width == 0) cameraInfo.width = 224;
    assert(cname != "");
    cameraInfo.header.frame_id = cname;
    cameraInfo.header.stamp = ros::Time::now();
    cameraInfo.header.seq = 1;

    transmit_eeprom(device, calibrationData);

    this->setCameraInfo(cameraInfo);
    return true;
}
