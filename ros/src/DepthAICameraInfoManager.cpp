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

static void copy(std::vector<std::vector<float>> &d, const double* s) {
    d.resize(3);
    for(int i = 0;i < 3;i++) {
        d[i].resize(3);
        for(int j = 0;j < 3;j++) {
            d[i][j] = s[j+i*3];
        }
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

std::shared_ptr<DepthaiCameraInfoManager> DepthaiCameraInfoManager::get(const std::shared_ptr<dai::Device> &device, enum dai::CameraBoardSocket socket) {
    std::string key = device->getMxId() + std::to_string((int)socket);
    return managers[key];
}



std::shared_ptr<DepthaiCameraInfoManager>
DepthaiCameraInfoManager::get(const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationHandler, dai::CameraBoardSocket socket, ros_impl::Node nh,
                              const std::string &cname, const std::string &url, int width, int height,
                              dai::Point2f topLeftPixelId,
                              dai::Point2f bottomRightPixelId) {
    std::string key = device->getMxId() + std::to_string((int)socket);
    if(managers.find(key) == managers.end() || managers[key] == 0) {
        managers[key] = ::std::shared_ptr<DepthaiCameraInfoManager>(new DepthaiCameraInfoManager(device, calibrationHandler, socket, nh, cname, url, width, height, topLeftPixelId, bottomRightPixelId));
    }
    return managers[key];
}

static tf2_ros::Buffer& buffer(ros_impl::Node nh) {
    static std::unique_ptr<tf2_ros::Buffer> buffer_ptr;
    static std::unique_ptr<tf2_ros::TransformListener> listener;

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

DepthaiCameraInfoManager::DepthaiCameraInfoManager(const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationHandler, dai::CameraBoardSocket socket,
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

static dai::CameraBoardSocket get_next_populated_socket(const std::shared_ptr<dai::Device> &device, dai::CameraBoardSocket socket) {
    auto next_socket = get_next_socket(socket);
    std::string key = device->getMxId() + std::to_string((int)next_socket);
    if(managers[key]) {
        return next_socket;
    }
    if(next_socket > dai::CameraBoardSocket::CAM_H)
        return dai::CameraBoardSocket::CAM_H;
    return get_next_populated_socket(device, next_socket);
}

std::map<std::string, ros_impl::geometry_msgs::TransformStamped> device_transforms;
static void transmit_eeprom(const ros_impl::Node& n, const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationData) {
    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

//        if(manager->Device().getMxId() != device.getMxId())
//            continue;
        auto this_socket = kv.second->Socket();
        auto this_name = kv.second->cname();
        if(this_socket == dai::CameraBoardSocket::CAM_A) {
            auto imu_frame = "dai_" + manager->Device().getMxId() + "_imu";
            try {
                auto extrinsics = calibrationData.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A, true);
                auto imu2refcam = cr::dai_tools::tx::daiExtrinsics2Tx(extrinsics, imu_frame, this_name);
                imu2refcam.header.stamp = ros_impl::now(n);
                device_transforms[imu2refcam.child_frame_id] = imu2refcam;
            } catch(std::exception& e) {
                ROS_IMPL_WARN(nh, "Cant find extrinsics for IMU and %d", (int)dai::CameraBoardSocket::CAM_A);
            }
        }

        auto next_socket = get_next_populated_socket(device, kv.second->Socket());
        auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket);
        if(!daiInfo)
            continue;
        auto next_name = daiInfo->cname();
        try {
            auto extrinsics = calibrationData.getCameraExtrinsics( next_socket, this_socket);
            auto pose_msg = cr::dai_tools::tx::daiExtrinsics2Tx(extrinsics, next_name, kv.second->cname());
            pose_msg.header.stamp = ros_impl::now(n);

            device_transforms[pose_msg.child_frame_id] = pose_msg;
            ROS_IMPL_INFO(n, "Broadcast tx between %s and %s", next_name.c_str(), kv.second->cname().c_str());
        } catch(std::runtime_error& e) {
            ROS_IMPL_WARN(n, "Could not broadcast tx between %s and %s, %s", next_name.c_str(), kv.second->cname().c_str(), e.what());
        }
    }
}

bool saveAllCalibrationData(const ros_impl::Node& n, const std::shared_ptr<dai::Device> &device, dai::CalibrationHandler& calibrationHandler) {
    auto now = std::chrono::high_resolution_clock::now();
    calibrationHandler.eepromToJsonFile("camera_info_backup_" + std::to_string(now.time_since_epoch().count()) + ".json");

    for(auto& kv : managers) {
        auto name = kv.first;
        auto& manager = kv.second;
        if(manager == 0) continue;

        if(manager->Device().getMxId() != device->getMxId())
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

        std::vector<std::vector<float>> R;
        copy(R, new_info.R.data());
        if(calibrationHandler.getStereoRightCameraId() == socket) {
            calibrationHandler.setStereoRight(socket, R);
        } else if(calibrationHandler.getStereoLeftCameraId() == socket) {
            calibrationHandler.setStereoLeft(socket, R);
        }

        std::vector<float> D;
        for (auto &d : new_info.D) D.push_back(d);
        D.resize(14);
        calibrationHandler.setDistortionCoefficients(socket, D);

        auto thisFrame = new_info.header.frame_id;
        auto imu_frame = "dai_" + manager->Device().getMxId() + "_imu";

        auto lookupTime = ros_impl::Time(0);
        //auto lookupTime = ros_impl::now(n);
        std::string error_msg;
        if(buffer(n).canTransform(imu_frame, thisFrame, lookupTime), &error_msg) {
            auto imu2cam = buffer(n).lookupTransform(thisFrame, imu_frame, lookupTime);
            tf2::Matrix3x3 rot;
            rot.setRotation(tf2::Quaternion(imu2cam.transform.rotation.x,
                                            imu2cam.transform.rotation.y,
                                            imu2cam.transform.rotation.z,
                                            imu2cam.transform.rotation.w));
            std::vector<std::vector<float>> dai_rot_imu2cam;
            std::vector<float> translation = {(float)imu2cam.transform.translation.x * 100.f,
                                              (float)imu2cam.transform.translation.y * 100.f,
                                              (float)imu2cam.transform.translation.z * 100.f};

            ROS_IMPL_INFO(n, "Saving extrinsics between %s and %s [%f %f %f] [%f %f %f %f]", cname.c_str(), imu_frame.c_str(),
                          translation[0], translation[1], translation[2], imu2cam.transform.rotation.x,
                          imu2cam.transform.rotation.y, imu2cam.transform.rotation.z, imu2cam.transform.rotation.w);
            copy(dai_rot_imu2cam, rot);
            calibrationHandler.setImuExtrinsics(socket, dai_rot_imu2cam, translation, translation);
        } else {
            ROS_IMPL_WARN(n, "Could not find transform between %s and %s (%s)", imu_frame.c_str(), thisFrame.c_str(), error_msg.c_str());
            return false;
        }


        if(auto daiInfo = DepthaiCameraInfoManager::get(device, next_socket)) {
            auto ci = daiInfo->getCameraInfo();
            auto thisFrame = new_info.header.frame_id;
            auto nextFrame = ci.header.frame_id;
            assert(thisFrame != "" && nextFrame != "");

            auto lookupTime = ros_impl::Time(0);

            /// NOTE: lookupTransform is target, source. setCameraExtrinsics is source, target.
            if(buffer(n).canTransform(nextFrame, thisFrame, lookupTime), &error_msg) {
                auto tx = buffer(n).lookupTransform(nextFrame, thisFrame, lookupTime);
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
        } else {
            std::vector<float> translation = {0,0,0};
            std::vector<std::vector<float>> rotationMatrix;
            rotationMatrix.resize(3);
            for(int i = 0;i < 3;i++) rotationMatrix[i].resize(3);
            calibrationHandler.setCameraExtrinsics(socket, dai::CameraBoardSocket::AUTO, rotationMatrix, translation);
        }

    }

    calibrationHandler.eepromToJsonFile("camera_info_" + std::to_string(now.time_since_epoch().count()) + ".json");
    try{
        //transmit_eeprom(n, device, calibrationHandler);
        return device->flashCalibration(calibrationHandler);
    } catch(std::runtime_error& e) {
        ROS_IMPL_WARN(n, "Error flashing calibration: %s", e.what());
        return false;
    }
}

static bool transmit_transforms = true;
void DepthaiCameraInfoManager::spin() {
    if(transmit_transforms) {
        for(auto& kv : device_transforms) {
            auto time = ros_impl::now(nh_);
            kv.second.header.stamp = ros_impl::Time().fromSec(time.toSec() + .15);
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

void DepthaiCameraInfoManager::setSize(int width, int height) {
    if(width > 0) this->width = width;
    if(height > 0) this->height = height;
    loadCalibrationFlash("", this->cname());
}
bool DepthaiCameraInfoManager::loadCalibrationFlash(const std::string &flashURL, const std::string &cname) {
    auto saveData = calibrationHandler.getEepromData();
    auto camera_data = saveData.cameraData[socket];

    ros_impl::sensor_msgs::CameraInfo cameraInfo = {};
    cameraInfo.distortion_model = "rational_polynomial";

    if(width == -1) width = camera_data.width;
    if(height == -1) height = camera_data.height;

    cameraInfo.width = width;
    cameraInfo.height = height;

    cameraInfo.R[0] = cameraInfo.R[4] = cameraInfo.R[8] = 1;
    cameraInfo.header.frame_id = cname;
    cameraInfo.header.stamp = ros_impl::now(nh_);

    try {
        auto intrinsics = calibrationHandler.getCameraIntrinsics(socket, width, height);

        copy(cameraInfo.K, intrinsics);
        ROS_IMPL_INFO(nh_, "Loading calibration for %s Fx %f Fy %f Cx %f Cy %f", cname.c_str(), cameraInfo.K[0],
                      cameraInfo.K[4],
                      cameraInfo.K[2], cameraInfo.K[5]);
        copy(cameraInfo.D, camera_data.distortionCoeff);
        cameraInfo.D.resize(8);
        copy43(cameraInfo.P, intrinsics);
        assert(cname != "");

        if(socket == calibrationHandler.getStereoLeftCameraId()) {
            auto baseline_m = calibrationHandler.getBaselineDistance(calibrationHandler.getStereoRightCameraId(), calibrationHandler.getStereoLeftCameraId()) * .01;
            //cameraInfo.P[3] = - cameraInfo.P[0] * baseline_m;
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

ros_impl::sensor_msgs::CameraInfo DepthaiCameraInfoManager::getCameraInfo(bool isRectified) {
    auto info = this->shim::camera_info_manager::CameraInfoManager::getCameraInfo();
    if(isRectified) {
        for (auto &d: info.D) {
            d = 0;
        }
    }
    return info;
}

ros_impl::sensor_msgs::CameraInfo DepthaiCameraInfoManager::getCameraInfo(void) {
    return CameraInfoManager::getCameraInfo();
}
