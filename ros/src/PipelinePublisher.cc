#include "cr/dai-tools/PipelinePublisher.h"
#include "cr/dai-tools/NodeWalker.h"
#include "depthai/pipeline/Pipeline.hpp"

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "cr_dai_ros/StereoDepthConfig.h"
#include "cr_dai_ros/CameraControlConfig.h"
#include "cr_dai_ros/DeviceControlConfig.h"

#include "cr/dai-tools/ImagePublisher.h"
#include "cr/dai-tools/ToFDepthPublisher.h"
#include "cr/dai-tools/IMUPublisher.h"
#include "cr/dai-tools/NNPublisher.h"

#include <utility>
namespace cr {
    namespace dai_rosnode {

        static std::map<dai::CameraBoardSocket, std::string> default_frame_mapping() {
            std::map<dai::CameraBoardSocket, std::string> frameNames;
            for(int i = (int)dai::CameraBoardSocket::CAM_A; i < 8; i++) {
                auto name = std::string("CAM_") + (char)(i + 'A');
                frameNames[(dai::CameraBoardSocket)i] = name;
            }
//            frameNames[dai::CameraBoardSocket::LEFT] = "left_camera_optical_frame";
//            frameNames[dai::CameraBoardSocket::RIGHT] = "right_camera_optical_frame";
//            frameNames[dai::CameraBoardSocket::RGB] = "rgb_camera_optical_frame";
            return frameNames;
        }

        static sensor_msgs::CameraInfo calibrationToCameraInfo(
                dai::CalibrationHandler calibHandler, dai::CameraBoardSocket cameraId, int width, int height, dai::Point2f topLeftPixelId = {}, dai::Point2f bottomRightPixelId = {}) {
            std::vector<std::vector<float>> camIntrinsics, rectifiedRotation;
            std::vector<float> distCoeffs;
            std::vector<double> flatIntrinsics, distCoeffsDouble;
            int defWidth, defHeight;
            sensor_msgs::CameraInfo cameraData = {};

            try {
                std::tie(std::ignore, defWidth, defHeight) = calibHandler.getDefaultIntrinsics(cameraId);

                if(width == -1) {
                    cameraData.width = static_cast<uint32_t>(defWidth);
                } else {
                    cameraData.width = static_cast<uint32_t>(width);
                }

                if(height == -1) {
                    cameraData.height = static_cast<uint32_t>(defHeight);
                } else {
                    cameraData.height = static_cast<uint32_t>(height);
                }

                camIntrinsics = calibHandler.getCameraIntrinsics(cameraId, cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);

                flatIntrinsics.resize(9);
                for(int i = 0; i < 3; i++) {
                    std::copy(camIntrinsics[i].begin(), camIntrinsics[i].end(), flatIntrinsics.begin() + 3 * i);
                }
            } catch(std::exception& exception) {
                ROS_WARN("Failed to get cameraInfo for socket %d. Error: %s", (int)cameraId, exception.what());
                return cameraData;
            }

            auto& intrinsics = cameraData.K;
            auto& distortions = cameraData.D;
            auto& projection = cameraData.P;
            auto& rotation = cameraData.R;

            std::copy(flatIntrinsics.begin(), flatIntrinsics.end(), intrinsics.begin());

            distCoeffs = calibHandler.getDistortionCoefficients(cameraId);

            for(size_t i = 0; i < 8; i++) {
                distortions.push_back(static_cast<double>(distCoeffs[i]));
            }

            // Setting Projection matrix if the cameras are stereo pair. Right as the first and left as the second.
            if(calibHandler.getStereoRightCameraId() != dai::CameraBoardSocket::AUTO && calibHandler.getStereoLeftCameraId() != dai::CameraBoardSocket::AUTO) {
                if(calibHandler.getStereoRightCameraId() == cameraId || calibHandler.getStereoLeftCameraId() == cameraId) {
                    std::vector<std::vector<float>> stereoIntrinsics = calibHandler.getCameraIntrinsics(
                            calibHandler.getStereoRightCameraId(), cameraData.width, cameraData.height, topLeftPixelId, bottomRightPixelId);
                    std::vector<double> stereoFlatIntrinsics(12), flatRectifiedRotation(9);
                    for(int i = 0; i < 3; i++) {
                        std::copy(stereoIntrinsics[i].begin(), stereoIntrinsics[i].end(), stereoFlatIntrinsics.begin() + 4 * i);
                        stereoFlatIntrinsics[(4 * i) + 3] = 0;
                    }

                    if(calibHandler.getStereoLeftCameraId() == cameraId) {
                        // This defines where the first camera is w.r.t second camera coordinate system giving it a translation to place all the points in the first
                        // camera to second camera by multiplying that translation vector using transformation function.
                        stereoFlatIntrinsics[3] = stereoFlatIntrinsics[0]
                                                  * calibHandler.getCameraExtrinsics(calibHandler.getStereoRightCameraId(), calibHandler.getStereoLeftCameraId())[0][3]
                                                  / 100.0;  // Converting to meters
                        rectifiedRotation = calibHandler.getStereoLeftRectificationRotation();
                    } else {
                        rectifiedRotation = calibHandler.getStereoRightRectificationRotation();
                    }

                    for(int i = 0; i < 3; i++) {
                        std::copy(rectifiedRotation[i].begin(), rectifiedRotation[i].end(), flatRectifiedRotation.begin() + 3 * i);
                    }

                    std::copy(stereoFlatIntrinsics.begin(), stereoFlatIntrinsics.end(), projection.begin());
                    std::copy(flatRectifiedRotation.begin(), flatRectifiedRotation.end(), rotation.begin());
                }
            }
            cameraData.distortion_model = "rational_polynomial";
            cameraData.header.frame_id = default_frame_mapping()[cameraId];

            return cameraData;
        }

        struct SetupPostStart : public cr::dai_tools::NodeWalker {
            PipelinePublisher* self = 0;
            SetupPostStart(PipelinePublisher* self) : self(self) {}
            bool Visit(std::shared_ptr <dai::node::MonoCamera> mono) override {
                self->setupCameraControlServer(mono, "mono");
                return true;
            }
            bool Visit(std::shared_ptr <dai::node::ColorCamera> rgb) override {
                self->setupCameraControlServer(rgb, "rgb");
                return true;
            }
            bool Visit(std::shared_ptr <dai::node::StereoDepth> stereo) override {
                self->setupCameraControlServer(stereo, "stereo");
                return true;
            }
        };

        PipelinePublisher::PipelinePublisher(::ros::NodeHandle& pnh,
                                             std::shared_ptr<dai::Device> device,
                                             dai::Pipeline& pipeline)
                : _pnh(pnh), _device(*device), metaInfo(device) {
            std::cerr << "Reading calibration data..." << std::endl;
            _calibrationHandler = device->readCalibration();
            BuildPublisherFromPipeline(pipeline);
        }

        void PipelinePublisher::setupCameraControlServer(std::shared_ptr<dai::node::StereoDepth> stereo, const std::string& prefix) {
            auto configQueue = _device.getInputQueue(prefix + "Config");
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::StereoDepthConfig>>(_pnh);
            cr_dai_ros::StereoDepthConfig def_config = { };
            def_config.left_right_check = stereo->initialConfig.getLeftRightCheckThreshold() > 0; // No getter for this; just check threshold??
            def_config.confidence = stereo->initialConfig.getConfidenceThreshold();
            def_config.bilateral_sigma = stereo->initialConfig.getBilateralFilterSigma();
            def_config.extended_disparity = stereo->initialConfig.get().algorithmControl.enableExtended;
            def_config.subpixel = stereo->initialConfig.get().algorithmControl.enableSubpixel;
            def_config.lr_check_threshold = stereo->initialConfig.getLeftRightCheckThreshold();
            server->setConfigDefault(def_config);

            server->setCallback([configQueue, stereo](cr_dai_ros::StereoDepthConfig& cfg, unsigned level) {
                dai::StereoDepthConfig dcfg = stereo->initialConfig;
                auto rawCfg = dcfg.get();
                rawCfg.postProcessing.thresholdFilter.maxRange = cfg.threshold_max;
                rawCfg.postProcessing.thresholdFilter.minRange = cfg.threshold_min;
                dcfg.set(rawCfg);

                dcfg.setConfidenceThreshold(cfg.confidence);
                dcfg.setLeftRightCheckThreshold(cfg.lr_check_threshold);
                dcfg.setBilateralFilterSigma(cfg.bilateral_sigma);
                dcfg.setSubpixel(cfg.subpixel);
                dcfg.setLeftRightCheck(cfg.left_right_check);
                dcfg.setExtendedDisparity(cfg.extended_disparity);

                configQueue->send(dcfg);
            });
            keep_alive.push_back(server);
        }

        void PipelinePublisher::BuildPublisherFromPipeline(dai::Pipeline& pipeline) {
            setupDeviceServer();

            std::cerr << "Making prepass " << _pnh.getNamespace() << std::endl;
            if(!_device.isPipelineRunning()) {
                for(auto& node : pipeline.getAllNodes()) {
                    addConfigNodes(pipeline, node);
                }

                std::cerr << "Starting pipeline" << std::endl;
                _device.startPipeline(pipeline);

                std::cerr << "Doing post-startup pass" << std::endl;
                SetupPostStart walker(this);
                walker.VisitAll(pipeline.getAllNodes());
            } else {
                ROS_WARN("Device is running already, PipelinePublisher can not add configuration servers");
            }

            std::cerr << "Mapping connections..." << std::endl;
            auto connections = pipeline.getConnectionMap();
            for(auto& connection : connections) {
                auto node = pipeline.getNode(connection.first);
                if(auto xlinkOut = std::dynamic_pointer_cast<dai::node::XLinkOut>(node)) {
                    for(auto& nodeConnection : connection.second) {
                        auto otherNode = pipeline.getNode(nodeConnection.outputId);
                        StartVisit(SetupPublishers(), xlinkOut, nodeConnection.outputName, otherNode);
                    }
                }
            }
        }


        template<typename T> void PipelinePublisher::setupCameraControlQueue(std::shared_ptr<T> cam, const std::string& prefix) {
            auto configIn = cam->getParentPipeline().template create<dai::node::XLinkIn>();
            auto name = prefix + std::to_string((int)cam->getBoardSocket());
            std::cerr << "Setting up camera control xlinks for " << name << std::endl;
            configIn->setStreamName(name + "_inputControl");
            configIn->out.link(cam->inputControl);
        }

        void PipelinePublisher::setupCameraControlServer(dai::CameraBoardSocket socket, const std::string& prefix) {
            auto name = prefix + std::to_string((int)socket);
            ROS_INFO("Setting up camera control server for %s", name.c_str());
            auto configQueue = _device.getInputQueue(name + "_inputControl");
            auto n = getNodeHandle(socket);
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::CameraControlConfig>>(n);

            auto current_config = std::make_shared<cr_dai_ros::CameraControlConfig>();
            server->getConfigDefault(*current_config);
            keep_alive.push_back(current_config);

            auto triggger_update = [=](cr_dai_ros::CameraControlConfig& cfg, unsigned level) {
                dai::CameraControl dcfg;
                if(level == 0xffffffff || level & 7) dcfg.setStartStreaming();
                if(level & 1) dcfg.setAutoFocusMode(static_cast<dai::CameraControl::AutoFocusMode>(cfg.autofocus_mode));
                if(level & 2) dcfg.setAutoFocusRegion(cfg.autofocus_startx, cfg.autofocus_starty, cfg.autofocus_width, cfg.autofocus_height);
                if(level & 2) dcfg.setAutoFocusLensRange(cfg.autofocus_min, cfg.autofocus_max);
                if(level & 4) dcfg.setManualFocus(cfg.manual_focus);
                if(level & 8) {
                    dcfg.setAutoExposureLock(cfg.autoexposure_lock);
                }

                if(level & 16) dcfg.setAutoExposureRegion(cfg.autoexposure_startx, cfg.autoexposure_starty, cfg.autoexposure_width, cfg.autoexposure_height);

                if(level & 32) dcfg.setAutoExposureCompensation(cfg.autoexposure_compensation);
                if(level & 64) dcfg.setContrast(cfg.contrast);
                if(level & 128) dcfg.setBrightness(cfg.brightness);
                if(level & 256) dcfg.setSaturation(cfg.saturation);
                if(level & 512) dcfg.setSharpness(cfg.sharpness);
                if(level & 1024) dcfg.setChromaDenoise(cfg.chroma_denoise);
                if(level & 2048) {
                    if(cfg.manual_exposure < 0 || cfg.manual_iso < 0) {
                        dcfg.setAutoExposureEnable();
                    } else {
                        dcfg.setManualExposure(cfg.manual_exposure, cfg.manual_iso < 100 ? 100 : cfg.manual_iso);
                    }
                }
                *current_config = cfg;
                configQueue->send(dcfg);
            };

            server->setCallback([=](cr_dai_ros::CameraControlConfig& cfg, unsigned level) {
                triggger_update(cfg, level);
            });

            keep_alive.push_back(server);
        }
        template <typename T>
        void PipelinePublisher::setupCameraControlServer(std::shared_ptr<T> cam, const std::string& prefix) {
            setupCameraControlServer(cam->getBoardSocket(), prefix);
        }

        void PipelinePublisher::addConfigNodes(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node) {
            if(auto stereo = std::dynamic_pointer_cast<dai::node::StereoDepth>(node)) {
                std::cerr << "Setting up camera control xlinks for stereo" << std::endl;
                auto configIn = pipeline.create<dai::node::XLinkIn>();
                configIn->setStreamName("stereoConfig");
                configIn->out.link(stereo->inputConfig);
            } else if(auto rgb = std::dynamic_pointer_cast<dai::node::ColorCamera>(node)) {
                setupCameraControlQueue(rgb, "rgb");
            }
            else if(auto mono = std::dynamic_pointer_cast<dai::node::MonoCamera>(node)) {
                setupCameraControlQueue(mono, "mono");
            }
        }
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::NeuralNetwork> ptr) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);
            make_publisher<NNPublisher>(
                    queue,
                    _pnh,
                    30,
                    xLinkOut);
            return true;

        }
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::IMU> inputNode) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);
            make_publisher<IMUPublisher>(
                    queue,
                    _pnh,
                    30,
                    xLinkOut);
            return true;
        }

        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::Camera> inputNode) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);

            sensor_msgs::CameraInfo rawCameraInfo;
            make_publisher<ImagePublisher>(
                    queue,
                    getNodeHandle(inputNode->getBoardSocket()),
                    30,
                    rawCameraInfo,
                    xLinkOut);

            return true;
        }

        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::ToF> inputNode) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);

            int width = 224, height = 172;

            auto socket = dai::CameraBoardSocket::RGB;
            auto cameraInfo = calibrationToCameraInfo(_calibrationHandler, socket, width, height);
            if(inputName == "out") {
                make_publisher<ToFDepthPublisher>(
                        queue,
                        getNodeHandle(socket),
                        30,
                        cameraInfo, xLinkOut);
            } else {
                make_publisher<ImagePublisher>(
                        queue,
                        getNodeHandle(socket),
                        30,
                        cameraInfo, xLinkOut);
            }
            return true;
        }

        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                                      const std::string &inputName, std::shared_ptr<dai::node::MonoCamera> inputNode) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);

            int width = 1280, height = 720;
            width = inputNode->getResolutionWidth();
            height = inputNode->getResolutionHeight();

            auto cameraInfo = calibrationToCameraInfo(_calibrationHandler, inputNode->getBoardSocket(), width, height);
            auto publisher = make_publisher<ImagePublisher>(
                    queue,
                    getNodeHandle(inputNode->getBoardSocket()),
                    30,
                    cameraInfo, xLinkOut);
            return true;
        }
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                                      const std::string &inputName, std::shared_ptr<dai::node::ColorCamera> inputNode) {
            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);

            int width = 1280, height = 720;
            if(inputName == "video") {
                width = inputNode->getVideoWidth();
                height = inputNode->getVideoHeight();
            } else if(inputName == "still") {
                width = inputNode->getStillWidth();
                height = inputNode->getStillHeight();
            } else if(inputName == "preview") {
                width = inputNode->getPreviewWidth();
                height = inputNode->getPreviewHeight();
            } else if(inputName == "isp") {
                height = inputNode->getIspHeight();
                width = inputNode->getIspWidth();
            } else {
                ROS_WARN("Don't understand output named %s in ColorCamera. Using default image size for intrinsics", inputName.c_str());
            }

            auto rgbCameraInfo = calibrationToCameraInfo(_calibrationHandler, inputNode->getBoardSocket(), width, height);
            make_publisher<ImagePublisher>(
                    queue,
                    getNodeHandle(inputNode->getBoardSocket()),
                    30,
                    rgbCameraInfo,
                    xLinkOut);

            return true;
        }

        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                                      const std::string &inputName,
                                      std::shared_ptr<dai::node::StereoDepth> stereo) {

            auto queue = _device.getOutputQueue(xLinkOut->getStreamName(), 30, false);
            auto alignSocket = stereo->properties.depthAlignCamera;
            if(alignSocket == dai::CameraBoardSocket::AUTO) {
                alignSocket = dai::CameraBoardSocket::RIGHT;
            }

            auto depthCameraInfo = calibrationToCameraInfo(_calibrationHandler, alignSocket, 1280, 720);
            if(inputName == "depth") {
                //converter = std::make_shared<ImageConverter>(_frame_prefix + frame, true);
                make_publisher<ImagePublisher>(
                        queue,
                        _pnh,
                        30,
                        depthCameraInfo,
                        xLinkOut);
            } else if(inputName == "disparity") {
//                auto converter =
//                        std::make_shared<dai::rosBridge::DisparityConverter>(_frame_prefix + frame, 880, 7.5, 20,
//                                                                             2000);  // TODO(sachin): undo hardcoding of baseline
//                publisher = std::make_shared<dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame>>(
//                        queue,
//                        _pnh,
//                        std::string("stereo/disparity"),
//                        std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, converter.get(), std::placeholders::_1,
//                                  std::placeholders::_2),
//                        30,
//                        depthCameraInfo,
//                        "stereo");
//                keep_alive.push_back(converter);
            } else if(inputName == "confidenceMap") {
                //converter = std::make_shared<ImageConverter>(_frame_prefix + frame, true);
                make_publisher<ImagePublisher>(
                        queue,
                        _pnh,
                        30,
                        depthCameraInfo,
                        xLinkOut);
            } else if(inputName == "rectifiedLeft" || inputName == "rectifiedRight" || inputName == "syncedRight" || inputName == "syncedLeft") {
                std::string side_name = (inputName == "rectifiedLeft" || inputName == "syncedLeft") ? "left" : "right";
                auto socket = side_name == "left" ? dai::CameraBoardSocket::LEFT : dai::CameraBoardSocket::RIGHT;
                std::string pub_name = side_name + ((inputName == "rectifiedLeft" || inputName == "rectifiedRight") ? "/image_rect" : "/image_raw");

                int monoWidth = 0, monoHeight = 0;
                auto pipeline = stereo->getParentPipeline();
                auto connections = pipeline.getConnectionMap();
                auto stereoConnections = connections[stereo->id];
                std::shared_ptr<dai::node::MonoCamera> monoNode;
                for(auto& connection : stereoConnections) {
                    if(connection.inputName == side_name) {
                        monoNode = std::dynamic_pointer_cast<dai::node::MonoCamera>(pipeline.getNode(connection.outputId));
                    }
                }

                if(!monoNode) {
                    ROS_WARN("Could not get input source for %s on stereo node", side_name.c_str());
                    return true;
                }

                auto cameraInfo = calibrationToCameraInfo(
                        _calibrationHandler, monoNode->getBoardSocket(), monoNode->getResolutionWidth(), monoNode->getResolutionHeight());
                make_publisher<ImagePublisher>(
                        queue,
                        _pnh,
                        30,
                        cameraInfo,
                        xLinkOut);
            } else {
                ROS_WARN("Don't understand output named %s in StereoDepth", inputName.c_str());
            }

            return true;
        }

        void PipelinePublisher::setupDeviceServer() {
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::DeviceControlConfig>>(_pnh);
            std::cerr << "Setting up device server at " << _pnh.getNamespace() << std::endl;
            auto current_config = std::make_shared<cr_dai_ros::DeviceControlConfig>();
            server->getConfigDefault(*current_config);
            keep_alive.push_back(current_config);

            auto triggger_update = [=](cr_dai_ros::DeviceControlConfig& cfg, unsigned level) {
                if(level & 1) _device.setIrFloodLightBrightness(cfg.IrFloodLightBrightness_right, 0x2);
                if(level & 2) _device.setIrFloodLightBrightness(cfg.IrFloodLightBrightness_left, 0x1);
                if(level & 4) _device.setIrLaserDotProjectorBrightness(cfg.IrLaserDotProjectorBrightness_right, 0x2);
                if(level & 8) _device.setIrLaserDotProjectorBrightness(cfg.IrLaserDotProjectorBrightness_left, 0x1);
                if(level & 16) {
                    _device.setLogLevel(static_cast<dai::LogLevel>(cfg.LogLevel));
                    _device.setLogOutputLevel(static_cast<dai::LogLevel>(cfg.LogLevel));
                }

            };

            server->setCallback([=](cr_dai_ros::DeviceControlConfig& cfg, unsigned level) {
                triggger_update(cfg, level);
            });

            keep_alive.push_back(server);
        }

        ::ros::NodeHandle &PipelinePublisher::getNodeHandle(dai::CameraBoardSocket socket) {
            if(_nodeHandles.find(socket) == _nodeHandles.end()) {
                auto ns = default_frame_mapping()[socket];
                _nodeHandles[socket] = ::ros::NodeHandle(_pnh, ns);

                auto cname = metaInfo.Name + "_" + ns;
                auto  cameraInfoManager = std::make_shared<DepthaiCameraInfoManager>(_device, socket, _nodeHandles[socket], cname);
                keep_alive.push_back(cameraInfoManager);
            }
            return _nodeHandles[socket];
        }

    }
}