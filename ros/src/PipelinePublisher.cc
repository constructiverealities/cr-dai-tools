#include "cr/dai-tools/PipelinePublisher.h"
#include "cr/dai-tools/NodeWalker.h"
#include "depthai/pipeline/Pipeline.hpp"

#include "cr/dai-tools/ImagePublisher.h"
#include "cr/dai-tools/DepthPublisher.h"
#include "cr/dai-tools/ToFDepthPublisher.h"
#include "cr/dai-tools/IMUPublisher.h"
#include "cr/dai-tools/NNPublisher.h"
#include "cr/dai-tools/Tx.h"

#include <utility>
namespace cr {
    namespace dai_rosnode {

        static std::map<dai::CameraBoardSocket, std::string> default_frame_mapping() {
            std::map<dai::CameraBoardSocket, std::string> frameNames;
            for(int i = (int)dai::CameraBoardSocket::CAM_A; i < 8; i++) {
                auto name = std::string("CAM_") + (char)(i + 'A');
                frameNames[(dai::CameraBoardSocket)i] = name;
            }
            return frameNames;
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
#ifdef HAS_CR_FORK
            bool Visit(std::shared_ptr <dai::node::ToF> tof) override {
                self->setupCameraControlServer(tof, "tof");
                return true;
            }
#endif
        };

        PipelinePublisher::PipelinePublisher(::ros_impl::Node& pnh,
                                             std::shared_ptr<dai::Device> device,
                                             dai::Pipeline& pipeline)
                : _device_node(pnh), _device(device), metaInfo(device), mxId(device->getMxId()) {
            _calibrationHandler = device->readCalibration();
            BuildPublisherFromPipeline(pipeline);
        }

        void PipelinePublisher::setupCameraControlServer(std::shared_ptr<dai::node::StereoDepth> stereo, const std::string& prefix) {
#ifdef HAS_DYNAMIC_RECONFIGURE
            auto n = ros_impl::make_node(_device_node, "stereo");
            ROS_IMPL_INFO(_device_node, "Setting up stereo control server for %s", ros_impl::Namespace(n));
            auto configQueue = _device->getInputQueue(prefix + "Config");
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::StereoDepthConfig>>(*n);
            cr_dai_ros::StereoDepthConfig def_config = { };
            def_config.threshold_min = stereo->initialConfig.get().postProcessing.thresholdFilter.minRange;
            def_config.threshold_max = stereo->initialConfig.get().postProcessing.thresholdFilter.maxRange;
            def_config.left_right_check = stereo->initialConfig.getLeftRightCheckThreshold() > 0; // No getter for this; just check threshold??
            def_config.confidence = stereo->initialConfig.getConfidenceThreshold();
            def_config.bilateral_sigma = stereo->initialConfig.getBilateralFilterSigma();
            def_config.extended_disparity = stereo->initialConfig.get().algorithmControl.enableExtended;
            def_config.subpixel = stereo->initialConfig.get().algorithmControl.enableSubpixel;
            def_config.lr_check_threshold = stereo->initialConfig.getLeftRightCheckThreshold();
            def_config.median_filter_mode = static_cast<int>(stereo->initialConfig.getMedianFilter());
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
                dcfg.setMedianFilter(static_cast<dai::MedianFilter>(cfg.median_filter_mode));

                configQueue->send(dcfg);
            });
            keep_alive.push_back(server);
#endif
        }

        void PipelinePublisher::BuildPublisherFromPipeline(dai::Pipeline& pipeline) {
            setupDeviceServer();
            bool needsPipelineStart = !_device->isPipelineRunning();
            if(needsPipelineStart) {
                for(auto& node : pipeline.getAllNodes()) {
                    addConfigNodes(pipeline, node);
                }

                _device->startPipeline(pipeline);

                SetupPostStart walker(this);
                walker.VisitAll(pipeline.getAllNodes());
            } else {
                ROS_IMPL_WARN(_device_node, "Device is running already, PipelinePublisher can not add configuration servers");
            }

            auto connections = pipeline.getConnectionMap();
            for(auto& connection : connections) {
                auto node = pipeline.getNode(connection.first);
                if(auto xlinkOut = std::dynamic_pointer_cast<dai::node::XLinkOut>(node)) {
                    for(auto& nodeConnection : connection.second) {
                        auto otherNode = pipeline.getNode(nodeConnection.outputId);
                        bool handled = StartVisit(SetupPublishers(), xlinkOut, nodeConnection.outputName, otherNode);
                        if(!handled) {
                            ROS_IMPL_WARN(_device_node, "Could not map xlinkout named %s", nodeConnection.outputName.c_str());
                        }
                    }
                }
            }

            auto qs = _device->getOutputQueueNames();
            for(auto q : qs) {
                mappedQueues.erase(q);
            }
            for(auto& q : mappedQueues) {
                ROS_IMPL_WARN(_device_node, "Unmapped queue %s", q.c_str());
            }
        }

#ifdef HAS_CR_FORK
        void PipelinePublisher::setupCameraControlQueue(std::shared_ptr<dai::node::ToF> tof, const std::string& prefix) {
#ifdef HAS_DYNAMIC_RECONFIGURE
            auto configIn = tof->getParentPipeline().template create<dai::node::XLinkIn>();
            auto name = prefix;
            ROS_IMPL_INFO(_device_node,"Setting up camera control xlinks for %s", name.c_str());
            configIn->setStreamName(name + "_inputControl");
            configIn->out.link(tof->inputConfig);
#endif
        }

        void PipelinePublisher::setupCameraControlServer(std::shared_ptr<dai::node::ToF> cam, const std::string& prefix) {
#ifdef HAS_DYNAMIC_RECONFIGURE
            auto socket = dai::CameraBoardSocket::CAM_A;
            auto name = prefix;
            ROS_IMPL_INFO(_device_node, "Setting up ToF control server for %s", name.c_str());
            auto configQueue = _device->getInputQueue(name + "_inputControl");
            auto n = getNodeHandle(socket);
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::ToFControlConfig>>(*n);

            auto current_config = std::make_shared<cr_dai_ros::ToFControlConfig>();
            server->getConfigDefault(*current_config);
            keep_alive.push_back(current_config);

            auto triggger_update = [=](cr_dai_ros::ToFControlConfig& cfg, unsigned level) {
                dai::ToFConfig tofCfg;
                tofCfg.get().albedoCutoffAmplitude = cfg.albedo_cutoff_amp;
                tofCfg.get().maxAlbedo = cfg.max_albedo_param;
                tofCfg.get().minAlbedo = cfg.min_albedo_param;
                tofCfg.get().maxAsymmetry = cfg.max_asymmetry;
                tofCfg.get().maxDistance = cfg.max_dist_param;
                tofCfg.get().minDistance = cfg.min_dist_param;
                tofCfg.get().minAmplitude = cfg.min_amp_param;
                tofCfg.get().maxError = cfg.max_error_param;
                tofCfg.get().useLoadedFilter = cfg.use_loaded_filter;
                configQueue->send(tofCfg);
            };

            server->setCallback([=](cr_dai_ros::ToFControlConfig& cfg, unsigned level) {
                triggger_update(cfg, level);
            });

            keep_alive.push_back(server);
#endif
        }

#endif

        template<typename T> void PipelinePublisher::setupCameraControlQueue(std::shared_ptr<T> cam, const std::string& prefix) {
#ifdef HAS_DYNAMIC_RECONFIGURE
            auto configIn = cam->getParentPipeline().template create<dai::node::XLinkIn>();
            auto name = prefix + std::to_string((int)cam->getBoardSocket());
            ROS_IMPL_INFO(_device_node,"Setting up camera control xlinks for %s", name.c_str());
            configIn->setStreamName(name + "_inputControl");
            configIn->out.link(cam->inputControl);
#endif
        }

        void PipelinePublisher::setupCameraControlServer(dai::CameraBoardSocket socket, const std::string& prefix) {
#ifdef HAS_DYNAMIC_RECONFIGURE
            auto name = prefix + std::to_string((int)socket);
            ROS_IMPL_INFO(_device_node, "Setting up camera control server for %s", name.c_str());
            auto configQueue = _device->getInputQueue(name + "_inputControl");
            auto n = getNodeHandle(socket);
            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::CameraControlConfig>>(*n);

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
#endif
        }
        template <typename T>
        void PipelinePublisher::setupCameraControlServer(std::shared_ptr<T> cam, const std::string& prefix) {
            setupCameraControlServer(cam->getBoardSocket(), prefix);
        }

        void PipelinePublisher::addConfigNodes(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node) {
            if(auto stereo = std::dynamic_pointer_cast<dai::node::StereoDepth>(node)) {
#ifdef HAS_DYNAMIC_RECONFIGURE
                ROS_IMPL_INFO(_device_node, "Setting up camera control xlinks for stereo");
                auto configIn = pipeline.create<dai::node::XLinkIn>();
                configIn->setStreamName("stereoConfig");
                configIn->out.link(stereo->inputConfig);
#endif
            } else if(auto rgb = std::dynamic_pointer_cast<dai::node::ColorCamera>(node)) {
                setupCameraControlQueue(rgb, "rgb");
            }
            else if(auto mono = std::dynamic_pointer_cast<dai::node::MonoCamera>(node)) {
                setupCameraControlQueue(mono, "mono");
            }
#ifdef HAS_CR_FORK
            else if(auto tof = std::dynamic_pointer_cast<dai::node::ToF>(node)) {
                setupCameraControlQueue(tof, "tof");
            }
#endif
        }

        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::NeuralNetwork> ptr) {
            auto queue = getOutputQueue(xLinkOut, 4, false);
            make_publisher<NNPublisher>(
                    queue,
                    _device_node,
                    30,
                    xLinkOut);
            return true;

        }
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::IMU> inputNode) {
            auto queue = getOutputQueue(xLinkOut, 4, false);
            auto ns = default_frame_mapping()[dai::CameraBoardSocket::CAM_A];
            auto cname = "dai_" + mxId + "_" + ns;

            make_publisher<IMUPublisher>(
                    queue,
                    _device_node,
                    _calibrationHandler,
                    xLinkOut, cname);

            return true;
        }

#ifdef HAS_CR_FORK
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                   const std::string& inputName, std::shared_ptr<dai::node::Camera> inputNode) {
            auto queue = getOutputQueue(xLinkOut, 4, false);

            ros_impl::sensor_msgs::CameraInfo rawCameraInfo;
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
            auto queue = getOutputQueue(xLinkOut, 4, false);

            int width = -1, height = -1;
            auto socket = dai::CameraBoardSocket::RGB;
            auto cameraInfo = CameraInfo(socket, width, height);
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
#endif
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                                      const std::string &inputName, std::shared_ptr<dai::node::MonoCamera> inputNode) {
            auto queue = getOutputQueue(xLinkOut, 4, false);

            int width = 1280, height = 800;
            width = inputNode->getResolutionWidth();
            height = inputNode->getResolutionHeight();

            auto cameraInfo = CameraInfo(inputNode->getBoardSocket(), width, height);
            auto publisher = make_publisher<ImagePublisher>(
                    queue,
                    getNodeHandle(inputNode->getBoardSocket()),
                    30,
                    cameraInfo, xLinkOut);
            return true;
        }
        bool PipelinePublisher::Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                                      const std::string &inputName, std::shared_ptr<dai::node::ColorCamera> inputNode) {
            auto queue = getOutputQueue(xLinkOut, 4, false);

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
                ROS_IMPL_WARN (_device_node, "Don't understand output named %s in ColorCamera. Using default image size for intrinsics", inputName.c_str());
            }

            auto rgbCameraInfo = CameraInfo(inputNode->getBoardSocket(), width, height);
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

            auto queue = getOutputQueue(xLinkOut, 4, false);
            auto alignSocket = stereo->properties.depthAlignCamera;
            if(alignSocket == dai::CameraBoardSocket::AUTO) {
                alignSocket = dai::CameraBoardSocket::LEFT;
            }

            auto depthCameraInfo = CameraInfo(alignSocket, 1280, 800);
            if(inputName == "depth") {
                //converter = std::make_shared<ImageConverter>(_frame_prefix + frame, true);
                make_publisher<DepthPublisher>(
                        queue,
                        _device_node,
                        30,
                        depthCameraInfo,
                        xLinkOut);
            } else if(inputName == "disparity") {
//                auto converter =
//                        std::make_shared<dai::rosBridge::DisparityConverter>(_frame_prefix + frame, 880, 7.5, 20,
//                                                                             2000);  // TODO(sachin): undo hardcoding of baseline
//                publisher = std::make_shared<dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame>>(
//                        queue,
//                        _device_node,
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
                        _device_node,
                        30,
                        depthCameraInfo,
                        xLinkOut);
            } else if(inputName == "rectifiedLeft" || inputName == "rectifiedRight" || inputName == "syncedRight" || inputName == "syncedLeft") {
                bool isLeft = (inputName == "rectifiedLeft" || inputName == "syncedLeft");

                std::string side_name = isLeft ? "left" : "right";
                auto socket = side_name == "left" ? dai::CameraBoardSocket::LEFT : dai::CameraBoardSocket::RIGHT;
                bool isRect = (inputName == "rectifiedLeft" || inputName == "rectifiedRight");
                std::string pub_name = side_name + (isRect ? "/image_rect" : "/image_raw");

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
                    ROS_IMPL_WARN (_device_node, "Could not get input source for %s on stereo node", side_name.c_str());
                    return true;
                }

                auto cameraInfo = CameraInfo(monoNode->getBoardSocket(),
                                                          monoNode->getResolutionWidth(),
                                                          monoNode->getResolutionHeight());
                if(isRect) {
                    for(auto& d : cameraInfo.D)
                        d = 0;
                }
                make_publisher<ImagePublisher>(
                        queue,
                        _device_node,
                        30,
                        cameraInfo,
                        xLinkOut);
            } else {
                ROS_IMPL_WARN (_device_node, "Don't understand output named %s in StereoDepth", inputName.c_str());
            }

            return true;
        }

        void PipelinePublisher::setupDeviceServer() {
#ifdef HAS_DYNAMIC_RECONFIGURE

            auto server = std::make_shared<dynamic_reconfigure::Server<cr_dai_ros::DeviceControlConfig>>(*_device_node);
            ROS_INFO("Setting up device server at %s",  _device_node->getNamespace().c_str());
            auto current_config = std::make_shared<cr_dai_ros::DeviceControlConfig>();
            server->getConfigDefault(*current_config);
            current_config->LogLevel = static_cast<int>(_device->getLogOutputLevel());

            keep_alive.push_back(current_config);

            auto triggger_update = [=](cr_dai_ros::DeviceControlConfig& cfg, unsigned level) {
                if(level & 1) _device->setIrFloodLightBrightness((float)cfg.IrFloodLightBrightness_right, 0x2);
                if(level & 2) _device->setIrFloodLightBrightness((float)cfg.IrFloodLightBrightness_left, 0x1);
                if(level & 4) _device->setIrLaserDotProjectorBrightness((float)cfg.IrLaserDotProjectorBrightness_right, 0x2);
                if(level & 8) _device->setIrLaserDotProjectorBrightness((float)cfg.IrLaserDotProjectorBrightness_left, 0x1);
                if(level & 16) {
                    ROS_IMPL_WARN(_device_node, "Setting log level to %d", cfg.LogLevel);
                    //_device->setLogLevel(static_cast<dai::LogLevel>(cfg.LogLevel));
                    //_device->setLogOutputLevel(static_cast<dai::LogLevel>(cfg.LogLevel));
                }

            };

            server->setCallback([=](cr_dai_ros::DeviceControlConfig& cfg, unsigned level) {
                triggger_update(cfg, level);
            });

            keep_alive.push_back(server);
#endif
        }

        ::ros_impl::Node &PipelinePublisher::getNodeHandle(dai::CameraBoardSocket socket) {
            if(_nodeHandles.find(socket) == _nodeHandles.end()) {
                auto ns = default_frame_mapping()[socket];
                _nodeHandles[socket] = ros_impl::make_node(_device_node, ns);
            }
            return _nodeHandles[socket];
        }

        ros_impl::sensor_msgs::CameraInfo PipelinePublisher::CameraInfo(dai::CameraBoardSocket socket, int width, int height,
                                                              dai::Point2f topLeftPixelId,
                                                              dai::Point2f bottomRightPixelId) {
            auto manager = _cameraManagers[socket];
            if(!manager) {
                auto ns = default_frame_mapping()[socket];
                auto cname = "dai_" + mxId + "_" + ns;
                _cameraManagers[socket] = manager = DepthaiCameraInfoManager::get(_device, _calibrationHandler, socket, getNodeHandle(socket), cname,
                                                                                  "flash:///", width, height, topLeftPixelId, bottomRightPixelId);
            }
            //return ros_impl::sensor_msgs::CameraInfo();
            return manager->getCameraInfo();
        }

        std::shared_ptr<dai::DataOutputQueue>
        PipelinePublisher::getOutputQueue(std::shared_ptr<dai::node::XLinkOut> xlinkOut, int qsize, bool blocking) {
            if(mappedQueues.find(xlinkOut->getStreamName()) != mappedQueues.end()) {
                ROS_IMPL_WARN(this->_device_node, "Output queue %s mapped twice", xlinkOut->getStreamName().c_str());
            }
            mappedQueues.insert(xlinkOut->getStreamName());
            return _device->getOutputQueue(xlinkOut->getStreamName(), qsize, blocking);
        }

    }
}