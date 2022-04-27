#include <memory>

#include "cr/dai-tools/PipelineBuilder.h"

namespace cr {
    namespace dai_tools {
        std::shared_ptr<dai::Pipeline> GeneratePipeline(std::shared_ptr<dai::Device> &device) {
            PipelineBuilder builder(device);
            return builder.Pipeline();
        }

        void PipelineBuilder::HandleMTP006(const CameraFeatures &features) {
            HandleToF(features);
        }

        void PipelineBuilder::HandleIMX378(const CameraFeatures &features) {
            HandleColor(features, dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        }

        void PipelineBuilder::HandleOV9_82(const CameraFeatures &features) {
            std::cerr << "Warning: OV9*82 camera can not reliably be differentiated between color and mono. Override `HandleOV9_82` with the correct logic for your board" << std::endl;
            bool isColor = features.socket != dai::CameraBoardSocket::CAM_C;
            if(isColor) {
                HandleColor(features, dai::ColorCameraProperties::SensorResolution::THE_720_P);
            } else {
                HandleMono(features, dai::MonoCameraProperties::SensorResolution::THE_720_P);
            }
        }

        void PipelineBuilder::HandleColor(const CameraFeatures &features,
                                          dai::ColorCameraProperties::SensorResolution resolution) {
            auto rgbPicture = pipeline->create<dai::node::ColorCamera>();
            rgbPicture->setFps(30);
            rgbPicture->initialControl.setManualFocus(135);
            rgbPicture->initialControl.setManualExposure(10000, 1597);
            rgbPicture->setBoardSocket(features.socket);
            rgbPicture->setResolution(resolution);

            rgbPicture->setInterleaved(false);

            auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
            auto name = get_unique_name("rgb");
            {
                xoutVideo->setStreamName(name);
                rgbPicture->isp.link(xoutVideo->input);
            }
        }

        std::string PipelineBuilder::get_unique_name(const std::string &prefix) {
            int cnt = used_names[prefix];
            used_names[prefix]++;
            if(cnt == 0) {
                return prefix;
            }
            return prefix + std::to_string(cnt);
        }

        void PipelineBuilder::HandleMono(const CameraFeatures &features,
                                         dai::MonoCameraProperties::SensorResolution resolution) {
            auto mono = pipeline->create<dai::node::MonoCamera>();
            mono->setBoardSocket(features.socket);
            mono->setFps(30);
            mono->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
            mono->initialControl.setManualExposure(1000, 1599);
            auto xoutVideo = pipeline->create<dai::node::XLinkOut>();

            std::string name = "mono";
            if(stereo_depth_node) {
                mono->out.link(features.socket == dai::CameraBoardSocket::LEFT ? stereo_depth_node->left : stereo_depth_node->right);
                name = features.socket == dai::CameraBoardSocket::LEFT ? "left" : "right";
            }
            xoutVideo->setStreamName(get_unique_name(name));
            mono->out.link(xoutVideo->input);
        }

        void PipelineBuilder::HandleToF(const CameraFeatures &features) {
            auto xinPicture = pipeline->create<dai::node::Camera>();
            xinPicture->setBoardSocket(features.socket);
            auto tof = pipeline->create<dai::node::ToF>();

            std::list<std::pair<std::string, typeof(tof->out) *>> outs = {
                    {"depth",     &tof->out},
                    {"amplitude", &tof->amp_out},
                    {"raw", &xinPicture->raw},
                    //{"rgb_pc",    &tof->rgb_pc_out},
                    {"error", &tof->err_out},
            };

            if(auto tof_filter_fn = std::getenv("CR_TOF_FILTER_BLOB")) {
                auto nn = pipeline->create<dai::node::NeuralNetwork>();
                nn->setNumInferenceThreads(2);
                nn->setNumPoolFrames(4);
                nn->setBlobPath(tof_filter_fn);
                nn->input.setBlocking(false);
                xinPicture->raw.link(nn->input);

                outs.emplace_back(std::make_pair("raw_filtered", &nn->out));
            }

            xinPicture->raw.link(tof->inputImage);

            for (auto &kv : outs) {
                auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
                xoutVideo->setStreamName(kv.first);
                kv.second->link(xoutVideo->input);
            }
        }

        void PipelineBuilder::HandleStereo() {
            stereo_depth_node = std::shared_ptr<dai::node::StereoDepth>(pipeline->create<dai::node::StereoDepth>());

            stereo_depth_node->initialConfig.setLeftRightCheckThreshold(45);
            stereo_depth_node->initialConfig.setConfidenceThreshold(180);
            stereo_depth_node->setLeftRightCheck(true);
            stereo_depth_node->setExtendedDisparity(false);
            stereo_depth_node->setSubpixel(true);
            stereo_depth_node->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);

            auto xout = pipeline->create<dai::node::XLinkOut>();
            xout->setStreamName("stereo_depth");
            stereo_depth_node->depth.link(xout->input);
        }

        void PipelineBuilder::Generate() {
            if(!pipeline)
                pipeline = std::make_shared<dai::Pipeline>();

            auto calibrationData = device->readCalibration();
            auto eeprom = calibrationData.getEepromData();

            auto features = device->getConnectedCameraFeatures();
            if(eeprom.stereoRectificationData.leftCameraSocket != dai::CameraBoardSocket::AUTO &&
               eeprom.stereoRectificationData.rightCameraSocket != dai::CameraBoardSocket::AUTO) {
                bool hasRight = false, hasLeft = false;
                for(auto& feature : features) {
                    hasRight = feature.socket == dai::CameraBoardSocket::RIGHT;
                    hasLeft = feature.socket == dai::CameraBoardSocket::LEFT;
                }
                if(hasRight && hasLeft) {
                    HandleStereo();
                }
            }

            auto factory = std::map<std::string, std::function<void(const CameraFeatures &features)>>(
            {
                    {"MTP006", [this](const CameraFeatures &features){this->HandleToF(features);}},
                    {"OV9*82", [this](const CameraFeatures &features){this->HandleOV9_82(features);}},
                    {"IMX378", [this](const CameraFeatures &features){this->HandleIMX378(features);}},
            });

            for(auto& feature : features) {
                auto& fn = factory[feature.sensorName];
                if(feature.socket == dai::CameraBoardSocket::RIGHT && feature.sensorName == "IMX378") {
                    // Right and left share an i2c bus and IMX378's don't have an option to have different device IDs. So
                    // we just assume they are in the left socket always for now.
                    continue;
                }
                if(fn) {
                    fn(feature);
                } else {
                    HandleUnknown(feature.sensorName, feature);
                }
            }

        }

        void PipelineBuilder::HandleUnknown(const std::string &name, const CameraFeatures &features) {
            std::cerr << "Don't recognize sensor '" << name << "'." << std::endl;
            if(features.supportedTypes.size() == 1) {
                switch(features.supportedTypes[0]) {
                    case dai::CameraSensorType::COLOR:
                        HandleColor(features, dai::ColorCameraProperties::SensorResolution::THE_720_P);
                        break;
                    case dai::CameraSensorType::MONO:
                        HandleMono(features, dai::MonoCameraProperties::SensorResolution::THE_720_P);
                        break;
                    case dai::CameraSensorType::TOF:
                        HandleToF(features);
                        break;
                    case dai::CameraSensorType::THERMAL:
                        break;
                }
            } else {
                std::cerr << "Sensor '" << name << "' has more than one supported camera type; ignoring" << std::endl;
            }
        }

        void PipelineBuilder::HandleIMU() {
            auto imuNode = pipeline->create<dai::node::IMU>();
            imuNode->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
            imuNode->setBatchReportThreshold(1);
            auto xout = pipeline->create<dai::node::XLinkOut>();
            xout->setStreamName("imu");
            imuNode->out.link(xout->input);
        }

        std::shared_ptr<dai::Pipeline> PipelineBuilder::Pipeline() {
                if(!pipeline)
                    Generate();
            return pipeline;
        }
    }
}