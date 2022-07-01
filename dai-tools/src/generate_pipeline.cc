#include <memory>
#include <yaml-cpp/emittermanip.h>

#include "cr/dai-tools/PipelineBuilder.h"
#include "yaml-cpp/yaml.h"
#include <typeinfo>

#include <filesystem>

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
            auto sensorInfo = metaInfo.SensorInfo.find(features.socket);
            if(sensorInfo == metaInfo.SensorInfo.end()) {
                metaInfo.SensorInfo[features.socket] = SensorMetaInfo("IMX378", dai::CameraSensorType::COLOR, 30, ColorCameraResolution::THE_1080_P);
            }
            HandleColor(features);
        }

        void PipelineBuilder::HandleOV9_82(const CameraFeatures &features) {
            auto sensorInfo = metaInfo.SensorInfo.find(features.socket);
            bool isColor = false;
            if(sensorInfo == metaInfo.SensorInfo.end()) {
                std::cerr
                        << "Warning: OV9*82 camera can not reliably be differentiated between color and mono. Override `HandleOV9_82` with the correct logic for your board"
                        << std::endl;

                metaInfo.SensorInfo[features.socket] = SensorMetaInfo("OV9*82", isColor ? dai::CameraSensorType::COLOR : dai::CameraSensorType::MONO, 30, ColorCameraResolution::THE_800_P);
            } else {
                isColor = sensorInfo->second.SensorType == dai::CameraSensorType::COLOR;
            }

            if(isColor) {
                HandleColor(features);
            } else {
                HandleMono(features);
            }
        }

        void PipelineBuilder::HandleColor(const CameraFeatures &features) {
            auto sensorInfo = metaInfo.SensorInfo[features.socket];

            auto rgbPicture = pipeline->create<dai::node::ColorCamera>();
            rgbPicture->setFps(sensorInfo.FPS);
            rgbPicture->initialControl.setManualFocus(135);
            rgbPicture->setBoardSocket(features.socket);
            rgbPicture->setResolution(sensorInfo.ColorResolution());

            rgbPicture->setInterleaved(false);

            auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
            auto name = "rgb";
            xoutVideo->setStreamName(name + std::to_string((int)features.socket));
            rgbPicture->isp.link(xoutVideo->input);
        }

        void PipelineBuilder::HandleMono(const CameraFeatures &features) {
            auto sensorInfo = metaInfo.SensorInfo[features.socket];

            auto mono = pipeline->create<dai::node::MonoCamera>();
            mono->setBoardSocket(features.socket);
            mono->setFps(sensorInfo.FPS);
            mono->setResolution(sensorInfo.MonoResolution());

            std::string name = "mono";
            if(stereo_depth_node) {
                mono->out.link(features.socket == dai::CameraBoardSocket::LEFT ? stereo_depth_node->left : stereo_depth_node->right);
                mono->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

                // Stereo has a synced output for left/right, so we don't need one here.
                return;
            }
            auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
            xoutVideo->setStreamName(name + std::to_string((int) features.socket));
            mono->out.link(xoutVideo->input);
        }

        void PipelineBuilder::HandleRaw(const CameraFeatures& features) {
#if HAS_CR_FORK
            auto xinPicture = pipeline->create<dai::node::Camera>();
            xinPicture->setBoardSocket(features.socket);

            auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
            xoutVideo->setStreamName("raw");
            xinPicture->raw.link(xoutVideo->input);
            printf("Creating raw camera on socket %d\n", (int)features.socket);
#endif
        }

        float GetFPS(const SensorMetaInfo& sensorMetaInfo) {
            std::string env_key = "CR_" + sensorMetaInfo.SensorName + "_FPS";
            if(auto fps = std::getenv(env_key.c_str())) {
                return std::stod(fps);
            }
            return sensorMetaInfo.FPS;
        }

        void PipelineBuilder::HandleToF(const CameraFeatures &features) {
#if HAS_CR_FORK
            SensorMetaInfo sensorMetaInfo;
            if(metaInfo.SensorInfo.find(features.socket) == metaInfo.SensorInfo.end()) {
                sensorMetaInfo = metaInfo.SensorInfo[features.socket] =
                        SensorMetaInfo(features.sensorName,dai::CameraSensorType::TOF,
                                       30, dai::CameraProperties::SensorResolution::THE_400_P);
            }

            auto xinPicture = pipeline->create<dai::node::Camera>();
            xinPicture->setBoardSocket(features.socket);
            xinPicture->setFps(GetFPS(sensorMetaInfo));

            auto tof = pipeline->create<dai::node::ToF>();
            if(auto tof_filter_config = std::getenv((std::string("CR_TOF_FILTER_CONFIG_") + features.sensorName).c_str())) {
                tof->setFilterConfig(tof_filter_config);
            }
            tof->initialConfig.get().useLoadedFilter = 1;

            using output_t = decltype(&tof->out);
            std::list<std::pair<std::string, output_t>> outs = {
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
                xoutVideo->setStreamName(kv.first + std::to_string((int)features.socket));
                kv.second->link(xoutVideo->input);
            }
#endif
        }

        void PipelineBuilder::HandleStereo() {
            stereo_depth_node = std::shared_ptr<dai::node::StereoDepth>(pipeline->create<dai::node::StereoDepth>());

            stereo_depth_node->initialConfig.setLeftRightCheckThreshold(10);
            stereo_depth_node->initialConfig.setConfidenceThreshold(220);
            stereo_depth_node->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
            stereo_depth_node->setLeftRightCheck(true);
            stereo_depth_node->setExtendedDisparity(false);
            stereo_depth_node->setSubpixel(true);
            stereo_depth_node->setRuntimeModeSwitch(true);

            if(metaInfo.StereoAlignment == dai::CameraBoardSocket::RIGHT) {
                stereo_depth_node->setDepthAlign(dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
            } else {
                stereo_depth_node->setDepthAlign(dai::RawStereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
            }

            using output_t = decltype(&stereo_depth_node->depth);
            std::list<std::pair<std::string, output_t>> outs = {
                    {"stereo_depth",     &stereo_depth_node->depth},
                    {"confidence_map", &stereo_depth_node->confidenceMap},
                    {"left", &stereo_depth_node->syncedLeft},
                    {"right", &stereo_depth_node->syncedRight},
                    {"rectLeft", &stereo_depth_node->rectifiedLeft},
                    {"rectRight", &stereo_depth_node->rectifiedRight},
            };

            for (auto &kv : outs) {
                auto xoutVideo = pipeline->create<dai::node::XLinkOut>();
                xoutVideo->setStreamName(kv.first );
                kv.second->link(xoutVideo->input);
            }
        }

        void PipelineBuilder::Generate() {
            if(!pipeline)
                pipeline = std::make_shared<dai::Pipeline>();

            auto calibrationData = device->readCalibration();
            auto eeprom = calibrationData.getEepromData();

            bool useImuDefault = eeprom.imuExtrinsics.rotationMatrix.size() != 0 && eeprom.imuExtrinsics.toCameraSocket != dai::CameraBoardSocket::AUTO;
            bool useImu = metaInfo.UseIMU == DeviceMetaInfo::OptionalBool::DEFAULT ? useImuDefault : (metaInfo.UseIMU == DeviceMetaInfo::OptionalBool::TRUE);
            if(useImu) {
                HandleIMU();
            }

#if HAS_CR_FORK
            auto features = device->getConnectedCameraFeatures();
#else
            auto features = device->getConnectedCameraProperties();
#endif
            if(eeprom.stereoRectificationData.leftCameraSocket != dai::CameraBoardSocket::AUTO &&
               eeprom.stereoRectificationData.rightCameraSocket != dai::CameraBoardSocket::AUTO) {
                std::cerr << "Found stereo data " << (int)eeprom.stereoRectificationData.leftCameraSocket << " to " << (int) eeprom.stereoRectificationData.rightCameraSocket << std::endl;
                bool hasRight = false, hasLeft = false, hasToF = false;
                for(auto& feature : features) {
                    if(feature.socket == dai::CameraBoardSocket::RIGHT && feature.sensorName == "IMX378") {
                        // Right and left share an i2c bus and IMX378's don't have an option to have different device IDs. So
                        // we just assume they are in the left socket always for now.
                        continue;
                    }

                    hasRight |= feature.socket == dai::CameraBoardSocket::RIGHT;
                    hasLeft |= feature.socket == dai::CameraBoardSocket::LEFT;
                    hasToF |= !feature.supportedTypes.empty() && feature.supportedTypes[0] == dai::CameraSensorType::TOF;
                }


                bool useStereo = hasRight && hasLeft;
                if(hasToF) {
                    useStereo &= std::getenv("CR_TOF_FILTER_CONFIG_MTP006") == 0 && std::getenv("CR_TOF_FILTER_CONFIG_OZT0358") == 0;
                }
                if(useStereo && metaInfo.StereoAlignment != dai::CameraBoardSocket::AUTO) {
                    HandleStereo();
                } else {
                    std::cerr << "Could not find cameras at stereo positions" << std::endl;
                }
            }

            auto factory = std::map<std::string, std::function<void(const CameraFeatures &features)>>(
            {
                    {"OZT0358", [this](const CameraFeatures &features){this->HandleToF(features);}},
                    {"MTP006", [this](const CameraFeatures &features){this->HandleToF(features);}},
                    {"OV9*82", [this](const CameraFeatures &features){this->HandleOV9_82(features);}},
                    {"IMX378", [this](const CameraFeatures &features){this->HandleIMX378(features);}},
            });

            for(auto& feature : features) {
                auto& fn = factory[feature.sensorName];
                std::cerr << "Found sensor " << feature.sensorName << " on socket " << (int) feature.socket << std::endl;
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

            metaInfo.Save();
        }

        void PipelineBuilder::HandleUnknown(const std::string &name, const CameraFeatures &features) {
            std::cerr << "Don't recognize sensor '" << name << "'..." << std::endl;
            if(features.supportedTypes.size() == 1) {
                switch(features.supportedTypes[0]) {
                    case dai::CameraSensorType::COLOR:
                        HandleColor(features);
                        break;
                    case dai::CameraSensorType::MONO:
                        HandleMono(features);
                        break;
                    case dai::CameraSensorType::TOF:
                        HandleToF(features);
                        break;
                    case dai::CameraSensorType::THERMAL:
                        HandleRaw(features);
                        break;
                }
            } else {
                std::cerr << "Sensor '" << name << "' has more than one supported camera type; mapping as raw" << std::endl;
                HandleRaw(features);
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


        static std::string GetSaveDir() {
            const char* home = getenv("XDG_CONFIG_HOME");
            if(home && home[0]) return home;

            home = getenv("HOME");
            if(home && strcmp(home, "/") == 0) home = "";
            if(home) return std::string(home) + "/.local/share";

            return "./";
        }
        DeviceMetaInfo::DeviceMetaInfo(const std::shared_ptr<dai::Device> &device) : device(device) {
            Name = "dai_" + device->getMxId();
            auto env_name = getenv("DEPTHAI_DEVICE_NAME");
            if(env_name && env_name[0]) {
                Name = env_name;
            }
            Load();
        }

        std::string DeviceMetaInfo::SaveFileName() const {
            auto fn = "dai_" + device->getMxId() + ".yml";
            auto saveDir = GetSaveDir() + "/cr-dai-tools/devices.d/";
            std::filesystem::create_directories(saveDir);
            return saveDir + fn;
        }

        static std::string Socket2Str(dai::CameraBoardSocket s) {
            if(s == dai::CameraBoardSocket::AUTO) {
                return "NONE";
            }
            std::string rtn = "";
            rtn += ('A' + (int) s);
            return rtn;
        }
        static dai::CameraBoardSocket Str2Socket(const std::string& s) {
            if(s == "NONE" || s == "AUTO") {
                return dai::CameraBoardSocket::AUTO;
            }
            if(s.size() == 1 && s[0] >= 'A' && s[0] <= 'H') {
                return static_cast<dai::CameraBoardSocket>(s[0] - 'A');
            }
            fprintf(stderr, "Warning: %s not understood as a camera board socket.", s.c_str());
            return dai::CameraBoardSocket::AUTO;
        }
        static std::vector<std::string> CameraSensorTypeNames = {"COLOR", "MONO", "TOF", "THERMAL"};
        static std::string SensorType2Str(dai::CameraSensorType t) {
            if((int)t > CameraSensorTypeNames.size() || (int)t < 0) {
                return "UNKNOWN";
            }
            return CameraSensorTypeNames[(int)t];
        }
        dai::CameraSensorType Str2SensorType(const std::string& v) {
            for(int i = 0;i < CameraSensorTypeNames.size();i++) {
                if(v == CameraSensorTypeNames[i])
                    return static_cast<CameraSensorType>(i);
            }
            if(v.size() == 1 && v[0] >= '0' && v[0] <= '9') {
                return static_cast<CameraSensorType>(v[0] - '0');
            }
            fprintf(stderr, "Warning: %s not understood as a camera type.", v.c_str());
            return dai::CameraSensorType::MONO;
        }
        void DeviceMetaInfo::Save() {
            YAML::Emitter out;
            out << YAML::BeginMap;
            out << YAML::Key << "Name" << YAML::Value << Name;
            out << YAML::Key << "Id" << YAML::Value << device->getMxId();
            out << YAML::Key << "UseIMU" << YAML::Value << (int)UseIMU;
            out << YAML::Key << "StereoAlignment" << YAML::Value << Socket2Str(StereoAlignment);
            out << YAML::Key << "SensorInfos" << YAML::Value;
            {
                out << YAML::BeginSeq;
                for (auto &infos: this->SensorInfo) {
                    out << YAML::BeginMap;
                    out << YAML::Key << "Socket" << YAML::Value << Socket2Str(infos.first);
                    out << YAML::Key << "SensorName" << YAML::Value << infos.second.SensorName;
                    out << YAML::Key << "FPS" << YAML::Value << infos.second.FPS;
                    out << YAML::Key << "SensorType" << YAML::Value << SensorType2Str(infos.second.SensorType);
                    out << YAML::Key << "Resolution" << YAML::Value << (int) infos.second.Resolution;
                    out << YAML::Key << "Outputs" << YAML::Value << YAML::BeginSeq;
                    for (auto &o: infos.second.Outputs) {
                        out << o;
                    }
                    out << YAML::EndSeq;
                    out << YAML::EndMap;
                }
                out << YAML::EndSeq;
            }
            out << YAML::EndMap;

            auto fn = SaveFileName();
            std::cerr << "Saving metadata file to " << fn << std::endl;
            std::ofstream fs(SaveFileName());
            fs << out.c_str() << std::endl;
        }
        void DeviceMetaInfo::Load() {
            auto fn = SaveFileName();
            std::ifstream fs(fn);
            auto yaml = YAML::Load(fs);
            Name = yaml["Name"].as<std::string>(Name);
            std::cerr << "Loading meta info for " << Name << " from " << fn << std::endl;

            UseIMU = static_cast<OptionalBool>(yaml["UseIMU"].as<int>(-1));
            StereoAlignment = Str2Socket(yaml["StereoAlignment"].as<std::string>("C"));

            for(auto sensorMetadataNode : yaml["SensorInfos"]) {
                auto socket = Str2Socket(sensorMetadataNode["Socket"].as<std::string>("NONE"));
                try {
                    auto newInfo = SensorMetaInfo(sensorMetadataNode["SensorName"].as<std::string>(""),
                                                  Str2SensorType(sensorMetadataNode["SensorType"].as<std::string>("MONO")),
                                                  sensorMetadataNode["FPS"].as<double>(30),
                                                  (SensorResolution) sensorMetadataNode["Resolution"].as<int>(
                                                          2));
                    if (socket != dai::CameraBoardSocket::AUTO) {
                        SensorInfo[(dai::CameraBoardSocket) socket] = newInfo;
                    }
                } catch(std::exception& e) {
                    fprintf(stderr, "Warning: Error reading device meta file %s", e.what());
                }
            }
        }

        SensorMetaInfo::SensorMetaInfo(const std::string& name, dai::CameraSensorType sensorType, double fps, SensorResolution resolution)
                : SensorName(name), SensorType(sensorType), FPS(fps), Resolution(resolution) {}

#ifdef HAS_CR_FORK
        dai::MonoCameraProperties::SensorResolution SensorMetaInfo::MonoResolution() {
            switch(Resolution) {
                case dai::CameraProperties::SensorResolution::THE_400_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_400_P;
                case dai::CameraProperties::SensorResolution::THE_480_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_480_P;
                case dai::CameraProperties::SensorResolution::THE_720_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_720_P;
                case dai::CameraProperties::SensorResolution::THE_800_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_800_P;
                case dai::CameraProperties::SensorResolution::THE_1080_P:
                case dai::CameraProperties::SensorResolution::THE_1200_P:
                case dai::CameraProperties::SensorResolution::THE_4_K:
                case dai::CameraProperties::SensorResolution::THE_5_MP:
                case dai::CameraProperties::SensorResolution::THE_12_MP:
                case dai::CameraProperties::SensorResolution::THE_13_MP:
                default:
                    return dai::MonoCameraProperties::SensorResolution::THE_800_P;
            }
        }

        dai::ColorCameraProperties::SensorResolution SensorMetaInfo::ColorResolution() {
            switch(Resolution) {
                case dai::CameraProperties::SensorResolution::THE_400_P:
                case dai::CameraProperties::SensorResolution::THE_480_P:
                case dai::CameraProperties::SensorResolution::THE_720_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_720_P;
                case dai::CameraProperties::SensorResolution::THE_800_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_800_P;
                case dai::CameraProperties::SensorResolution::THE_1080_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_1080_P;
                case dai::CameraProperties::SensorResolution::THE_1200_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_1200_P;
                case dai::CameraProperties::SensorResolution::THE_4_K:
                    return dai::ColorCameraProperties::SensorResolution::THE_4_K;
                case dai::CameraProperties::SensorResolution::THE_5_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_5_MP;
                case dai::CameraProperties::SensorResolution::THE_12_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_12_MP;
                case dai::CameraProperties::SensorResolution::THE_13_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_13_MP;
                default:
                    return dai::ColorCameraProperties::SensorResolution::THE_1080_P;
            }
        }
#else
        dai::MonoCameraProperties::SensorResolution SensorMetaInfo::MonoResolution() { return (dai::MonoCameraProperties::SensorResolution)Resolution; }
        dai::ColorCameraProperties::SensorResolution SensorMetaInfo::ColorResolution() { return (dai::ColorCameraProperties::SensorResolution)Resolution; }
#endif
    }
}