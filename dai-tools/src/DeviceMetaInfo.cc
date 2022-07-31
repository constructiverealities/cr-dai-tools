#include <yaml-cpp/emittermanip.h>

#include "cr/dai-tools/PipelineBuilder.h"
#include "yaml-cpp/yaml.h"


namespace cr {
    namespace dai_tools {


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
        static dai::CameraSensorType Str2SensorType(const std::string& v) {
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

        static std::string SensorResolution2Str(SensorResolution r) {
#define X(n) case SensorResolution::n: return #n;
            switch(r) {
                X(THE_400_P)
                X(THE_480_P)
                X(THE_720_P)
                X(THE_800_P)
                X(THE_1080_P)
                X(THE_1200_P)
                X(THE_4_K)
                X(THE_5_MP)
                X(THE_12_MP)
                X(THE_13_MP)
            }
#undef X
            return "Unknown";
        }
        static SensorResolution Str2SensorResolution(const std::string& r) {
#define X(n) if(r == #n) return SensorResolution::n;
                X(THE_400_P)
                X(THE_480_P)
                X(THE_720_P)
                X(THE_800_P)
                X(THE_1080_P)
                X(THE_1200_P)
                X(THE_4_K)
                X(THE_5_MP)
                X(THE_12_MP)
                X(THE_13_MP)
#undef X
            if(r.size() == 1 && r[0] >= '0' && r[0] <= '9') {
                return static_cast<SensorResolution>(r[0] - '0');
            }
            fprintf(stderr, "Unknown sensor resolution '%s', need one of: THE_400_P, THE_480_P, THE_720_P, THE_800_P, THE_1080_P, THE_1200_P, THE_4_K, THE_5_MP, THE_12_MP, THE_13_MP\n", r.c_str());
            return SensorResolution::THE_400_P;
        }

        static std::string GetSaveDir() {
            const char* home = getenv("XDG_CONFIG_HOME");
            if(home && home[0]) return home;

            home = getenv("HOME");
            if(home && strcmp(home, "/") == 0) home = "";

            if(!home) home = getenv("APPDATA");
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
                    out << YAML::Key << "Resolution" << YAML::Value << SensorResolution2Str(infos.second.Resolution);
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
                                                          Str2SensorResolution(sensorMetadataNode["Resolution"].as<std::string>("Unknown")));
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

        dai::MonoCameraProperties::SensorResolution SensorMetaInfo::MonoResolution() {
            switch(Resolution) {
                case SensorResolution::THE_400_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_400_P;
                case SensorResolution::THE_480_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_480_P;
                case SensorResolution::THE_720_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_720_P;
                case SensorResolution::THE_800_P:
                    return dai::MonoCameraProperties::SensorResolution::THE_800_P;
                case SensorResolution::THE_1080_P:
                case SensorResolution::THE_1200_P:
                case SensorResolution::THE_4_K:
                case SensorResolution::THE_5_MP:
                case SensorResolution::THE_12_MP:
                case SensorResolution::THE_13_MP:
                default:
                    return dai::MonoCameraProperties::SensorResolution::THE_800_P;
            }
        }

        dai::ColorCameraProperties::SensorResolution SensorMetaInfo::ColorResolution() {
            switch(Resolution) {
                case SensorResolution::THE_400_P:
                case SensorResolution::THE_480_P:
                case SensorResolution::THE_720_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_720_P;
                case SensorResolution::THE_800_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_800_P;
                case SensorResolution::THE_1080_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_1080_P;
                case SensorResolution::THE_1200_P:
                    return dai::ColorCameraProperties::SensorResolution::THE_1200_P;
                case SensorResolution::THE_4_K:
                    return dai::ColorCameraProperties::SensorResolution::THE_4_K;
                case SensorResolution::THE_5_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_5_MP;
                case SensorResolution::THE_12_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_12_MP;
                case SensorResolution::THE_13_MP:
                    return dai::ColorCameraProperties::SensorResolution::THE_13_MP;
                default:
                    return dai::ColorCameraProperties::SensorResolution::THE_1080_P;
            }
        }


    }
}