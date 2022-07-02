#pragma once

#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <utility>

namespace cr {
    namespace dai_tools {
#if HAS_CR_FORK
        typedef dai::CameraFeatures CameraFeatures;
        typedef dai::CameraProperties::SensorResolution SensorResolution;
        typedef SensorResolution ColorCameraResolution;
        typedef SensorResolution MonoCameraResolution;
#else
        typedef dai::CameraProperties CameraFeatures;
        enum class SensorResolution : int32_t { THE_400_P, THE_480_P, THE_720_P, THE_800_P, THE_1080_P, THE_1200_P, THE_4_K, THE_5_MP, THE_12_MP, THE_13_MP };
        typedef SensorResolution ColorCameraResolution;
        typedef SensorResolution MonoCameraResolution;
#endif
        typedef dai::CameraSensorType CameraSensorType;

        std::shared_ptr<dai::Pipeline> GeneratePipeline(std::shared_ptr<dai::Device> &device);

        struct SensorMetaInfo {
            std::string SensorName;
            CameraSensorType SensorType;
            double FPS = 30;
            SensorResolution Resolution = (SensorResolution)0;
            std::vector<std::string> Outputs;

            SensorMetaInfo() {}
            SensorMetaInfo(const std::string& name, CameraSensorType sensorType, double fps, SensorResolution resolution);

            dai::MonoCameraProperties::SensorResolution MonoResolution();

            dai::ColorCameraProperties::SensorResolution ColorResolution();
        };

        class DeviceMetaInfo {
        protected:
            std::shared_ptr<dai::Device> device;
            std::string SaveFileName() const;

            void Load();
        public:
            std::string Name;
            enum class OptionalBool {
                DEFAULT = -1,
                FALSE = 0,
                TRUE = 1
            };
            dai::CameraBoardSocket StereoAlignment = dai::CameraBoardSocket::RIGHT;
            OptionalBool UseIMU = OptionalBool::DEFAULT;
            std::map<dai::CameraBoardSocket, SensorMetaInfo> SensorInfo;

            explicit DeviceMetaInfo(const std::shared_ptr<dai::Device>& device);

            void Save();
        };

    }
}