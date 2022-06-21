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
        typedef int32_t SensorResolution;
        typedef dai::ColorCameraProperties::SensorResolution ColorCameraResolution;
        typedef dai::MonoCameraProperties::SensorResolution MonoCameraResolution;
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
#ifndef HAS_CR_FORK
            SensorMetaInfo(const std::string& name, CameraSensorType sensorType, double fps, ColorCameraResolution resolution) : SensorMetaInfo(name, sensorType, fps, (SensorResolution)resolution){}
            SensorMetaInfo(const std::string& name, CameraSensorType sensorType, double fps, MonoCameraResolution resolution) : SensorMetaInfo(name, sensorType, fps, (SensorResolution)resolution){}
#endif
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
            OptionalBool UseStereo = OptionalBool::DEFAULT, UseIMU = OptionalBool::DEFAULT;
            std::map<dai::CameraBoardSocket, SensorMetaInfo> SensorInfo;

            explicit DeviceMetaInfo(const std::shared_ptr<dai::Device>& device);

            void Save();
        };

        class PipelineBuilder {
        protected:
            std::shared_ptr<dai::Pipeline> pipeline;
            std::shared_ptr<dai::Device> device;
            std::shared_ptr<dai::node::StereoDepth> stereo_depth_node;

            DeviceMetaInfo metaInfo;

        public:
            explicit PipelineBuilder(const std::shared_ptr<dai::Device>& device) : device(device), metaInfo(device) {}
            virtual ~PipelineBuilder() = default;
            virtual void HandleUnknown(const std::string& name, const CameraFeatures& features);
            virtual void HandleIMU();

            virtual void HandleMTP006(const CameraFeatures& features);
            virtual void HandleIMX378(const CameraFeatures& features);
            virtual void HandleOV9_82(const CameraFeatures& features);

            virtual void HandleToF(const CameraFeatures& features);
            virtual void HandleRaw(const CameraFeatures& features);
            virtual void HandleColor(const CameraFeatures& features);
            virtual void HandleMono(const CameraFeatures& features);

            virtual void HandleStereo();
            virtual void Generate();
            std::shared_ptr<dai::Pipeline> Pipeline();
        };
    }
}