#pragma once

#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <utility>

namespace cr {
    namespace dai_tools {
        typedef dai::CameraFeatures CameraFeatures;

        std::shared_ptr<dai::Pipeline> GeneratePipeline(std::shared_ptr<dai::Device> &device);

        struct SensorMetaInfo {
            bool IsColor;
            double FPS = 30;
            dai::CameraProperties::SensorResolution Resolution = dai::CameraProperties::SensorResolution::THE_720_P;
            std::vector<std::string> Outputs;

            SensorMetaInfo() {}
            SensorMetaInfo(bool isColor, double fps, dai::CameraProperties::SensorResolution resolution);

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
            std::map<dai::CameraBoardSocket, SensorMetaInfo> SensorInfo;

            explicit DeviceMetaInfo(const std::shared_ptr<dai::Device>& device);

            void Save();
        };

        class PipelineBuilder {
        protected:
            std::shared_ptr<dai::Pipeline> pipeline;
            std::shared_ptr<dai::Device> device;
            std::map<std::string, int> used_names;
            std::shared_ptr<dai::node::StereoDepth> stereo_depth_node;

            DeviceMetaInfo metaInfo;

            std::string get_unique_name(const std::string& prefix);
        public:
            explicit PipelineBuilder(const std::shared_ptr<dai::Device>& device) : device(device), metaInfo(device) {}
            virtual ~PipelineBuilder() = default;
            virtual void HandleUnknown(const std::string& name, const CameraFeatures& features);
            virtual void HandleIMU();

            virtual void HandleMTP006(const CameraFeatures& features);
            virtual void HandleIMX378(const CameraFeatures& features);
            virtual void HandleOV9_82(const CameraFeatures& features);

            virtual void HandleToF(const CameraFeatures& features);
            virtual void HandleColor(const CameraFeatures& features);
            virtual void HandleMono(const CameraFeatures& features);

            virtual void HandleStereo();
            virtual void Generate();
            std::shared_ptr<dai::Pipeline> Pipeline();
        };
    }
}