#pragma once

#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <utility>

#include <cr/dai-tools/DeviceMetaInfo.h>

namespace cr {
    namespace dai_tools {
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