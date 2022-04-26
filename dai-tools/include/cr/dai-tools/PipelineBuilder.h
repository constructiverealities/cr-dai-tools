#include <depthai/depthai.hpp>
#include <depthai/device/Device.hpp>
#include <utility>

namespace cr {
    namespace dai_tools {
#ifdef DEPTHAI_HAS_TOF_NODE
        typedef dai::CameraFeatures CameraFeatures;
#else
        typedef dai::CameraProperties CameraFeatures;
#endif

        std::shared_ptr<dai::Pipeline> GeneratePipeline(std::shared_ptr<dai::Device> &device);

        class PipelineBuilder {
        protected:
            std::shared_ptr<dai::Pipeline> pipeline;
            std::shared_ptr<dai::Device> device;
            std::map<std::string, int> used_names;
            std::shared_ptr<dai::node::StereoDepth> stereo_depth_node;

            std::string get_unique_name(const std::string& prefix);
        public:
            explicit PipelineBuilder(std::shared_ptr<dai::Device> device) : device(std::move(device)) {}
            virtual ~PipelineBuilder() = default;
            virtual void HandleUnknown(const std::string& name, const CameraFeatures& features);
            virtual void HandleIMU();

            virtual void HandleMTP006(const CameraFeatures& features);
            virtual void HandleIMX378(const CameraFeatures& features);
            virtual void HandleOV9_82(const CameraFeatures& features);

            virtual void HandleToF(const CameraFeatures& features);
            virtual void HandleColor(const CameraFeatures& features, dai::ColorCameraProperties::SensorResolution resolution);
            virtual void HandleMono(const CameraFeatures& features, dai::MonoCameraProperties::SensorResolution resolution);

            virtual void HandleStereo();
            virtual void Generate();
            std::shared_ptr<dai::Pipeline> Pipeline();
        };
    }
}