#include <depthai/pipeline/nodes.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <depthai/pipeline/datatype/IMUData.hpp>
#include <depthai/utility/Clock.hpp>

namespace cr {
    namespace dai_tools {

        struct OutputPerformanceCounter {
            int count;
            double latency;
        };
        class DeviceRunner {
        protected:
            std::shared_ptr<dai::Pipeline> pipeline;
            std::shared_ptr<dai::Device> device;
            std::map<std::string, OutputPerformanceCounter> performance_counters;
        public:
            virtual ~DeviceRunner() = default;
            explicit DeviceRunner(std::shared_ptr<dai::Device> device) : device(std::move(device)) {}
            DeviceRunner() : device(std::make_unique<dai::Device>()) {}

            virtual bool ShouldKeepRunning();
            virtual void Run();
            virtual void SetupPipeline() = 0;

            virtual std::shared_ptr<dai::Pipeline> Pipeline();
            virtual void OnSetupPipeline(std::shared_ptr<dai::Pipeline> pipeline) {}
            virtual void OnStreamData(const std::string& stream_name, std::shared_ptr<dai::ADatatype> msg);
            virtual void OnStreamData(const std::string& stream_name, std::shared_ptr<dai::Buffer> msg);
            virtual void OnStreamData(const std::string& stream_name, std::shared_ptr<dai::ImgFrame> img);
            virtual void OnStreamData(const std::string& stream_name, std::shared_ptr<dai::IMUData> img);
            virtual void OnStreamData(const std::string& stream_name, std::shared_ptr<dai::NNData> img);
        };

        class AutoDeviceRunner : public DeviceRunner {
        public:
            virtual void SetupPipeline();
            explicit AutoDeviceRunner(std::shared_ptr<dai::Device> device) : DeviceRunner(std::move(device)) {}
            AutoDeviceRunner() : DeviceRunner() {}

        };

    }
}