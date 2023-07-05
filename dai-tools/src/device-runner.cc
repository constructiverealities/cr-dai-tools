#include "cr/dai-tools/DeviceRunner.h"
#include "cr/dai-tools/PipelineBuilder.h"

namespace cr {
    namespace dai_tools {
        void DeviceRunner::Start() {
            device->startPipeline(*Pipeline());

            for(auto& n : device->getOutputQueueNames()) {
                outputQueues[n] = device->getOutputQueue(n, 10, true);
            }
        }
        int32_t DeviceRunner::Poll() {
            if(!ShouldKeepRunning())
                return -1;

            auto event = device->getQueueEvent(std::chrono::milliseconds(get_timeout_ms));
            if(auto& queue = outputQueues[event]) {
                if(auto msg = queue->tryGet()) {
                    OnStreamData(event, msg);
                }
                queue->setMaxSize(1);
                queue->setBlocking(false);
            }

            return 1;
        }
        void DeviceRunner::Run() {
            Start();

            auto start = std::chrono::high_resolution_clock::now();
            while(Poll()) {
                auto now = std::chrono::high_resolution_clock::now();
                if (now - start > std::chrono::seconds(3)) {
                    for (auto& kv : performance_counters) {
                        printf("%10s FPS \t%7.2f latency \t%7.2fms\n",
                               kv.first.c_str(),
                               (kv.second.count / (double) std::chrono::duration_cast<std::chrono::seconds>(now - start).count()),
                               (kv.second.latency / kv.second.count));
                    }
                    performance_counters.clear();
                    start = now;
                }
            }

        }

        bool DeviceRunner::ShouldKeepRunning() {
            return !device->isClosed();
        }

        void DeviceRunner::OnStreamData(const std::string &stream_name, std::shared_ptr<dai::ADatatype> msg) {
            performance_counters[stream_name].count++;
            if(auto buffer = std::dynamic_pointer_cast<dai::Buffer>(msg)) {
                OnStreamData(stream_name, buffer);
            }
        }

        void DeviceRunner::OnStreamData(const std::string &stream_name, std::shared_ptr<dai::Buffer> msg) {
            if(auto buffer = std::dynamic_pointer_cast<dai::ImgFrame>(msg)) {
                OnStreamData(stream_name, buffer);
            } else if(auto buffer = std::dynamic_pointer_cast<dai::NNData>(msg)) {
                OnStreamData(stream_name, buffer);
            } else if(auto buffer = std::dynamic_pointer_cast<dai::IMUData>(msg)) {
                OnStreamData(stream_name, buffer);
            }
        }

        double ms_since(dai::Clock::time_point tp) {
            auto now = dai::Clock::now();// std::chrono::high_resolution_clock::now();
            return std::chrono::duration_cast<std::chrono::microseconds>(now - tp).count() / 1000.;
        }
        void DeviceRunner::OnStreamData(const std::string &event, std::shared_ptr<dai::ImgFrame> img) {
            performance_counters[event].latency += ms_since(img->getTimestamp());

            bool isDepth = event.find("depth") != std::string::npos;
            bool isCalib = img->getWidth() == 1 || img->getWidth() == 1;

            if(isDepth && isCalib) {
                auto filename = cr::dai_tools::ToFCacheFileName(device);

                if(auto calibration_path_file = std::ofstream(cr::dai_tools::ToFCacheFileName(device), std::ios::binary)) {
                    calibration_path_file.write(reinterpret_cast<const char *>(img->getData().data()), img->getData().size());
                }
            }
        }

        void DeviceRunner::OnStreamData(const std::string &stream_name, std::shared_ptr<dai::IMUData> img) {

        }

        void DeviceRunner::OnStreamData(const std::string &stream_name, std::shared_ptr<dai::NNData> img) {
            performance_counters[stream_name].latency += ms_since(img->getTimestamp());
        }

        std::shared_ptr<dai::Pipeline> DeviceRunner::Pipeline() {
            if(!pipeline)
                SetupPipeline();
            return pipeline;
        }

        DeviceRunner::DeviceRunner() : device(std::make_unique<dai::Device>()) {
            calibration = device->readCalibration();
        }

    }



}