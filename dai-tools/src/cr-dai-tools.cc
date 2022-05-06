#include "cr/dai-tools/DeviceRunner.h"
#include "cr/dai-tools/PipelineBuilder.h"

#include <map>
#include <string>
#include <memory>
#include <depthai/device/DataQueue.hpp>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <depthai/pipeline/datatype/IMUData.hpp>
#include <depthai/utility/Clock.hpp>

namespace cr {
    namespace dai_tools {

        void DeviceRunner::Run() {
            device->startPipeline(*Pipeline());

            std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> outputQueues;
            for(auto& n : device->getOutputQueueNames()) {
                outputQueues[n] = device->getOutputQueue(n, 4, false);
            }

            auto start = std::chrono::high_resolution_clock::now();
            while(ShouldKeepRunning()) {
                auto event = device->getQueueEvent(std::chrono::milliseconds(100));
                if(auto& queue = outputQueues[event]) {
                    while(auto msg = queue->tryGet()) {
                        OnStreamData(event, msg);
                    }
                }

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

        void AutoDeviceRunner::SetupPipeline() {
            pipeline = GeneratePipeline(device);
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
        void DeviceRunner::OnStreamData(const std::string &stream_name, std::shared_ptr<dai::ImgFrame> img) {
            performance_counters[stream_name].latency += ms_since(img->getTimestamp());
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

    }
}