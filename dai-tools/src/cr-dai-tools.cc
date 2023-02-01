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
        void DeviceRunner::Start() {
            device->startPipeline(*Pipeline());
        }
        void DeviceRunner::Run() {
            Start();

            std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> outputQueues;
            for(auto& n : device->getOutputQueueNames()) {
                outputQueues[n] = device->getOutputQueue(n, 2, true);
            }

            auto start = std::chrono::high_resolution_clock::now();
            while(ShouldKeepRunning()) {
                auto event = device->getQueueEvent(std::chrono::milliseconds(get_timeout_ms));
                if(auto& queue = outputQueues[event]) {
                    if(auto msg = queue->tryGet()) {
                        OnStreamData(event, msg);
                    }
                    queue->setBlocking(false);
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
            this->OnSetupPipeline(pipeline);
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

        DeviceRunner::DeviceRunner() : device(std::make_unique<dai::Device>()) {
            calibration = device->readCalibration();
        }

        std::array<float, 16> daiExtrinsics2mat4(const std::vector<std::vector<float>> &extrinsics, bool column_major) {
            /// DepthAI extrinsics are src -> dest in cm

            std::array<float, 16> mat4 = { 0 };
            int stride_i = column_major ? 4 : 1;
            int stride_j = column_major ? 1 : 4;
            mat4[stride_i * 3 + stride_j * 3] = 1;
            mat4[stride_i * 0 + stride_j * 3] = extrinsics[0][3] * .01;
            mat4[stride_i * 1 + stride_j * 3] = extrinsics[1][3] * .01;
            mat4[stride_i * 2 + stride_j * 3] = extrinsics[2][3] * .01;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    mat4[i * stride_i + j * stride_j] = extrinsics[i][j];
                }
            }

            return mat4;
        }

    }



}