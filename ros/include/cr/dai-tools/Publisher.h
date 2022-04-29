#pragma once

#include "ros/ros.h"
#include <memory>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Clock.hpp>

#include "depthai/pipeline/datatypes.hpp"
#include "cr_dai_ros/CameraMetadata.h"
#include "sensor_msgs/Image.h"

namespace cr {
    namespace dai_rosnode {
        template <typename T, typename RosPublisher_T = ros::Publisher>
        class Publisher_ {
        protected:
            int perf_count = 0;
            double perf_latency = 0;
            dai::Clock::time_point perf_start;

            int queueSize;
            RosPublisher_T publisher;
            ros::NodeHandle _nh;
            std::shared_ptr <dai::DataOutputQueue> _daiMessageQueue;

            std::shared_ptr<dai::node::XLinkOut> xLinkOut;

            virtual std::string Name() const {
                auto name = xLinkOut->getStreamName();
                if(name.length() && name.back() >= '0' && name.back() <= '9')
                    name.pop_back();
                return name;
            }
            virtual bool hasDataListeners() const {
                return publisher.getNumSubscribers() > 0;
            }
            virtual void operator()(std::shared_ptr<T> msg) {
                bool shouldProduce = hasDataListeners();
                bool doesProduce = !xLinkOut->getMetadataOnly();
                if (shouldProduce != doesProduce) {
                    ROS_WARN("Setting metadata only to %d on %s", shouldProduce, Name().c_str());
                    xLinkOut->setMetadataOnly(!shouldProduce);
                }

                auto now = dai::Clock::now();
                perf_count++;
                if((now - perf_start).count() > 3e9) {
                    ROS_INFO("%10s FPS \t%7.2f latency \t%7.2fms",
                             Name().c_str(),
                             (perf_count * 1000. / (double) std::chrono::duration_cast<std::chrono::milliseconds>(now - perf_start).count()),
                             (perf_latency / perf_count));
                    perf_count = 0;
                    perf_latency = 0;
                    perf_start = now;
                }
            }
        public:
            Publisher_(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                           const ros::NodeHandle& nh,
                           int queueSize,
                           std::shared_ptr<dai::node::XLinkOut> xlinkOut) : _daiMessageQueue(daiMessageQueue), _nh(nh), xLinkOut(xlinkOut), queueSize(queueSize) {
                perf_start = dai::Clock::now();
            }
            virtual void Setup() {
                _daiMessageQueue->addCallback([this](std::string name, std::shared_ptr<dai::ADatatype> msg) {
                    (*this)(std::dynamic_pointer_cast<T>(msg));
                });
            }
        };

    }
}