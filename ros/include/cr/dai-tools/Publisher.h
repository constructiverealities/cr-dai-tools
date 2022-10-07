#pragma once

#include "cr/dai-tools/ros_headers.h"

#include <memory>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Clock.hpp>

#include "depthai/pipeline/datatypes.hpp"


namespace cr {
    namespace dai_rosnode {
        template <typename T, typename S, typename RosPublisher_T = ros_impl::Publisher<S>>
        class Publisher_ {
        protected:
            int perf_count = 0;
            double perf_latency = 0;
            dai::Clock::time_point perf_start;

            int queueSize;
            RosPublisher_T publisher;
            ros_impl::Node _nh;
            std::shared_ptr <dai::DataOutputQueue> _daiMessageQueue;

            std::shared_ptr<dai::node::XLinkOut> xLinkOut;

            virtual std::string Name() const {
                auto name = xLinkOut->getStreamName();
                if(name.length() && name.back() >= '0' && name.back() <= '9')
                    name.pop_back();
                return name;
            }
            virtual bool hasDataListeners() const {
                return ros_impl::get_subscription_count(_nh, publisher) > 0;// ->getNumSubscribers() > 0;
            }
            virtual void operator()(std::shared_ptr<T> msg) {
                bool shouldProduce = hasDataListeners();
                bool doesProduce = !xLinkOut->getMetadataOnly();
                if (shouldProduce != doesProduce) {
                    ROS_IMPL_WARN(_nh, "Setting metadata only to %d on %s", shouldProduce, Name().c_str());
                    xLinkOut->setMetadataOnly(!shouldProduce);
                }

                auto now = dai::Clock::now();
                perf_count++;
                if((now - perf_start).count() > 3e9) {
                    ROS_IMPL_INFO(_nh, "%10s FPS \t%7.2f latency \t%7.2fms",
                             Name().c_str(),
                             (perf_count * 1000. / (double) std::chrono::duration_cast<std::chrono::milliseconds>(now - perf_start).count()),
                             (perf_latency / perf_count));
                    perf_count = 0;
                    perf_latency = 0;
                    perf_start = now;
                }
            }

            ros_impl::Time ros_time(const dai::Timestamp& ts) {
                return ros_time(ts.get());
            }

            ros_impl::Time ros_time(const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& tstamp) {
                auto rosNow = ros_impl::now(_nh);
                auto steadyTime = std::chrono::steady_clock::now();
                auto diffTime = steadyTime - tstamp;
#ifdef HAS_ROS2
                int64_t nsec = rosNow.seconds() - diffTime.count();//rosNow.toNSec() - diffTime.count();
#else
                int64_t nsec = rosNow.toNSec() - diffTime.count();
#endif
                return ros_impl::Time(nsec / 1000000000, nsec % 1000000000);
            }

        public:
            Publisher_(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                           const ros_impl::Node& nh,
                           int queueSize,
                           std::shared_ptr<dai::node::XLinkOut> xlinkOut) : _daiMessageQueue(daiMessageQueue), _nh(nh), xLinkOut(xlinkOut), queueSize(queueSize) {
                perf_start = dai::Clock::now();
            }
            virtual void Setup() {
                ROS_IMPL_INFO(_nh, "Adding callback for %s", xLinkOut->getStreamName().c_str());
                _daiMessageQueue->addCallback([this](std::string name, std::shared_ptr<dai::ADatatype> msg) {
                    (*this)(std::dynamic_pointer_cast<T>(msg));
                });
            }
        };

    }
}