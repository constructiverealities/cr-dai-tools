#include "cr/dai-tools/ImagePublisher.h"
namespace cr {
    namespace dai_rosnode {
        class NNPublisher : public Publisher_<dai::NNData, ros_impl::sensor_msgs::Image> {

        protected:
            void operator()(std::shared_ptr<dai::NNData> msg) override;

        public:
            void Setup() override;

        public:
            NNPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros_impl::Node &nh,
                              int queueSize,
                              std::shared_ptr<dai::node::XLinkOut> xlinkOut) : Publisher_(daiMessageQueue, nh, queueSize,
                                                                                              xlinkOut) {}
        };

    }
}