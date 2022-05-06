#include "cr/dai-tools/Publisher.h"

namespace cr {
    namespace dai_rosnode {
    class IMUPublisher : public Publisher_<dai::IMUData, ros_impl::sensor_msgs::Imu> {
    protected:
            void operator()(std::shared_ptr<dai::IMUData> msg) override;

        public:
            IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros_impl::Node &nh,
                              int queueSize, std::shared_ptr<dai::node::XLinkOut> xlinkOut);
            void Setup() override;
        };

    }
}