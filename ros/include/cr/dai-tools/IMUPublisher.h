#include "cr/dai-tools/Publisher.h"

namespace cr {
    namespace dai_rosnode {
    class IMUPublisher : public Publisher_<dai::IMUPacket> {
    protected:
            void operator()(std::shared_ptr<dai::IMUPacket> msg) override;

        public:
            IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros::NodeHandle &nh,
                              int queueSize, std::shared_ptr<dai::node::XLinkOut> xlinkOut);
            void Setup() override;
        };

    }
}