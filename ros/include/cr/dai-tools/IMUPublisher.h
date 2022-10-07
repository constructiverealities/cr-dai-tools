#include "cr/dai-tools/Publisher.h"
#include <depthai/device/Device.hpp>

namespace cr {
    namespace dai_rosnode {
    class IMUPublisher : public Publisher_<dai::IMUData, ros_impl::sensor_msgs::Imu> {
    protected:
        tf2_ros::TransformBroadcaster broadcaster;
        dai::CalibrationHandler& calibrationHandler;
        double publish_rate = 1./30.;
        double last_publish = 0;

        std::string deviceName;

        void operator()(std::shared_ptr<dai::IMUData> msg) override;
        std::string frame();
        public:
            IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros_impl::Node &nh,
                         dai::CalibrationHandler& calibrationHandler, std::shared_ptr<dai::node::XLinkOut> xlinkOut, const std::string& deviceName);
            void Setup() override;
        };

    }
}