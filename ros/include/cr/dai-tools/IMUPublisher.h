#include "cr/dai-tools/Publisher.h"
#include <depthai/device/Device.hpp>

namespace cr {
    namespace dai_rosnode {
    class IMUPublisher : public Publisher_<dai::IMUData, ros_impl::sensor_msgs::Imu> {
    protected:
        tf2_ros::TransformBroadcaster broadcaster;
        ros_impl::geometry_msgs::TransformStamped imu2refcam;
        dai::CalibrationHandler& calibrationHandler;
        std::string referenceCamera;

        void operator()(std::shared_ptr<dai::IMUData> msg) override;

        public:
            IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros_impl::Node &nh,
                         dai::CalibrationHandler& calibrationHandler, std::shared_ptr<dai::node::XLinkOut> xlinkOut, const std::string& ref_camera);
            void Setup() override;
        };

    }
}