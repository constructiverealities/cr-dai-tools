#include "cr/dai-tools/IMUPublisher.h"
#include "cr/dai-tools/Tx.h"

cr::dai_rosnode::IMUPublisher::IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue,
                                            const ros_impl::Node &nh, dai::CalibrationHandler& calibrationHandler,
                                            std::shared_ptr<dai::node::XLinkOut> xlinkOut, const std::string& deviceName) :
        Publisher_<dai::IMUData, ros_impl::sensor_msgs::Imu>(daiMessageQueue, nh, 30, xlinkOut),
        calibrationHandler(calibrationHandler), deviceName(deviceName)
#ifdef HAS_ROS2
                                            ,broadcaster(nh)
#endif
                                            {

}
std::string cr::dai_rosnode::IMUPublisher::frame() {
    return this->deviceName + "_imu";
}
void cr::dai_rosnode::IMUPublisher::operator()(std::shared_ptr<dai::IMUData> imuData) {
    ros_impl::sensor_msgs::Imu outImuMsg;
    for(auto& msg : imuData->packets) {
        outImuMsg.header.stamp = ros_time(msg.acceleroMeter.timestamp);
        outImuMsg.header.frame_id = frame();

        {
            const auto &rVvalues = msg.rotationVector;

            outImuMsg.orientation.x = rVvalues.i;
            outImuMsg.orientation.y = rVvalues.j;
            outImuMsg.orientation.z = rVvalues.k;
            outImuMsg.orientation.w = -rVvalues.real;
        }

        {
            const auto &gyroValues = msg.gyroscope;

            outImuMsg.angular_velocity.x = gyroValues.x;
            outImuMsg.angular_velocity.y = gyroValues.y;
            outImuMsg.angular_velocity.z = gyroValues.z;
        }

        {
            const auto &acceleroValues = msg.acceleroMeter;

            outImuMsg.linear_acceleration.x = acceleroValues.x;
            outImuMsg.linear_acceleration.y = acceleroValues.y;
            outImuMsg.linear_acceleration.z = acceleroValues.z;
        }

        publisher->publish(outImuMsg);

        if (last_publish + publish_rate < outImuMsg.header.stamp.toSec()) {
            last_publish = outImuMsg.header.stamp.toSec();
            ros_impl::geometry_msgs::TransformStamped pose_msg;
            //pose_msg.header.seq = 1;
            pose_msg.header.frame_id = frame();
            pose_msg.child_frame_id = "map";
            pose_msg.header.stamp = outImuMsg.header.stamp;
            pose_msg.transform.translation.x = 0;
            pose_msg.transform.translation.y = 0;
            pose_msg.transform.translation.z = 0;
            tf2::Matrix3x3 m2;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    m2[i][j] = i == j;
                }
            }
            tf2::Quaternion q;
            m2.getRotation(q);
            pose_msg.transform.rotation = outImuMsg.orientation;
            broadcaster.sendTransform(pose_msg);
        }

        Publisher_::operator()(imuData);
    }
}

void cr::dai_rosnode::IMUPublisher::Setup() {
    publisher = ros_impl::create_publisher<ros_impl::sensor_msgs::Imu>(_nh, Name(), queueSize);
    //publisher = _nh.advertise<sensor_msgs::Imu>(Name(), queueSize);
    Publisher_::Setup();
}
