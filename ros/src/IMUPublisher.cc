#include "cr/dai-tools/IMUPublisher.h"
#include "sensor_msgs/Imu.h"

cr::dai_rosnode::IMUPublisher::IMUPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue,
                                            const ros::NodeHandle &nh, int queueSize,
                                            std::shared_ptr<dai::node::XLinkOut> xlinkOut) :
                                            Publisher_<dai::IMUPacket>(daiMessageQueue, nh, queueSize, xlinkOut){

}

void cr::dai_rosnode::IMUPublisher::operator()(std::shared_ptr<dai::IMUPacket> msg) {
    sensor_msgs::Imu outImuMsg = {};
    outImuMsg.header.stamp = ros::Time::now();
    outImuMsg.header.frame_id = "imu_frame";

    {
        const auto& rVvalues = msg->rotationVector;

        outImuMsg.orientation.x = rVvalues.i;
        outImuMsg.orientation.y = rVvalues.j;
        outImuMsg.orientation.z = rVvalues.k;
        outImuMsg.orientation.w = rVvalues.real;
    }

    {
        const auto& gyroValues = msg->gyroscope;

        outImuMsg.angular_velocity.x = gyroValues.x;
        outImuMsg.angular_velocity.y = gyroValues.y;
        outImuMsg.angular_velocity.z = gyroValues.z;
    }

    {
        const auto& acceleroValues = msg->acceleroMeter;

        outImuMsg.linear_acceleration.x = acceleroValues.x;
        outImuMsg.linear_acceleration.y = acceleroValues.y;
        outImuMsg.linear_acceleration.z = acceleroValues.z;
    }

    publisher.publish(outImuMsg);

    Publisher_::operator()(msg);
}

void cr::dai_rosnode::IMUPublisher::Setup() {
    publisher = _nh.advertise<sensor_msgs::Imu>(Name(), queueSize);
    Publisher_::Setup();
}
