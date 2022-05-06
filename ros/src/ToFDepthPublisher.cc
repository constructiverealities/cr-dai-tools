#include "cr/dai-tools/ToFDepthPublisher.h"

void cr::dai_rosnode::ToFDepthPublisher::publishCalibrationBlob(const dai::ImgFrame &frame) {
    if(!calibrationBlobPublisher) {
        calibrationBlobPublisher = ros_impl::create_publisher<ros_impl::std_msgs::UInt8MultiArray>(_nh, Name() + "/calibration_blob", 4);//, true);
    }
    ros_impl::std_msgs::UInt8MultiArray blob;
    blob.data = frame.getData();
    calibrationBlobPublisher->publish(blob);
}

void cr::dai_rosnode::ToFDepthPublisher::operator()(std::shared_ptr<dai::ImgFrame> msg) {
    if (msg->getWidth() == 1) {
        publishCalibrationBlob(*msg);
        return;
    }
    ImagePublisher::operator()(msg);
}