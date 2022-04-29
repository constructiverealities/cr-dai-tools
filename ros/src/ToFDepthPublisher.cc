#include "cr/dai-tools/ToFDepthPublisher.h"
#include <std_msgs/UInt8MultiArray.h>

void cr::dai_rosnode::ToFDepthPublisher::publishCalibrationBlob(const dai::ImgFrame &frame) {
    if(!calibrationBlobPublisher) {
        calibrationBlobPublisher = _nh.advertise<std_msgs::UInt8MultiArray>(Name() + "/calibration_blob", 4, true);
    }
    std_msgs::UInt8MultiArray blob;
    blob.data = frame.getData();
    calibrationBlobPublisher.publish(blob);
}

void cr::dai_rosnode::ToFDepthPublisher::operator()(std::shared_ptr<dai::ImgFrame> msg) {
    if (msg->getWidth() == 1) {
        publishCalibrationBlob(*msg);
        return;
    }
    ImagePublisher::operator()(msg);
}