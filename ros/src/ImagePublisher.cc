#include "cr/dai-tools/ImagePublisher.h"

#include <utility>

uint8_t clamp8(int v) {
    if(v < 0)
        return 0;
    if(v > 255)
        return 255;
    return v;
}
void nv12_to_rgb(int width, int height, const uint8_t* yuv, uint8_t* rgb) {
    int Y, U, V;

    for(int j = 0;j < height;j++) {
        for (int i = 0; i < width; i++) {
            int subindex = i / 2 + j / 2 * (width / 2);

            Y = yuv[j * width + i];
            U = yuv[width * height + subindex * 2 + 0];
            V = yuv[width * height + subindex * 2 + 1];

            int B = clamp8(1.164*(Y - 16)                   + 2.018*(U - 128));
            int G = clamp8(1.164*(Y - 16) - 0.813*(V - 128) - 0.391*(U - 128));
            int R = clamp8(1.164*(Y - 16) + 1.596*(V - 128));
            rgb[(j * width + i) * 3 + 0] = R;
            rgb[(j * width + i) * 3 + 1] = G;
            rgb[(j * width + i) * 3 + 2] = B;
        }
    }

}
void yuv420_to_rgb(int width, int height, const uint8_t* yuv, uint8_t* rgb) {
    int Y, U, V;

    for(int j = 0;j < height;j++) {
        for (int i = 0; i < width; i++) {
            int subindex = i / 2 + j / 2 * (width / 2);

            Y = yuv[j * width + i];
//            U = yuv[width * height + subindex * 2 + 0];
//            V = yuv[width * height + subindex * 2 + 1];
            U = yuv[width * height + subindex];
            V = yuv[width * height + width * height / 4 + subindex];

            int B = clamp8(1.164*(Y - 16)                   + 2.018*(U - 128));
            int G = clamp8(1.164*(Y - 16) - 0.813*(V - 128) - 0.391*(U - 128));
            int R = clamp8(1.164*(Y - 16) + 1.596*(V - 128));
            rgb[(j * width + i) * 3 + 0] = R;
            rgb[(j * width + i) * 3 + 1] = G;
            rgb[(j * width + i) * 3 + 2] = B;
        }
    }
}

static void dai_to_rosimg(std::shared_ptr<dai::ImgFrame> daiImg, sensor_msgs::Image& rosImg) {
    rosImg.width = daiImg->getWidth();
    rosImg.height = daiImg->getHeight();
    rosImg.data = daiImg->getData();

    switch (daiImg->getType()) {
        case dai::RawImgFrame::Type::RGB161616:
            rosImg.encoding = "rgb16";
            break;
        case dai::RawImgFrame::Type::RAW8:
        case dai::RawImgFrame::Type::GRAY8:
            rosImg.encoding = "mono8";
            break;
        case dai::RawImgFrame::Type::RAW10:
        case dai::RawImgFrame::Type::RAW12:
        case dai::RawImgFrame::Type::RAW14:
        case dai::RawImgFrame::Type::RAW16:
            rosImg.encoding = "16UC1";
            break;
        case dai::RawImgFrame::Type::YUV420p:
            rosImg.encoding = "rgb8";
            rosImg.data.resize(rosImg.width * rosImg.height * 3);
            yuv420_to_rgb(rosImg.width, rosImg.height, daiImg->getData().data(), rosImg.data.data());
            break;
        case dai::RawImgFrame::Type::NV12:
            rosImg.encoding = "rgb8";
            rosImg.data.resize(rosImg.width * rosImg.height * 3);
            nv12_to_rgb(rosImg.width, rosImg.height, daiImg->getData().data(), rosImg.data.data());
            break;
        default:
            ROS_WARN("Do not understand dai type %d", (int)daiImg->getType());
    }

    rosImg.step = rosImg.data.size() / rosImg.height;
}

cr::dai_rosnode::ImagePublisher::ImagePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                const ros::NodeHandle& nh,
                                                int queueSize,
                                                const sensor_msgs::CameraInfo& cameraInfoData,
                                                std::shared_ptr<dai::node::XLinkOut> xlinkOut) :
        _cameraInfoData(cameraInfoData),
        Publisher_<dai::ImgFrame, image_transport::CameraPublisher>(daiMessageQueue, nh, queueSize, xlinkOut) {
    ROS_INFO("Creating image publisher for ns %s/%s", nh.getNamespace().c_str(), xLinkOut->getStreamName().c_str());
}

void cr::dai_rosnode::ImagePublisher::operator()(std::shared_ptr<dai::ImgFrame> inFrame) {
    //ROS_WARN("Start %s", Name().c_str());
    auto tstamp = inFrame->getTimestamp();
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    long int nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);

    std_msgs::Header header;
    header.frame_id = _cameraInfoData.header.frame_id;
    header.seq = inFrame->getSequenceNum();
    header.stamp = rosStamp;
    std::string encoding;

    _cameraInfoData.header.seq = inFrame->getSequenceNum();
    _cameraInfoData.header.stamp = rosStamp;

    if(hasDataListeners()) {
        sensor_msgs::Image imageBuffer;
        dai_to_rosimg(inFrame, imageBuffer);
        imageBuffer.header = header;
        publisher.publish(imageBuffer, _cameraInfoData);
    }

    cr_dai_ros::CameraMetadata msg;
    msg.header = header;
    msg.lens_position = inFrame->getLensPosition() < 0 ? -1 : (inFrame->getLensPosition() / 255.);
    msg.exposure_time = inFrame->getExposureTime();
    msg.category = inFrame->getCategory();
    msg.sensitivity = inFrame->getSensitivity();
    _cameraMetaPublisher.publish(msg);
    //ROS_WARN("Stop %s", Name().c_str());

    auto now = dai::Clock::now();// std::chrono::high_resolution_clock::now();
    auto ms_since = std::chrono::duration_cast<std::chrono::microseconds>(now - tstamp).count() / 1000.;
    perf_latency += ms_since;

    Publisher_::operator()(inFrame);
}

void cr::dai_rosnode::ImagePublisher::Setup() {
    _cameraMetaPublisher = _nh.advertise<cr_dai_ros::CameraMetadata>(Name() + "/camera_metadata", queueSize);

    image_transport::ImageTransport it(_nh);
    publisher = it.advertiseCamera(Name() + "/image", queueSize);
    Publisher_::Setup();
}

