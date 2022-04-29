#include "cr/dai-tools/PipelinePublisher.h"
#include "cr/dai-tools/DeviceRunner.h"
#include "cr/dai-tools/PipelineBuilder.h"

static std::shared_ptr<dai::Device> get_device() {
    try {
        return std::make_shared<dai::Device>();
    } catch(std::runtime_error& error) {
        fprintf(stderr, "Could not find connected device (%s)\n", error.what());
        return 0;
    }
}

int main(int argc, char** argv) {
    auto d = get_device();
    ros::init(argc, argv, "dai_" + d->getMxId());
    if(!d) {
        return -1;
    }

    if(d->getUsbSpeed() < dai::UsbSpeed::SUPER) {
        fprintf(stderr, "REFUSING TO USE THE DEVICE; USB SPEED IS %d", (int)d->getUsbSpeed());
        return -2;
    }
    fprintf(stderr, "Usb speed is %d\n", (int)d->getUsbSpeed());

    ROS_INFO("Creating Pipeline...");
    auto pipeline = cr::dai_tools::GeneratePipeline(d);
    cr::dai_tools::DeviceMetaInfo metaInfo(d);
    std::string tfPrefix = metaInfo.Name;
    ros::NodeHandle n(tfPrefix);

    ROS_INFO("Creating Publisher...");
    auto publisher = cr::dai_rosnode::PipelinePublisher(n, d, *pipeline);

    ROS_INFO("Setup done, wait...");
    ros::spin();

    ROS_INFO("Exiting...");
    return 0;
}