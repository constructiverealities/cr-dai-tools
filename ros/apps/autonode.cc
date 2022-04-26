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
    ros::init(argc, argv, "depthai_device", ros::init_options::AnonymousName);
    auto d = get_device();
    if(!d) {
        return -1;
    }

    if(d->getUsbSpeed() < dai::UsbSpeed::SUPER) {
        fprintf(stderr, "REFUSING TO USE THE DEVICE; USB SPEED IS %d", (int)d->getUsbSpeed());
        return -2;
    }
    fprintf(stderr, "Usb speed is %d\n", (int)d->getUsbSpeed());

    auto pipeline = cr::dai_tools::GeneratePipeline(d);
    std::string tfPrefix = "dai_" + d->getMxId();
    std::string topicPrefix = tfPrefix;

    ros::NodeHandle n(topicPrefix);
    auto publisher = cr::dai_rosnode::PipelinePublisher(n, *d, *pipeline, tfPrefix);

    ros::spin();

    return 0;
}