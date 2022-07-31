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

    if(!d) {
        return -1;
    }

    if(d->getUsbSpeed() < dai::UsbSpeed::SUPER) {
        fprintf(stderr, "REFUSING TO USE THE DEVICE; USB SPEED IS %d", (int)d->getUsbSpeed());
        return -2;
    }
    fprintf(stderr, "Usb speed is %d\n", (int)d->getUsbSpeed());

    auto pipeline = cr::dai_tools::GeneratePipeline(d);
    cr::dai_tools::DeviceMetaInfo metaInfo(d);
    std::string tfPrefix = metaInfo.Name;

    auto g = ros_impl::init(argc, argv, "dai_" + d->getMxId());
    auto n = ros_impl::make_node(g, tfPrefix);
    ROS_IMPL_INFO(n, "Creating Pipeline... %s", ros_impl::Namespace(n));

    ROS_IMPL_INFO(n, "Creating Publisher...");
    auto publisher = cr::dai_rosnode::PipelinePublisher(n, d, *pipeline);

    ROS_IMPL_INFO(n, "Setup done, wait...");

    while(ros_impl::spinOnce(g) && !d->isClosed() && d->isPipelineRunning()) {

    }


    return 0;
}