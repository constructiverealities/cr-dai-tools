#include "cr/dai-tools/NNPublisher.h"

void cr::dai_rosnode::NNPublisher::Setup() {
    publisher = ros_impl::create_publisher<ros_impl::sensor_msgs::Image>(_nh, Name() + "/image", queueSize);
    Publisher_::Setup();
}

void cr::dai_rosnode::NNPublisher::operator()(std::shared_ptr<dai::NNData> nn_data) {
    auto now = dai::Clock::now();// std::chrono::high_resolution_clock::now();
    auto ms_since = std::chrono::duration_cast<std::chrono::microseconds>(now - nn_data->getTimestamp()).count() / 1000.;
    perf_latency += ms_since;

    ros_impl::std_msgs::Header header;
    header.frame_id = "";
    //header.seq = nn_data->getSequenceNum();

    uint64_t us = nn_data->getTimestamp().time_since_epoch().count();
    header.stamp = ros_impl::Time(us / 1000000, us % 1000000);

    if(hasDataListeners()) {
        auto buffer = nn_data->getFirstLayerFp16();

        ros_impl::sensor_msgs::Image imageBuffer;
        imageBuffer.data.resize(buffer.size() * sizeof(float));
        memcpy(imageBuffer.data.data(), buffer.data(), imageBuffer.data.size());

        int w, h, c = 1;
        auto layerInfo = nn_data->getAllLayers()[0];
        if(layerInfo.dims.size() > 2) {
            c = layerInfo.dims[0];
            h = layerInfo.dims[1];
            w = layerInfo.dims[2];
        } else {
            h = layerInfo.dims[0];
            w = layerInfo.dims[1];
        }

        imageBuffer.width = w;
        imageBuffer.height = h;
        imageBuffer.encoding = c == 1 ? "32FC1" : "32FC3";
        imageBuffer.step = imageBuffer.data.size() / imageBuffer.height;

        imageBuffer.header = header;
        publisher->publish(imageBuffer);
    }

    Publisher_::operator()(nn_data);
}
