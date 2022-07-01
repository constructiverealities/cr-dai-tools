#include "cr/dai-tools/DepthPublisher.h"
#include <memory>

void cr::dai_rosnode::DepthPublisher::Setup() {
    ImagePublisher::Setup();

    pointcloudMapPublisher = ros_impl::create_publisher<ros_impl::sensor_msgs::Image>(_nh, "pointcloud_map", queueSize);
    pointcloudPublisher = ros_impl::create_publisher<ros_impl::sensor_msgs::PointCloud2>(_nh, "pointcloud", queueSize);

    pc_template.is_dense = true;
    pc_template.height = this->_cameraInfoData.height;
    pc_template.width = this->_cameraInfoData.width;
    pc_template.point_step = sizeof (float ) * 3;
    pc_template.row_step = pc_template.point_step * pc_template.width;

    ros_impl::sensor_msgs::PointField field;
    field.count = 1;
    field.datatype = 7;
    field.offset = 0;

    for(auto& name : {"x", "y", "z"}) {
        field.name = name;
        pc_template.fields.push_back(field);
        field.offset += sizeof(float);
    }

    {
        // In theory this is a 32FC2; but nothing can display that and you always need to know what this is anyway
        // so the encoding isn't super useful as that.
        ros_impl::sensor_msgs::Image pc_map_img;
        pc_map_img.width = _cameraInfoData.width * 2;
        pc_map_img.height = _cameraInfoData.height;
        pc_map_img.header = _cameraInfoData.header;
        auto &map = depthMapper.Map();
        pc_map_img.data.resize(map.size() * sizeof(float));
        memcpy(&pc_map_img.data[0], &map[0], pc_map_img.data.size());
        pc_map_img.encoding = "32FC1";
        pc_map_img.step = pc_map_img.width * sizeof(float);
        pointcloudMapPublisher->publish(pc_map_img);
    }
}

cr::dai_rosnode::DepthPublisher::DepthPublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, const ros_impl::Node& nh, int queueSize,
                                const ros_impl::sensor_msgs::CameraInfo& cameraInfoData, std::shared_ptr<dai::node::XLinkOut> out) :
                                cr::dai_rosnode::ImagePublisher(daiMessageQueue, nh, queueSize, cameraInfoData, out),
                                depthMapper(createDepthMapper(cameraInfoData)){

}

void cr::dai_rosnode::DepthPublisher::operator()(std::shared_ptr<dai::ImgFrame> msg) {
    ImagePublisher::operator()(msg);

    if(ros_impl::get_subscription_count(_nh, pointcloudPublisher) > 0){
        auto pc = pc_template;
#ifndef HAS_ROS2
        _cameraInfoData.header.seq++;
#endif
        pc.header = _cameraInfoData.header;
        pc.header.stamp = ros_impl::now(_nh);
        pc.data.resize(pc.point_step * pc.height * pc.width);
        auto& d = msg->getData();
        depthMapper((uint16_t *)&d[0], (float*)&pc.data[0]);

        pointcloudPublisher->publish(pc);
    }
}

bool cr::dai_rosnode::DepthPublisher::hasDataListeners() const {
    return ImagePublisher::hasDataListeners() || ros_impl::get_subscription_count(_nh, pointcloudPublisher) > 0;
}

cr::dai_tools::DepthToXYZ cr::dai_rosnode::DepthPublisher::createDepthMapper(const ros_impl::sensor_msgs::CameraInfo &info) {
    std::vector<double> lp = {info.K[0], info.K[4], info.K[2], info.K[5]};
    return cr::dai_tools::DepthToXYZ(info.width, info.height, lp, info.D);
}
