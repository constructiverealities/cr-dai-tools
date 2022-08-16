#pragma once

#include "cr/dai-tools/ros_headers.h"

#include <memory>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Clock.hpp>

#include "depthai/pipeline/datatypes.hpp"

#include "cr/dai-tools/DepthToXYZ.h"
#include "cr/dai-tools/ImagePublisher.h"
#include "cr/dai-tools/DepthAICameraInfoManager.hpp"

namespace cr {
    namespace dai_rosnode {
        class DepthPublisher : public ImagePublisher {
        protected:
            ros_impl::Publisher<ros_impl::sensor_msgs::Image> pointcloudMapPublisher;
            ros_impl::Publisher<ros_impl::sensor_msgs::PointCloud2> pointcloudPublisher;
            ros_impl::sensor_msgs::PointCloud2 pc_template;
            cr::dai_tools::DepthToXYZ depthMapper;
            void operator()(std::shared_ptr<dai::ImgFrame> msg) override;
            bool hasDataListeners() const override;
        public:
            DepthPublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                           const ros_impl::Node& nh,
                           int queueSize,
                           std::shared_ptr<DepthaiCameraInfoManager> cameraInfoManager,
                           std::shared_ptr<dai::node::XLinkOut>, bool isRectified = false);

            void Setup() override;

            static cr::dai_tools::DepthToXYZ createDepthMapper(const ros_impl::sensor_msgs::CameraInfo &info);
        };

    }
}