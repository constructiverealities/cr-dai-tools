#pragma once

#include "cr/dai-tools/ros_headers.h"

#include <memory>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Clock.hpp>

#include "depthai/pipeline/datatypes.hpp"

#include "cr/dai-tools/Publisher.h"
#include "cr/dai-tools/DepthAICameraInfoManager.hpp"

namespace cr {
    namespace dai_rosnode {
    class ImagePublisher : public Publisher_<dai::ImgFrame, ros_impl::sensor_msgs::Image> {
        protected:
#ifdef HAS_IDL_SUPPORT
#ifdef HAS_CAMERA_METADATA_SUPPORT
            ros_impl::Publisher<cr_dai_ros::CameraMetadata> _cameraMetaPublisher;
#endif
#endif
            ros_impl::Publisher<ros_impl::sensor_msgs::CameraInfo> _cameraInfoPub;
            ros_impl::sensor_msgs::CameraInfo _cameraInfoData;

            void operator()(std::shared_ptr<dai::ImgFrame> msg) override;
            bool hasDataListeners() const override;
        public:
            ImagePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                           const ros_impl::Node& nh,
                           int queueSize,
                           const ros_impl::sensor_msgs::CameraInfo& cameraInfoData,
                           std::shared_ptr<dai::node::XLinkOut>);

            void Setup() override;
        };

    }
}