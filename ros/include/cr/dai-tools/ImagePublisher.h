#pragma once

#include "ros/ros.h"
#include <memory>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Clock.hpp>

#include "depthai/pipeline/datatypes.hpp"
#include "cr_dai_ros/CameraMetadata.h"
#include "sensor_msgs/Image.h"

#include "cr/dai-tools/Publisher.h"

namespace cr {
    namespace dai_rosnode {
        class ImagePublisher : public Publisher_<dai::ImgFrame, image_transport::CameraPublisher> {
        protected:
            ros::Publisher _cameraMetaPublisher;
            sensor_msgs::CameraInfo _cameraInfoData;

            void operator()(std::shared_ptr<dai::ImgFrame> msg) override;
        public:
            ImagePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                           const ros::NodeHandle& nh,
                           int queueSize,
                           const sensor_msgs::CameraInfo& cameraInfoData,
                           std::shared_ptr<dai::node::XLinkOut>);

            void Setup() override;
        };

    }
}