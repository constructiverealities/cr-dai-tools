#include "cr/dai-tools/ImagePublisher.h"
namespace cr {
    namespace dai_rosnode {
        class ToFDepthPublisher : public ImagePublisher {
            ros::Publisher calibrationBlobPublisher;
            void publishCalibrationBlob(const dai::ImgFrame& frame);

        protected:
            void operator()(std::shared_ptr<dai::ImgFrame> msg) override;

        public:
            ToFDepthPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros::NodeHandle &nh,
                              int queueSize, const sensor_msgs::CameraInfo &cameraInfoData,
                              std::shared_ptr<dai::node::XLinkOut> xlinkOut) : ImagePublisher(daiMessageQueue, nh, queueSize,
                                                                              cameraInfoData, xlinkOut) {}
        };

    }
}