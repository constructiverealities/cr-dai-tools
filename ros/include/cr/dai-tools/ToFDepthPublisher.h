#include "cr/dai-tools/ImagePublisher.h"
namespace cr {
    namespace dai_rosnode {
        class ToFDepthPublisher : public ImagePublisher {
            ros_impl::Publisher<ros_impl::std_msgs::UInt8MultiArray> calibrationBlobPublisher;
            void publishCalibrationBlob(const dai::ImgFrame& frame);

        protected:
            void operator()(std::shared_ptr<dai::ImgFrame> msg) override;

        public:
            ToFDepthPublisher(const std::shared_ptr<dai::DataOutputQueue> &daiMessageQueue, const ros_impl::Node &nh,
                              int queueSize, std::shared_ptr<DepthaiCameraInfoManager> cameraInfoManager,
                              std::shared_ptr<dai::node::XLinkOut> xlinkOut) : ImagePublisher(daiMessageQueue, nh, queueSize,
                                                                                              cameraInfoManager, xlinkOut) {}
        };

    }
}