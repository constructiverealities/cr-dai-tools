#include "cr/dai-tools/ros_headers.h"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "cr/dai-tools/NodeWalker.h"
#include "cr/dai-tools/PipelineBuilder.h"
#include "DepthAICameraInfoManager.hpp"

namespace cr {
    namespace dai_rosnode {
        struct SetupPostStart;
        struct SetupPublishers {};
        class PipelinePublisher : cr::dai_tools::NodeWalker_<SetupPublishers, std::shared_ptr<dai::node::XLinkOut>, const std::string&> {
            friend class SetupPostStart;

            std::string mxId;
            std::set<std::string> mappedQueues;
            std::shared_ptr<dai::DataOutputQueue> getOutputQueue(std::shared_ptr<dai::node::XLinkOut> xlinkOut, int qsize = 4, bool blocking = false);
            ros_impl::sensor_msgs::CameraInfo CameraInfo(dai::CameraBoardSocket cameraId, int width, int height, dai::Point2f topLeftPixelId = {}, dai::Point2f bottomRightPixelId = {});
            ros_impl::Node& _device_node;
            std::shared_ptr<dai::Device> _device;
            std::string _frame_prefix;

            cr::dai_tools::DeviceMetaInfo metaInfo;

            std::map<dai::CameraBoardSocket, ros_impl::Node> _nodeHandles;
            std::map<dai::CameraBoardSocket, std::shared_ptr<DepthaiCameraInfoManager>> _cameraManagers;
            ros_impl::Node& getNodeHandle(dai::CameraBoardSocket socket);

            std::vector<std::shared_ptr<void>> keep_alive;
            dai::CalibrationHandler _calibrationHandler;

#ifdef HAS_CR_FORK
            void setupCameraControlQueue(std::shared_ptr<dai::node::ToF> tof, const std::string& prefix);
            void setupCameraControlServer(std::shared_ptr<dai::node::ToF> cam, const std::string& prefix);
#endif
            template<typename T> void setupCameraControlQueue(std::shared_ptr<T> cam, const std::string& prefix);
            void setupCameraControlServer(dai::CameraBoardSocket socket, const std::string& prefix);
            template<typename T> void setupCameraControlServer(std::shared_ptr<T> cam, const std::string& prefix);

            void setupCameraControlServer(std::shared_ptr<dai::node::StereoDepth> cam, const std::string& prefix);

            void addConfigNodes(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node);

            template<typename Tp, typename... Args> std::shared_ptr<Tp> make_publisher(Args&&... args)
            {
                auto publisher = std::make_shared<Tp>(args...);
                publisher->Setup();
                keep_alive.emplace_back(publisher);
                return publisher;
            }

            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::StereoDepth> stereo) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::NeuralNetwork> ptr) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::ColorCamera> ptr) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::MonoCamera> ptr) override;
#ifdef HAS_CR_FORK
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::ToF> ptr) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::Camera> ptr) override;
#endif
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::IMU> ptr) override;

        public:
            void BuildPublisherFromPipeline(dai::Pipeline& pipeline);
            PipelinePublisher(ros_impl::Node& pnh, std::shared_ptr<dai::Device> device, dai::Pipeline& pipeline);

            void setupDeviceServer();
        };

    }
}