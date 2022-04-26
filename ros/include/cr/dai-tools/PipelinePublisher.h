#include "depthai/device/Device.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "cr/dai-tools/NodeWalker.h"
#include "ros/ros.h"

namespace cr {
    namespace dai_rosnode {
        struct SetupPostStart;
        struct SetupPublishers {};
        class PipelinePublisher : cr::dai_tools::NodeWalker_<SetupPublishers, std::shared_ptr<dai::node::XLinkOut>, const std::string&> {
            friend class SetupPostStart;

            ::ros::NodeHandle& _pnh;
            dai::Device& _device;
            std::string _frame_prefix;

            std::map<dai::CameraBoardSocket, ::ros::NodeHandle> _nodeHandles;
            ::ros::NodeHandle& getNodeHandle(dai::CameraBoardSocket socket);

            std::vector<std::shared_ptr<void>> keep_alive;
            dai::CalibrationHandler _calibrationHandler;

            template<typename T> void setupCameraControlQueue(std::shared_ptr<T> cam, const std::string& prefix);

            void setupCameraControlServer(dai::CameraBoardSocket socket, const std::string& prefix);
            template<typename T> void setupCameraControlServer(std::shared_ptr<T> cam, const std::string& prefix);

            void setupCameraControlServer(std::shared_ptr<dai::node::StereoDepth> cam, const std::string& prefix);

            void addConfigNodes(dai::Pipeline& pipeline, std::shared_ptr<dai::Node> node);

            template<typename Tp, typename... Args> std::shared_ptr<Tp> make_publisher(Args&&... args)
            {
                auto publisher = std::make_shared<Tp>(args...);
                publisher->Setup();
                keep_alive.template emplace_back(publisher);
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
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::ToF> ptr) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::Camera> ptr) override;
            bool Visit(SetupPublishers, std::shared_ptr<dai::node::XLinkOut> xLinkOut,
                       const std::string& inputName, std::shared_ptr<dai::node::IMU> ptr) override;

        public:
            void BuildPublisherFromPipeline(dai::Pipeline& pipeline);
            PipelinePublisher(::ros::NodeHandle& pnh, dai::Device& device, dai::Pipeline& pipeline);
            PipelinePublisher(::ros::NodeHandle& pnh, dai::Device& device, dai::Pipeline& pipeline, const std::string& frame_prefix);

            void setupDeviceServer();
        };

    }
}