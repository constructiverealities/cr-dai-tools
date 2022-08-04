#pragma once

#include "depthai/depthai.hpp"


namespace cr {
    namespace dai_tools {

        template<typename... Args>
        struct NodeWalker_ {
            virtual bool StartVisit(Args... args, std::shared_ptr <dai::Node> node) {
                return VisitNode<
                        dai::node::ColorCamera,
                        dai::node::AprilTag,
#ifdef HAS_CR_FORK
                        dai::node::Camera,
                        dai::node::ToF,
#endif
                        dai::node::DetectionNetwork,
                        dai::node::EdgeDetector,
                        dai::node::FeatureTracker,
                        dai::node::ImageManip,
                        dai::node::IMU,
                        dai::node::MonoCamera,
                        dai::node::NeuralNetwork,
                        dai::node::ObjectTracker,
                        dai::node::StereoDepth,
                        dai::node::Script,
                        dai::node::SpatialDetectionNetwork,
                        dai::node::SpatialLocationCalculator,
                        dai::node::SPIIn,
                        dai::node::SPIOut,
                        dai::node::VideoEncoder,
                        dai::node::XLinkIn,
                        dai::node::XLinkOut>(args..., node);
            }

            virtual void VisitAll(Args... args, std::shared_ptr<dai::Pipeline> pipeline) {
                VisitAll(args..., pipeline->getAllNodes());
            }
            virtual void VisitAll(Args... args, std::vector <std::shared_ptr<dai::Node>> nodes) {
                for (auto &n: nodes) {
                    StartVisit(args..., n);
                }
            }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::AprilTag>) { return false; }

#ifdef HAS_CR_FORK
            virtual bool Visit(Args... args, std::shared_ptr <dai::node::Camera>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::ToF>) { return false; }
#endif
            virtual bool Visit(Args... args, std::shared_ptr <dai::node::ColorCamera>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::DetectionNetwork>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::EdgeDetector>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::FeatureTracker>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::ImageManip>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::IMU>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::MonoCamera>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::NeuralNetwork>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::ObjectTracker>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::Script>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::SpatialDetectionNetwork>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::SpatialLocationCalculator>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::SPIIn>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::SPIOut>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::StereoDepth>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::SystemLogger>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::VideoEncoder>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::XLinkIn>) { return false; }

            virtual bool Visit(Args... args, std::shared_ptr <dai::node::XLinkOut>) { return false; }

            template<int None = 0>
            bool VisitNode(Args... args, std::shared_ptr <dai::Node> inputNode) {
                return false;
            }

            template<typename T, typename... TArgs>
            bool VisitNode(Args... args, std::shared_ptr <dai::Node> inputNode) {
                auto typedNode = std::dynamic_pointer_cast<T>(inputNode);
                if (typedNode) {
                    return Visit(args..., typedNode);
                }
                return VisitNode<TArgs...>(args..., inputNode);
            }
        };
        typedef NodeWalker_<> NodeWalker;
    }
}