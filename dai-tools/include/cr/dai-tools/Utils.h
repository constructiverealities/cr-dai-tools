#pragma once

#include <depthai/depthai.hpp>

namespace cr {
    namespace dai_tools {

        std::vector<
                std::tuple<
                        std::shared_ptr<dai::node::XLinkOut>,
                        std::string,
                        std::shared_ptr<dai::Node>
                >
        > GetXLinkOuts(dai::Pipeline &pipeline) {
            std::vector<
                    std::tuple<
                            std::shared_ptr<dai::node::XLinkOut>,
                            std::string,
                            std::shared_ptr<dai::Node>
                    >
            > rtn;

            auto connections = pipeline.getConnectionMap();
            for (auto &connection: connections) {
                auto node = pipeline.getNode(connection.first);
                if (auto xlinkOut = std::dynamic_pointer_cast<dai::node::XLinkOut>(node)) {
                    for (auto &nodeConnection: connection.second) {
                        auto otherNode = pipeline.getNode(nodeConnection.outputId);

                        rtn.emplace_back(xlinkOut, nodeConnection.outputName, otherNode);
//                bool handled = StartVisit(SetupPublishers(), xlinkOut, nodeConnection.outputName, otherNode);
//                if(!handled) {
//                    ROS_IMPL_WARN(_device_node, "Could not map xlinkout named %s", nodeConnection.outputName.c_str());
//                }
                    }
                }
            }
            return rtn;
        }

    }
}