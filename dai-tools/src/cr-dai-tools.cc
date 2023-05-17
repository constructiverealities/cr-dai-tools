#include "cr/dai-tools/Utils.h"

#include <map>
#include <memory>

#include <depthai/pipeline/nodes.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>

namespace cr::dai_tools {
        std::array<float, 16> daiExtrinsics2mat4(const std::vector<std::vector<float>> &extrinsics, bool column_major) {
            /// DepthAI extrinsics are src -> dest in cm

            std::array<float, 16> mat4 = { 0 };
            int stride_i = column_major ? 4 : 1;
            int stride_j = column_major ? 1 : 4;
            mat4[stride_i * 3 + stride_j * 3] = 1;
            mat4[stride_i * 0 + stride_j * 3] = extrinsics[0][3] * .01;
            mat4[stride_i * 1 + stride_j * 3] = extrinsics[1][3] * .01;
            mat4[stride_i * 2 + stride_j * 3] = extrinsics[2][3] * .01;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    mat4[i * stride_i + j * stride_j] = extrinsics[i][j];
                }
            }

            return mat4;
        }

        XLinkOutputs GetXLinkOuts(dai::Pipeline &pipeline) {
            XLinkOutputs rtn;

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