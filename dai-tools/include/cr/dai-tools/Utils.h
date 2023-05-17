#pragma once

#include <depthai/depthai.hpp>

namespace cr {
    namespace dai_tools {
        std::array<float, 16> daiExtrinsics2mat4(const std::vector<std::vector<float>>& extrinsics, bool column_major=true);

        typedef
        std::vector<
                std::tuple<
                        std::shared_ptr<dai::node::XLinkOut>,
                        std::string,
                        std::shared_ptr<dai::Node>
                >
        > XLinkOutputs;
        XLinkOutputs GetXLinkOuts(dai::Pipeline &pipeline);

    }
}