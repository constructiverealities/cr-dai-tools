#pragma once
#include "ros_headers.h"

namespace cr {
    namespace dai_tools {
        namespace tx {
            ros_impl::geometry_msgs::TransformStamped daiExtrinsics2Tx(const std::vector<std::vector<float>>& extrinsics,
                                                                       const std::string& src, const std::string& dst);
            ros_impl::geometry_msgs::TransformStamped identityTx(const std::string& src, const std::string& dst);
        }
    }
}