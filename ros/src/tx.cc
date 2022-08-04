#include "cr/dai-tools/Tx.h"

namespace cr {
    namespace dai_tools {
        namespace tx {
            ros_impl::geometry_msgs::TransformStamped identityTx(const std::string& src, const std::string& dst) {
                ros_impl::geometry_msgs::TransformStamped pose_msg;
                //pose_msg.header.seq = 1;
                pose_msg.header.frame_id = src;
                pose_msg.child_frame_id = dst;

                pose_msg.transform.translation.x = 0;
                pose_msg.transform.translation.y = 0;
                pose_msg.transform.translation.z = 0;
                tf2::Matrix3x3 m2;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        m2[i][j] = i == j;
                    }
                }
                tf2::Quaternion q;
                m2.getRotation(q);
                pose_msg.transform.rotation.x = q.x();
                pose_msg.transform.rotation.y = q.y();
                pose_msg.transform.rotation.z = q.z();
                pose_msg.transform.rotation.w = q.w();
                return pose_msg;
            }
            ros_impl::geometry_msgs::TransformStamped daiExtrinsics2Tx(const std::vector<std::vector<float>>& extrinsics, const std::string& src, const std::string& dst) {
                /// TransformStamped describes the transform from frame_id _to_ child_frame_id; in meters
                /// DepthAI extrinsics are src -> dest in cm
                ros_impl::geometry_msgs::TransformStamped pose_msg;
                //pose_msg.header.seq = 1;
                pose_msg.header.frame_id = src;
                pose_msg.child_frame_id = dst;

                pose_msg.transform.translation.x = extrinsics[0][3] * .01;
                pose_msg.transform.translation.y = extrinsics[1][3] * .01;
                pose_msg.transform.translation.z = extrinsics[2][3] * .01;
                tf2::Matrix3x3 m2;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        m2[i][j] = extrinsics[i][j];
                    }
                }
                tf2::Quaternion q;
                m2.getRotation(q);
                pose_msg.transform.rotation.x = q.x();
                pose_msg.transform.rotation.y = q.y();
                pose_msg.transform.rotation.z = q.z();
                pose_msg.transform.rotation.w = q.w();
                return pose_msg;
            }
        }
    }
}