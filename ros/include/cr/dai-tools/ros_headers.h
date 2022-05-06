#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef HAS_ROS1
#include "ros/ros.h"
#define HAS_IDL_SUPPORT 1
#include <sensor_msgs/CameraInfo.h>

#include <tf2/LinearMath/Matrix3x3.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ros/macros.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <camera_calibration_parsers/parse.h>

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Imu.h>

#include "cr_dai_ros/CameraMetadata.h"
#include "cr_dai_ros/StereoDepthConfig.h"
#include "cr_dai_ros/CameraControlConfig.h"
#include "cr_dai_ros/DeviceControlConfig.h"
#include <image_transport/image_transport.h>

#include <std_msgs/UInt8MultiArray.h>

#define HAS_DYNAMIC_RECONFIGURE

#define ROS_IMPL_INFO(n, ...)  ROS_INFO(__VA_ARGS__)
#define ROS_IMPL_ERROR(n, ...)  ROS_ERROR(__VA_ARGS__)
#define ROS_IMPL_WARN(n, ...)  ROS_WARN(__VA_ARGS__)

namespace ros_impl {
    typedef tf2::Matrix3x3 Matrix3x3;

    typedef std::shared_ptr<ros::NodeHandle> Node;
    typedef std::shared_ptr<ros::Timer> Timer;
    void spin(const Node& n);

    Node make_node(const Node& p, const std::string& ns);
    Node make_node(const std::string& ns);
    Node init(int argc, char const * const argv[], const std::string& name);
    template <typename T>
    using Publisher = std::shared_ptr<ros::Publisher>;

    const char* Namespace(const Node& n);

    using Time = ros::Time;
    Time now(const Node& node);

    static inline int get_subscription_count(const Node& n, const std::shared_ptr<ros::Publisher>& p) {
        return p->getNumSubscribers();
    }

    int get_subscription_count(const Node& n, const std::shared_ptr<image_transport::CameraPublisher>& p);

    namespace std_msgs {
        typedef ::std_msgs::Header Header;
        typedef ::std_msgs::UInt8MultiArray UInt8MultiArray;
    }
    namespace sensor_msgs {
        typedef ::sensor_msgs::Image Image;
        typedef ::sensor_msgs::Imu Imu;
        typedef ::sensor_msgs::CameraInfo CameraInfo;

        typedef ::sensor_msgs::SetCameraInfo SetCameraInfo;
    }

    namespace geometry_msgs {
        typedef ::geometry_msgs::TransformStamped TransformStamped;
    }
    typedef std::shared_ptr< ros::ServiceServer> ServiceServer;

    template<typename ServiceT, typename CallbackT, typename CallbackClassT, typename... Args>
    static inline ServiceServer create_service(const Node& node, const std::string & service_name, CallbackT && callback, CallbackClassT* self, Args&&... args) {
        return std::make_shared<ros::ServiceServer>(std::move(node->advertiseService(service_name, callback, self, args...)));
//        (node->advertiseService(service_name, [=](typename ServiceT::Request& req, typename ServiceT::Response& rsp) -> bool{
//            return (self->*callback)(req, rsp, args...);
//        }));
        //return 0;
    }

    template<typename CallbackT>
    static inline auto create_wall_timer(const Node& node, double period, CallbackT&& callback) {
        return std::make_shared<ros::Timer>(node->createTimer(ros::Duration(period), [=](const ros::TimerEvent&) {
            callback();
        }));
    }

    template<typename MessageT, typename PublisherT = ros::Publisher, typename... Args>
    static inline std::shared_ptr<PublisherT> create_publisher(
            const Node& node, Args&&... args) {
        return std::make_shared<PublisherT>(node->template advertise<MessageT>(args...));
    }
}
#endif

#ifdef HAS_ROS2

#include <image_transport/image_transport.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "camera_calibration_parsers/parse.hpp"

#define ROS_IMPL_INFO(n, ...)  RCLCPP_INFO((n)->get_logger(), __VA_ARGS__)
#define ROS_IMPL_ERROR(n, ...)  RCLCPP_ERROR((n)->get_logger(), __VA_ARGS__)
#define ROS_IMPL_WARN(n, ...)  RCLCPP_WARN((n)->get_logger(), __VA_ARGS__)
#define ROS_HELPER_EXPORT

#ifdef HAS_IDL_SUPPORT
#include "cr_dai_ros/msg/detail/camera_metadata__struct.hpp"
namespace cr_dai_ros {
    using CameraMetadata = cr_dai_ros::msg::CameraMetadata;
}
#endif

namespace ros_impl {
    typedef std::shared_ptr<rclcpp::Node> Node;
    typedef std::shared_ptr<rclcpp::TimerBase> Timer;
    void spin(const Node& n);

    Node make_node(const Node& p, const std::string& ns);
    Node make_node(const std::string& ns);
    Node init(int argc, char const * const argv[], const std::string& name);
    template <typename T>
    using Publisher = std::shared_ptr<rclcpp::Publisher<T>>;

    const char* Namespace(const Node& n);

    using Time = rclcpp::Time;
    Time now(const Node& node);

    template <typename T>
    static inline int get_subscription_count(const Node& n, const Publisher<T>& p) {
        return p->get_subscription_count();
    }

    int get_subscription_count(const Node& n, const std::shared_ptr<image_transport::CameraPublisher>& p);

    namespace std_msgs {
        typedef ::std_msgs::msg::Header Header;
        typedef ::std_msgs::msg::UInt8MultiArray UInt8MultiArray;
    }
    namespace sensor_msgs {
        typedef ::sensor_msgs::msg::Image Image;
        typedef ::sensor_msgs::msg::Imu Imu;
        struct CameraInfo : public ::sensor_msgs::msg::CameraInfo {
            std::array<double, 9>& K;
            decltype(d)& D;
            decltype(r)& R;
            decltype(p)& P;

            CameraInfo(const ::sensor_msgs::msg::CameraInfo& ci) : ::sensor_msgs::msg::CameraInfo(ci), K(k), D(d), R(r), P(p) {}
            CameraInfo(const CameraInfo& ci) : ::sensor_msgs::msg::CameraInfo(ci), K(k), D(d), R(r), P(p) {}
            CameraInfo() : K(k), D(d), R(r), P(p) {}

            CameraInfo& operator=(const CameraInfo& ci) {
                ::sensor_msgs::msg::CameraInfo::operator=(ci);
                return *this;
            }

        };
        typedef ::sensor_msgs::srv::SetCameraInfo SetCameraInfo;
    }

    namespace geometry_msgs {
        typedef ::geometry_msgs::msg::TransformStamped TransformStamped;
    }
    typedef std::shared_ptr<rclcpp::Service<::sensor_msgs::srv::SetCameraInfo>> ServiceServer;

    template<typename ServiceT, typename CallbackT, typename CallbackClassT, typename... Args>
    static inline std::shared_ptr<rclcpp::Service<ServiceT>> create_service(const Node& node, const std::string & service_name, CallbackT && callback, CallbackClassT* self, Args&&... args) {
        return node->template create_service<ServiceT>(service_name, [=](std::shared_ptr<typename ServiceT::Request> req, std::shared_ptr<typename ServiceT::Response> rsp) {
            return (self->*callback)(*req, *rsp, args...);
        });
    }

    template<typename CallbackT>
    static inline auto create_wall_timer(const Node& node, double period, CallbackT callback) {
        return node->template create_wall_timer(std::chrono::milliseconds (int(1000*period)), [=]() {
            callback();
        });
    }

    template<typename MessageT, typename PublisherT = rclcpp::Publisher<MessageT>, typename... Args>
    static inline std::shared_ptr<PublisherT> create_publisher(
            const Node& node, Args&&... args) {
        return node->template create_publisher<MessageT>(args...);
    }
}
#endif