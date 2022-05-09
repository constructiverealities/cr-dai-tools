#include "cr/dai-tools/ros_headers.h"

namespace ros_impl {
#ifdef HAS_ROS2
    void spin(const Node &n) {
        rclcpp::spin(n);
    }

    Node make_node(const Node &p, const std::string &ns) {
        return p->create_sub_node(ns);
    }

    Node make_node(const std::string &ns) {
        return std::make_shared<rclcpp::Node>(ns);
    }

    Node init(int argc, const char *const *argv, const std::string &name) {
        rclcpp::init(argc, argv);
        return make_node(name);
    }

    rclcpp::Time now(const Node &node) {
        return node->get_clock()->now();
    }

    const char *Namespace(const Node &n) { return n->get_effective_namespace().c_str(); }
#else
    void spin(const Node &n) {
        ros::spin();
    }

    Node make_node(const Node &p, const std::string &ns) {
        return std::make_shared<ros::NodeHandle>(*p, ns);
    }

    Node make_node(const std::string &ns) {
        return std::make_shared<ros::NodeHandle>(ns);
    }

    Node init(int argc, const char *const *argv, const std::string &name) {
        ros::init(argc, (char**)argv, name, 0);
        return make_node(name);
    }

    Time now(const Node &node) {
        return Time::now();
    }


    const char *Namespace(const Node &n) { return n->getNamespace().c_str(); }
#endif
}

#ifdef HAS_ROS2
namespace rosidl_typesupport_cpp {
    template<> rosidl_message_type_support_t const * get_message_type_support_handle<ros_impl::sensor_msgs::CameraInfo>() {
        return rosidl_typesupport_cpp::get_message_type_support_handle<sensor_msgs::msg::CameraInfo>();
    }
}
#endif