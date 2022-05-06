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

    int get_subscription_count(const Node &n, const std::shared_ptr<image_transport::CameraPublisher> &p) {
        return n->count_subscribers(p->getTopic());
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

    int get_subscription_count(const Node &n, const std::shared_ptr<image_transport::CameraPublisher> &p) {
        return p->getNumSubscribers();
    }

    const char *Namespace(const Node &n) { return n->getNamespace().c_str(); }
#endif
}