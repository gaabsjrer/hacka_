#ifndef HACKA_GABRIEL__FLY_NODE_HPP
#define HACKA_GABRIEL__FLY_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hacka_gabriel
{
class FlyNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit FlyNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~FlyNode() override;

private:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &);

    CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    void getParameters();
    void configPubSub();
    void configTimers();

    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_point_;

    // rclcpp::Subscription<--->::ConstSharedPtr sub_---_;
    // void subFunc(const --- &msg);

    rclcpp::TimerBase::SharedPtr tmr_stt_machine_;
    void tmrSttMachine();

    double _rate_stt_machine_;
    std::vector<double> _pt1_;
    std::vector<double> _pt2_;
    std::vector<double> _pt3_;
    std::vector<double> _pt4_;

    geometry_msgs::msg::Pose point_;

    bool have_goal_{false};
    bool is_active_{false};
};
}

#endif