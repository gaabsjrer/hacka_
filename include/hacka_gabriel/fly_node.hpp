#ifndef HACKA_GABRIEL__FLY_NODE_HPP
#define HACKA_GABRIEL__FLY_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/bool.hpp"
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
    void configServices();

    // points publisher
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr pub_point_;

    // 'have a goal' condition subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr sub_have_goal_;
    void subHaveGoal(const std_msgs::msg::Bool &msg);

    // start service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_;
    void srvStartStateMachine(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // client takeoff
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_takeoff_;

    // client land
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_land_;

    // state machine timer
    rclcpp::TimerBase::SharedPtr tmr_stt_machine_;
    void tmrSttMachine();

    double _rate_stt_machine_;
    std::vector<double> _pt1_;
    std::vector<double> _pt2_;
    std::vector<double> _pt3_;
    std::vector<double> _pt4_;

    geometry_msgs::msg::Pose point_;

    std::string state_ = "INIT";
    bool start_{false};    
    bool have_goal_{false};
    bool is_active_{false};
};
}

#endif