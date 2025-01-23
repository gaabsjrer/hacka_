#include "hacka_gabriel/fly_node.hpp"

namespace hacka_gabriel
{
// FlyNode()    
FlyNode::FlyNode(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("fly_node", "", options) 
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("rate.stt_machine", rclcpp::ParameterValue(1.0));
  declare_parameter("waypoints.pt1", std::vector<double>(3, 0.0));
  declare_parameter("waypoints.pt2", std::vector<double>(3, 0.0));
  declare_parameter("waypoints.pt3", std::vector<double>(3, 0.0));
  declare_parameter("waypoints.pt4", std::vector<double>(3, 0.0));
}

// ~FlyNode()
FlyNode::~FlyNode(){ 
}

// on_configure()
CallbackReturn FlyNode::on_configure(const rclcpp_lifecycle::State &) 
{
  RCLCPP_INFO(get_logger(), "Configuring");

  getParameters();
  configPubSub();
  configTimers();
  configServices();

  return CallbackReturn::SUCCESS;
}

// on_activate()
CallbackReturn FlyNode::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &state) 
{
  RCLCPP_INFO(get_logger(), "Activating");

  pub_point_->on_activate();

  is_active_ = true;

  return CallbackReturn::SUCCESS;
}

// on_deactivate()
CallbackReturn FlyNode::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &state) 
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pub_point_->on_deactivate();

  is_active_ = false;

  return CallbackReturn::SUCCESS;
}

// on_cleanup()
CallbackReturn FlyNode::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &state) 
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  pub_point_.reset();

  // sub_---_.reset();

  tmr_stt_machine_.reset();

  return CallbackReturn::SUCCESS;
}

// on_shutdown()
CallbackReturn FlyNode::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &state) 
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return CallbackReturn::SUCCESS;
}

// get_parameters()
void FlyNode::getParameters()
{
  get_parameter("rate.stt_machine", _rate_stt_machine_);
  get_parameter("waypoints.pt1", _pt1_);
  get_parameter("waypoints.pt2", _pt2_);
  get_parameter("waypoints.pt3", _pt3_);
  get_parameter("waypoints.pt4", _pt4_);
}

// configPubSub()
void FlyNode::configPubSub() 
{
  RCLCPP_INFO(get_logger(), "initPubSub");

  pub_point_ = create_publisher<geometry_msgs::msg::Pose>("/uav1/goto", 1);

  sub_have_goal_ = create_subscription<std_msgs::msg::Bool>("/uav1/have_goal", 1, std::bind(&FlyNode::subHaveGoal, this, std::placeholders::_1));
}

// configTimers()
void FlyNode::configTimers() 
{
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_stt_machine_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_stt_machine_), std::bind(&FlyNode::tmrSttMachine, this), nullptr);
}

// configServices()
void FlyNode::configServices()
{
  RCLCPP_INFO(get_logger(), "initServices");

  client_takeoff_ = create_client<std_srvs::srv::Trigger>("/uav1/takeoff");
  client_land_ = create_client<std_srvs::srv::Trigger>("/uav1/land");

  srv_start_ =
      create_service<std_srvs::srv::Trigger>("start_state_machine", std::bind(&FlyNode::srvStartStateMachine, this, std::placeholders::_1, std::placeholders::_2));
}

// subHaveGoal()
void FlyNode::subHaveGoal(const std_msgs::msg::Bool &msg) 
{
    have_goal_ = msg.data;
}

void FlyNode::srvStartStateMachine([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = true;
  response->message = "starting...";

  start_ = true;
}

// timrSttMachine()
void FlyNode::tmrSttMachine()
{
  if(start_ ==  true)
  {
    point_.orientation.x = 0;
    point_.orientation.y = 0;
    point_.orientation.z = 0;
    point_.orientation.w = 1;

    if(state_ == "INIT" && !have_goal_){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

      auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
        RCLCPP_INFO(this->get_logger(), "%s", future.get()->message.c_str());
      };

      client_takeoff_->async_send_request(request, callback_result);

      state_ == "TAKEOFF";
    }

    /*
    else if(state_ == "TAKEOFF" && !have_goal){
      point_.position.x = _pt1_[0];
      point_.position.y = _pt1_[1];
      point_.position.z = _pt1_[2];

      state_ = "PT1";

      pub_point_->publish(point_);
    }

    else if(state_ == "PT1" && !have_goal){
      point_.position.x = _pt2_[0];
      point_.position.y = _pt2_[1];
      point_.position.z = _pt2_[2];

      state_ = "PT2";

      pub_point_->publish(point_);
    }

    else if(state_ == "PT2" && !have_goal){
      point_.position.x = _pt3_[0];
      point_.position.y = _pt3_[1];
      point_.position.z = _pt3_[2];

      state_ = "PT3";

      pub_point_->publish(point_);
    }

    else if(state_ == "PT3" && !have_goal){
      point_.position.x = _pt4_[0];
      point_.position.y = _pt4_[1];
      point_.position.z = _pt4_[2];

      state_ = "PT4";

      pub_point_->publish(point_);
    }
    */
  
    else if(state_ == "TAKEOFF" && !have_goal_){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

      auto callback_result = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) -> void {
        RCLCPP_INFO(this->get_logger(), "%s", future.get()->message.c_str());
      };

      client_land_->async_send_request(request, callback_result);
    }

      // point_.position.x = _pt2_[0];
      // point_.position.y = _pt2_[1];
      // point_.position.z = _pt2_[2];

      // pub_point_->publish(point_);
  }
}

} // namespace hacka_gabriel

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hacka_gabriel::FlyNode)