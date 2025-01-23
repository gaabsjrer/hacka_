#include "fly_mission/fly_node.hpp"

namespace fly_mission
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

  pub_point_ = create_publisher<geometry_msgs::msg::Pose>("go_to_out", 1);

  // sub_---_ = create_subscription<--->("topic_in", 1, std::bind(&FlyNode::subFunc, this, std::placeholders::_1));
}

// configTimers()
void FlyNode::configTimers() 
{
  RCLCPP_INFO(get_logger(), "initTimers");

  tmr_stt_machine_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_stt_machine_), std::bind(&FlyNode::tmrSttMachine, this), nullptr);
}

// tmrSttMachine()
void FlyNode::tmrSttMachine()
{
  point_.orientation.x = 0;
  point_.orientation.y = 0;
  point_.orientation.z = 0;
  point_.orientation.w = 0;

  point_.position.x = _pt2_[0];
  point_.position.y = _pt2_[1];
  point_.position.z = _pt2_[2];

  pub_point_->publish(point_);
}

// subFunc()
// void FlyNode::subFunc(const --- &msg) 
// {}

} // namespace fly_mission

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fly_mission::FlyNode)