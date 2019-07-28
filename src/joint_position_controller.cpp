
#include <joint_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace velocity_controllers {

JointPositionController::JointPositionController()
  : loop_count_(0)
{}

JointPositionController::~JointPositionController()
{
  sub_command_.shutdown();
}

bool JointPositionController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointPositionController::setCommandCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

std::string JointPositionController::getJointName()
{
  return joint_.getName();
}

double JointPositionController::getPosition()
{
  return joint_.getPosition();
}

// Set the joint position command
void JointPositionController::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void JointPositionController::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

void JointPositionController::starting(const ros::Time& time)
{
  double pos_command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  command_.initRT(command_struct_);
}

void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  double command_velocity = command_struct_.velocity_;
  bool has_velocity_ =  command_struct_.has_velocity_;

  double error, vel_error;
  double commanded_velocity;

  double current_position = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(command_position);

  // Compute position error
  if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  {
   angles::shortest_angular_distance_with_limits(
      current_position,
      command_position,
      joint_urdf_->limits->lower,
      joint_urdf_->limits->upper,
      error);
  }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else //prismatic
  {
    error = command_position - current_position;
  }
  joint_.setCommand(command_velocity);

  // publish state
  if (loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_position;
      controller_state_publisher_->msg_.process_value = current_position;
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_velocity;
      double dummy;
      bool antiwindup;
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void JointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointPositionController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( velocity_controllers::JointPositionController, controller_interface::ControllerBase)
