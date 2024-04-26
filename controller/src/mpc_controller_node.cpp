#include "MPCController.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_controller");

  ros::NodeHandle nh;

  // Parameters
  std::string topic_base_state;
  std::string topic_base_command;

  int Horizon;
  double dt, low_v_ref, high_v_ref, frequency;
  bool sim;
  std::vector<double> u_min;
  std::vector<double> u_max;
  std::vector<double> x_min;
  std::vector<double> x_max;

  /// LOADING PARAMETERS FROM THE ROS SERVER
  if (!nh.getParam("topic_base_state", topic_base_state))
  {
    ROS_ERROR("Couldn't retrieve the topic name for the state of the base.");
    return -1;
  }
  if (!nh.getParam("topic_base_command", topic_base_command))
  {
    ROS_ERROR("Couldn't retrieve the topic name for commanding the base.");
    return -1;
  }

  /// MPC PARAMETERS
  if (!nh.getParam("Horizon", Horizon))
  {
    ROS_ERROR("Couldn't retrieve the desired Horizon");
    return -1;
  }
  if (!nh.getParam("dt", dt))
  {
    ROS_ERROR("Couldn't retrieve the desired dt");
    return -1;
  }
  if (!nh.getParam("low_v_ref", low_v_ref))
  {
    ROS_ERROR("Couldn't retrieve the desired low_v_ref");
    return -1;
  }
  if (!nh.getParam("high_v_ref", high_v_ref))
  {
    ROS_ERROR("Couldn't retrieve the desired high_v_ref");
    return -1;
  }
  if (!nh.getParam("frequency", frequency))
  {
    ROS_ERROR("Couldn't retrieve the max acceleration for the arm.");
    return -1;
  }
  if (!nh.getParam("sim", sim))
  {
    ROS_ERROR("Couldn't retrieve the sim info.");
    return -1;
  }

  /// Constraints PARAMETERS
  if (!nh.getParam("u_min", u_min))
  {
    ROS_ERROR("Couldn't retrieve the limits of the u_min.");
    return -1;
  }
  if (!nh.getParam("u_max", u_max))
  {
    ROS_ERROR("Couldn't retrieve the limits of the u_max.");
    return -1;
  }
  if (!nh.getParam("x_min", x_min))
  {
    ROS_ERROR("Couldn't retrieve the limits of the x_min.");
    return -1;
  }
  if (!nh.getParam("x_max", x_max))
  {
    ROS_ERROR("Couldn't retrieve the limits of the x_max.");
    return -1;
  }

  // Constructing the controller
  MPCController mpc_controller(nh, frequency, topic_base_command,
                               topic_base_state, Horizon, dt, low_v_ref, high_v_ref, sim,
                               u_min, u_max, x_min, x_max);

  // Running the controller
  mpc_controller.run();

  return 0;
}