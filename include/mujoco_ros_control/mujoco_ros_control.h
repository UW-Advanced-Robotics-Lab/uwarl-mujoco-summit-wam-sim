/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   mujoco_ros_control.h
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
* 
* Current version specifically for UWARL
* Last edit: Nov 28, 2023 (Tim van Meijel)
*
**/

#ifndef MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
#define MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

// ros_control
#include <mujoco_ros_control/robot_hw_sim.h>
#include <mujoco_ros_control/robot_hw_sim_plugin.h>

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>


namespace mujoco_ros_control
{

class MujocoRosControl
{
public:
  MujocoRosControl();
  virtual ~MujocoRosControl();

  // initialize params and controller manager
  bool init(ros::NodeHandle &nodehandle);

  // step update function
  std::map<std::string, double >  update();

  // interface loader
  boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin> > robot_hw_sim_loader_;

  // robot simulator interface
  boost::shared_ptr<mujoco_ros_control::RobotHWSimPlugin> robot_hw_sim_;

  // Callback for mujoco
  void readCallback_mujoco(const sensor_msgs::JointState::ConstPtr& msg);

  // initialize mujoco joint data vector
  std::map<std::string, std::vector<double> > list_mj_data;

  // init control variable
  std::map<std::string, double > received_effort_control;

  int simfreqmuj;

protected:
  // free or static object
  enum Object_State { STATIC = true, FREE = false };

  // get the URDF XML from the parameter server
  std::string get_urdf(std::string param_name) const;

  // setup initial sim environment
  void setup_sim_environment();

  // parse transmissions from URDF
  bool parse_transmissions(const std::string& urdf_string);

  // node handles
  ros::NodeHandle robot_node_handle;

  // strings
  std::string robot_namespace_ = "mujoco_hw_interface";
  std::string robot_description_param_;
  std::string robot_model_path_;
  int sim_frequency_mujoco_;

  // vectors
  std::vector<int>::iterator it;
  std::vector<std::string> robot_link_names_;

  // transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;


  // controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Time variables
  ros::Duration sim_period;
  ros::Time sim_time_ros;
  ros::Time sim_time_last;
  
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
