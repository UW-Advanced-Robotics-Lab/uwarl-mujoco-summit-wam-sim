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
* @file   mujoco_ros_control.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
* 
* Current version specifically for UWARL
* Last edit: Nov 28, 2023 (Tim van Meijel)
*
**/


#include <boost/bind.hpp>
#include <mujoco_ros_control/mujoco_ros_control.h>
#include <urdf/model.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <vector>
#include <iostream>

namespace mujoco_ros_control
{
MujocoRosControl::MujocoRosControl()
{
}

MujocoRosControl::~MujocoRosControl()
{

}

bool MujocoRosControl::init(ros::NodeHandle &nodehandle)
{
      // Check that ROS has been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Unable to initialize Mujoco node.");
        return false;
    }

    ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control_node node in namespace: %s", robot_namespace_.c_str());


    if (nodehandle.getParam("sim_frequency_mujoco", sim_frequency_mujoco_))
    {
      // ROS_INFO("Got param: %s", sim_frequency_mujoco_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'sim_frequency'");
    }

    simfreqmuj = sim_frequency_mujoco_;

    // read urdf from ros parameter server then setup actuators and mechanism control node.
    if (nodehandle.getParam("mujoco_ros_control_node/robot_description_param", robot_description_param_))
    {
      ROS_INFO("Got param Robot description: %s", robot_description_param_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_description_param'");
    }

    const std::string urdf_string = get_urdf(robot_description_param_);

    if (!parse_transmissions(urdf_string))
    {
      ROS_ERROR_NAMED("mujoco_ros_control", "Error parsing URDF in mujoco_ros_control node, node not active.\n");
      return false;
    }

    if (nodehandle.getParam("mujoco_ros_control_node/robot_model_path", robot_model_path_))
    {
      // ROS_INFO("Got param: %s", robot_model_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_model_path'");
    }

    char error[1000];

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin>
          ("uwarl-mujoco-summit-wam-sim", "mujoco_ros_control::RobotHWSimPlugin"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHWSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    // get robot links from urdf
    std::map<std::string, std::shared_ptr<urdf::Link> > robot_links;
    robot_links = urdf_model_ptr->links_;
    std::map<std::string, std::shared_ptr<urdf::Link> >::iterator it;
    for (it = robot_links.begin(); it != robot_links.end(); ++it)
    {
      robot_link_names_.push_back(it->first);
    }


    ROS_INFO("Initialising robot simulation interface...");
    try
    {
      if (!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, 
                                  urdf_model_ptr, transmissions_))
      {
        ROS_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot sim interface");
        return false;
      }
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Failed to initialise robot simulation interface.");
      ROS_ERROR("%s", e.what());
      return false;
    }

    // // create the controller manager
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_node_handle));
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("mujoco_ros_control" , "Failed to create robot sim interface loader: "
                             << ex.what());
    }
    ROS_INFO_NAMED("mujoco_ros_control", "Loaded mujoco_ros_control.");

    // Init more variables
    ros::Time sim_time_last = ros::Time::now();
    ros::Duration(0,1000).sleep();

    return true;
}


// get the URDF XML from the parameter server
std::string MujocoRosControl::get_urdf(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (robot_node_handle.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_node_handle.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_param_.c_str());

      robot_node_handle.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Received urdf from param server, parsing...");

  return urdf_string;
}

// get Transmissions from the URDF
bool MujocoRosControl::parse_transmissions(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

std::map<std::string, double >  MujocoRosControl::update(void)
{
    sim_time_ros = ros::Time::now();
    sim_period = sim_time_ros - sim_time_last;
    bool reset_ctrls = false;
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrls);

    robot_hw_sim_->write(sim_time_ros, sim_period);

    received_effort_control = *robot_hw_sim_->get_mj_data();
    sim_time_last = ros::Time::now();

    return received_effort_control;
}



void MujocoRosControl::readCallback_mujoco(const sensor_msgs::JointState::ConstPtr& msg)
{
    list_mj_data.clear();
    std::vector<double> jointdata;
    std::string name; 
    for (size_t i = 0; i < msg->position.size(); ++i) 
    {
        jointdata.clear();
        name = msg->name[i];
        jointdata.push_back(msg->position[i]);
        jointdata.push_back(msg->velocity[i]);
        jointdata.push_back(msg->effort[i]);
        
        // Store the joint value in the map
        list_mj_data.insert(std::pair<std::string, std::vector<double> >(name, jointdata));
    } 
    robot_hw_sim_->pass_mj_data(&list_mj_data);
}

}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujoco_ros_control_node");

    ros::NodeHandle nh_;

    mujoco_ros_control::MujocoRosControl mujoco_ros_control_1;

    // create publisher to publish command signals for mujoco
    ros::Publisher pub = nh_.advertise<sensor_msgs::JointState>("/mujoco/ros_control/effort_commands", 1);

    std::map<std::string, double > received_effort_control_pub;

    std::map<std::string, std::vector<double> > list_mj_data;

    // ROS_INFO("Subscribing to joint_states");

    ros::Subscriber listener_mujoco = nh_.subscribe("/mujoco/joint_states", 1, &mujoco_ros_control::MujocoRosControl::readCallback_mujoco, &mujoco_ros_control_1);

    ROS_INFO("Subscribed to joint_states");

    ros::spinOnce();

    // initialize mujoco stuff
    if (!mujoco_ros_control_1.init(nh_))
    {
      ROS_ERROR("Could not initialise mujoco.");
      return 1;
    }

    // spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate r(mujoco_ros_control_1.simfreqmuj);

    sensor_msgs::JointState effort_pub;

    // run main loop, target real-time simulation and 60 fps rendering
    while ( ros::ok())
    {
      effort_pub.effort.clear();
      effort_pub.name.clear();

      // update:
      received_effort_control_pub = mujoco_ros_control_1.update();

      for (auto& x : received_effort_control_pub)
      {
        effort_pub.effort.push_back(x.second);
        effort_pub.name.push_back(x.first);
      }

      pub.publish(effort_pub);

      r.sleep();
    }
    return 0;
}
