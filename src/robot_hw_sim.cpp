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
* @file   robot_hw_sim.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
* 
* Current version specifically for UWARL
* Last edit: Nov 28, 2023 (Tim van Meijel)
*
**/

#include <mujoco_ros_control/robot_hw_sim.h>
#include <urdf/model.h>
#include <string>
#include <algorithm>
#include <vector>
#include <limits>
#include <utility>
#include <list>
#include <vector>
#include <map>
#include <std_msgs/String.h>


namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace mujoco_ros_control
{

RobotHWSim::RobotHWSim()
{
}


bool RobotHWSim::init_sim(
    const std::string& robot_namespace,
    ros::NodeHandle robot_nh,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions_info)
{

  const ros::NodeHandle joint_limit_nh(robot_nh);


  for (auto& transmission_info : transmissions_info)
  {
    transmissions_.push_back(TransmissionData());
    TransmissionData& transmission = transmissions_.back();
    transmission.name = transmission_info.name_;
    for (auto& joint_info : transmission_info.joints_)
    {
      joints_.insert(std::pair<std::string, JointData>(joint_info.name_, JointData()));
      JointData& joint = joints_.at(joint_info.name_);
      joint.name = joint_info.name_;
      transmission.joint_names.push_back(joint.name);

      joint.position = 1.0;
      joint.velocity = 0.0;
      joint.effort = 1.0;  // N/m for continuous joints
      joint.effort_command = 0.0;
      joint.position_command = 0.0;
      joint.velocity_command = 0.0;
      joint.hardware_interfaces = joint_info.hardware_interfaces_;

      if (joint.hardware_interfaces.empty())
      {
        ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << joint.name <<
          " of transmission " << transmission_info.name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
      }
      else if (joint.hardware_interfaces.size() > 1)
      {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << joint_info.name_ <<
        " of transmission " << transmission_info.name_ << " specifies multiple hardware interfaces. " <<
        "Currently the robot hardware simulation interface only supports one. Using the first entry");
      }

      if (joint.hardware_interfaces.front() == "EffortJointInterface" ||
          joint.hardware_interfaces.front() == "PositionJointInterface" ||
          joint.hardware_interfaces.front() == "VelocityJointInterface")
      {
        ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" <<
                        joint.hardware_interfaces.front() << "' within the <hardwareInterface> tag in joint '" <<
                        joint.name << "'.");
        joint.hardware_interfaces.front().insert(0, "hardware_interface/");
      }
      if (!transmission_info.actuators_[0].hardware_interfaces_.empty())
      {
        ROS_WARN_STREAM_NAMED("robot_hw_sim", "The <hardware_interface> element of tranmission " <<
          transmission.name << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will not be loaded.");
      }
    }
  }

 
  for (auto& transmission : transmissions_)
  {
    for (auto& joint_name : transmission.joint_names)
    {
      JointData& joint = joints_.at(joint_name);

      // Debug
      ROS_DEBUG_STREAM_NAMED("robot_hw_sim", "Loading joint '" << joint.name << "' of type '" <<
                             joint.hardware_interfaces.front() << "'.");

      // Create joint state interface for all joints
      ROS_DEBUG("Registered joint %s with position address %p.", joint.name.c_str(), &joint.position);
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint.name, &joint.position, &joint.velocity, &joint.effort));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle;
      if (joint.hardware_interfaces.front() == "hardware_interface/EffortJointInterface")
      {
        // Create effort joint interface
        joint.control_method = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.effort_command);
        ej_interface_.registerHandle(joint_handle);
      }
      else if (joint.hardware_interfaces.front() == "hardware_interface/PositionJointInterface")
      {
        // Create position joint interface
        joint.control_method = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.position_command);
        pj_interface_.registerHandle(joint_handle);
      }
      else if (joint.hardware_interfaces.front() == "hardware_interface/VelocityJointInterface")
      {
        // Create velocity joint interface
        joint.control_method = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.velocity_command);
        vj_interface_.registerHandle(joint_handle);
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
          << joint.hardware_interfaces.front() << "' while loading interfaces for " << joint.name);
        return false;
      }

      

      register_joint_limits(joint.name, joint_handle, joint.control_method,
                            joint_limit_nh, urdf_model,
                            &joint.type, &joint.lower_limit, &joint.upper_limit,
                            &joint.effort_limit);
  
      if (joint.control_method != EFFORT)
      {
        // Initialize the PID controller. If no PID gain values are found
        const ros::NodeHandle nh(robot_nh, "/mujoco_ros_control/pid_gains/" +
                                joint.name);
        if (joint.pid_controller.init(nh, true))
        {
          switch (joint.control_method)
          {
            case POSITION:
              joint.control_method = POSITION_PID;
              break;
            case VELOCITY:
              joint.control_method = VELOCITY_PID;
              break;
          }
        }
        else
        {
        }
      }
    }
    if (transmission.joint_names.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Transmission " << transmission.name
        << " has no associated joints.");
    }
    ROS_DEBUG("%s", transmission.to_string().c_str());
  }


  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  return true;
}


void RobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  // This function is reduntant
}

std::map<std::string, double>* RobotHWSim::get_mj_data(void)
{
  return &effort_control;
}

void RobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // write the control signals to mujoco data
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);


  effort_control.clear();
  for (auto& joint_item : joints_)
    {
      JointData& joint = joint_item.second;

        switch (joint.control_method)
        {
          case EFFORT:
          {
            const double effort = joint.effort_command;
            effort_control.insert(std::pair<std::string, double > (joint.name, effort));
            break;
          }

          case POSITION:
          {
            break;
          }

          case POSITION_PID:
          {
            double error;
            error = joint.position_command - joint.position;
            const double effort_limit = joint.effort_limit;
            const double effort = clamp(joint.pid_controller.computeCommand(error, period),
                                        -effort_limit, effort_limit);
            effort_control.insert(std::pair<std::string, double > (joint.name, effort));
            break;
          }

          case VELOCITY:
          {
            break;
          }

          case VELOCITY_PID:
          {
            double error;
            error = joint.velocity_command - joint.velocity;
            const double effort_limit = joint.effort_limit;
            const double effort = clamp(joint.pid_controller.computeCommand(error, period),
                                        -effort_limit, effort_limit);
            effort_control.insert(std::pair<std::string, double > (joint.name, effort));
            break;
          }
        }
    }
}

void RobotHWSim::pass_mj_data(std::map<std::string, std::vector <double> > *list_joints_mj)
{
  std::map<std::string, std::vector <double> > joints_mujoco = *list_joints_mj;


  for (auto& joint_item : joints_)
  {
    JointData& joint = joint_item.second;
    JointData joint_mujoco; 
    joint_mujoco.name = joint.name;

    try
    {
      joint_mujoco.position = joints_mujoco.at(joint.name).at(0);
      joint_mujoco.velocity = joints_mujoco.at(joint.name).at(1);
      joint_mujoco.effort = joints_mujoco.at(joint.name).at(2);

      joint.effort = joint_mujoco.effort;
      // }
      if (joint.type == urdf::Joint::PRISMATIC)
      {
        joint.position = joint_mujoco.position;
      }
      else
      {
        joint.position += angles::shortest_angular_distance(joint.position, joint_mujoco.position);
      }
      joint.velocity = joint_mujoco.velocity;
    }
    catch(const std::exception& e)
    {
    }
  }
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void RobotHWSim::register_joint_limits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}


}  // namespace mujoco_ros_control

std::string to_string(double x)
{
    std::ostringstream ss;
    ss << x;
    return ss.str();
}

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::RobotHWSim, mujoco_ros_control::RobotHWSimPlugin)
