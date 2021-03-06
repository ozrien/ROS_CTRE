/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
   Desc:   Helper ros_control hardware interface that loads configurations
*/

#include <ros_control_boilerplate/ctre_robot_interface.h>
#include <limits>

namespace ros_control_boilerplate
{
CTRERobotInterface::CTRERobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) :
   	  name_("generic_hw_interface")
	, nh_(nh)
	, num_can_talon_srxs_(0)
{
	// Check if the URDF model needs to be loaded
	if (urdf_model == NULL)
		loadURDF(nh, "robot_description");
	else
		urdf_model_ = urdf_model;

	// Load rosparams
	ros::NodeHandle rpnh(nh_, "hardware_interface"); // TODO(davetcoleman): change the namespace to "ctre_robot_interface" aka name_

	// Read a list of joint information from ROS parameters.  Each entry in the list
	// specifies a name for the joint and a hardware ID corresponding
	// to that value.  Joint types and locations are specified (by name)
	// in a URDF file loaded along with the controller.
	XmlRpc::XmlRpcValue joint_param_list;
	if (!rpnh.getParam("joints", joint_param_list))
		throw std::runtime_error("No joints were specified.");
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
		if (!joint_params.hasMember("name"))
			throw std::runtime_error("A joint name was not specified");
		XmlRpc::XmlRpcValue &xml_joint_name = joint_params["name"];
		if (!xml_joint_name.valid() ||
				xml_joint_name.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint name was specified (expecting a string)");
		const std::string joint_name = xml_joint_name;

		if (!joint_params.hasMember("type"))
			throw std::runtime_error("A joint type was not specified");
		XmlRpc::XmlRpcValue &xml_joint_type = joint_params["type"];
		if (!xml_joint_type.valid() ||
				xml_joint_type.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint type was specified (expecting a string).");
		const std::string joint_type = xml_joint_type;

		if (joint_type == "can_talon_srx")
		{
			if (!joint_params.hasMember("can_id"))
				throw std::runtime_error("A CAN Talon SRX can_id was not specified");
			XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
			if (!xml_can_id.valid() ||
					xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
				throw std::runtime_error("An invalid joint can_id was specified (expecting an int).");
			const int can_id = xml_can_id;

			can_talon_srx_names_.push_back(joint_name);
			can_talon_srx_can_ids_.push_back(can_id);
		}
		else
		{
			std::stringstream s;
			s << "Unknown joint type " << joint_type << " specified";
			throw std::runtime_error(s.str());
		}
	}
}

void CTRERobotInterface::init()
{
	num_can_talon_srxs_ = can_talon_srx_names_.size();
	// Create vectors of the correct size for
	// talon HW state and commands
	talon_command_.resize(num_can_talon_srxs_);

	// Loop through the list of joint names
	// specified as params for the hardware_interface.
	// For each of them, create a Talon object. This
	// object is used to send and recieve commands
	// and status to/from the physical Talon motor
	// controller on the robot.  Use this pointer
	// to initialize each Talon with various params
	// set for that motor controller in config files.
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		ROS_INFO_STREAM_NAMED(name_, "CTRERobotHWInterface: Registering Talon Interface for " << can_talon_srx_names_[i] << " at hw ID " << can_talon_srx_can_ids_[i]);

		// Create joint state interface
		// Also register as JointStateInterface so that legacy
		// ROS code which uses that object type can
		// access basic state info from the talon
		// Code which needs more specific status should
		// get a TalonStateHandle instead.
		talon_state_.push_back(hardware_interface::TalonHWState(can_talon_srx_can_ids_[i]));
	}
	for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		// Create state interface for the given Talon
		// and point it to the data stored in the
		// corresponding talon_state array entry
		hardware_interface::TalonStateHandle tsh(can_talon_srx_names_[i], &talon_state_[i]);
		talon_state_interface_.registerHandle(tsh);

		can_talon_srx_run_profile_stop_time_.push_back(0);
		// Do the same for a command interface for
		// the same talon
		hardware_interface::TalonCommandHandle tch(tsh, &talon_command_[i]);
		talon_command_interface_.registerHandle(tch);
	}

	// Add a flag which indicates we should signal
	// the driver station that robot code is initialized
	hardware_interface::JointStateHandle sh("robot_code_ready", &robot_code_ready_, &robot_code_ready_, &robot_code_ready_);
	joint_state_interface_.registerHandle(sh);

	hardware_interface::JointHandle ch(sh, &robot_code_ready_);
	joint_command_interface_.registerHandle(ch);
	joint_position_interface_.registerHandle(ch);
	joint_velocity_interface_.registerHandle(ch);
	joint_effort_interface_.registerHandle(ch);

	// Publish various FRC-specific data using generic joint state for now
	// For simple things this might be OK, but for more complex state
	// (e.g. joystick) it probably makes more sense to write a
	// RealtimePublisher() for the data coming in from
	// the DS
	registerInterface(&talon_state_interface_);
	registerInterface(&joint_state_interface_);
	registerInterface(&talon_command_interface_);
	registerInterface(&joint_command_interface_);
	registerInterface(&joint_position_interface_);
	registerInterface(&joint_velocity_interface_);
	registerInterface(&joint_effort_interface_); // empty for now

	ROS_INFO_STREAM_NAMED(name_, "CTRERobotInterface Ready.");
}

// Note - there are two commented-out can_talon calls here.  If
// they're put back in, make customProfileFoo() methods which
// are virtual in the ctre_robot_interface def.  For a Set()
// call, make the virtual one do nothing (like the other
// calls already). Then in the hw_interface, override it
// with a method which makes the actual talon call
// For get, not sure what to do exactly?  The hw one is obvious-
// get the value from the talon. For sim, maybe grab it from state?


void CTRERobotInterface::reset()
{
}

void CTRERobotInterface::printState()
{
	// WARNING: THIS IS NOT REALTIME SAFE
	// FOR DEBUGGING ONLY, USE AT YOUR OWN ROBOT's RISK!
	ROS_INFO_STREAM_THROTTLE(1,
							 std::endl << "State" <<
							 std::endl << printStateHelper());
}

std::string CTRERobotInterface::printStateHelper()
{
	std::stringstream ss;
	std::cout.precision(15);

	ss << "    CAN ID       position        velocity        effort" << std::endl;
	for (std::size_t i = 0; i < num_can_talon_srxs_; ++i)
	{
		ss << "j" << i << ":    " ;
		ss << talon_state_[i].getCANID() << "\t ";
		ss << std::fixed << talon_state_[i].getPosition() << "\t ";
		ss << std::fixed << talon_state_[i].getSpeed() << "\t ";
		ss << std::fixed << talon_state_[i].getOutputVoltage() << std::endl;
	}
	return ss.str();
}

std::string CTRERobotInterface::printCommandHelper()
{
	std::stringstream ss;
	std::cout.precision(15);
	ss << "    setpoint" << std::endl;
	for (std::size_t i = 0; i < num_can_talon_srxs_; ++i)
		ss << "j" << i << ": " << std::fixed << talon_command_[i].get() << std::endl;
	return ss.str();
}

void CTRERobotInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
	return;
#if 0
	std::string urdf_string;
	urdf_model_ = new urdf::Model();

	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok())
	{
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name))
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << search_param_name);
			nh.getParam(search_param_name, urdf_string);
		}
		else
		{
			ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
								  nh.getNamespace() << param_name);
			nh.getParam(param_name, urdf_string);
		}

		usleep(100000);
	}

	if (!urdf_model_->initString(urdf_string))
		ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	else
		ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
#endif
}

}  // namespace
