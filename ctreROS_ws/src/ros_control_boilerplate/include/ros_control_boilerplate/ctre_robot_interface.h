/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface that performs a perfect control loop for
   simulation
*/

#pragma once
#include <thread>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS Controls
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <talon_interface/talon_command_interface.h>

namespace ros_control_boilerplate
{
/// \brief Hardware interface for a robot
class CTRERobotInterface : public hardware_interface::RobotHW
{
	public:
		/**
		 * \brief Constructor
		 * \param nh - Node handle for topics.
		 * \param urdf - optional pointer to a parsed robot model
		 */
		CTRERobotInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

		/** \brief Destructor */
		virtual ~CTRERobotInterface() {}

		/** \brief Initialize the hardware interface */
		virtual void init();

		/** \brief Read the state from the robot hardware. */
		virtual void read(ros::Duration &elapsed_time) = 0;

		/** \brief Write the command to the robot hardware. */
		virtual void write(ros::Duration &elapsed_time) = 0;

		/** \brief Set all members to default values */
		virtual void reset();

		/**
		 * \brief Check (in non-realtime) if given controllers could be started and stopped from the
		 * current state of the RobotHW
		 * with regard to necessary hardware interface switches. Start and stop list are disjoint.
		 * This is just a check, the actual switch is done in doSwitch()
		 */
		virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							   const std::list<hardware_interface::ControllerInfo> &/*stop_list*/) const
		{
			return true;
		}

		/**
		 * \brief Perform (in non-realtime) all necessary hardware interface switches in order to start
		 * and stop the given controllers.
		 * Start and stop list are disjoint. The feasability was checked in canSwitch() beforehand.
		 */
		virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &/*start_list*/,
							  const std::list<hardware_interface::ControllerInfo> &/*stop_list*/)
		{
		}

		/** \brief Helper for debugging a joint's state */
		virtual void printState();
		std::string printStateHelper();

		/** \brief Helper for debugging a joint's command */
		std::string printCommandHelper();

	protected:
		/** \brief Get the URDF XML from the parameter server */
		virtual void loadURDF(ros::NodeHandle &nh, std::string param_name);

		// Short name of this class
		std::string name_;

		// Startup and shutdown of the internal node inside a roscpp program
		ros::NodeHandle nh_;

		// Hardware interfaces
		hardware_interface::JointStateInterface joint_state_interface_;
		hardware_interface::TalonStateInterface talon_state_interface_;

		hardware_interface::JointCommandInterface  joint_command_interface_;
		hardware_interface::PositionJointInterface joint_position_interface_;
		hardware_interface::VelocityJointInterface joint_velocity_interface_;
		hardware_interface::EffortJointInterface   joint_effort_interface_;
		hardware_interface::TalonCommandInterface  talon_command_interface_;

		// Configuration
		std::vector<std::string> can_talon_srx_names_;
		std::vector<int>         can_talon_srx_can_ids_;
		std::vector<double>      can_talon_srx_run_profile_stop_time_;
		std::size_t              num_can_talon_srxs_;

		urdf::Model *urdf_model_;

		// Array holding master cached state of hardware
		// resources
		std::vector<hardware_interface::TalonHWState> talon_state_;
		
        // Same as above, but for pending commands to be
		// written to the hardware
		std::vector<hardware_interface::TalonHWCommand> talon_command_;

        double robot_code_ready_;
};  // class

}  // namespace
