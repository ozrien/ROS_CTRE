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

/* Original Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the CTRERobot
           For a more detailed simulation example, see sim_hw_interface.cpp
*/

#include <cmath>
#include <iostream>
#include <math.h>
#include <thread>

// ROS message types
#include "ros_control_boilerplate/ctrerobot_hw_interface.h"

#include <ctre/phoenix/MotorControl/SensorCollection.h>
#include <ctre/phoenix/Platform/CANProtocol.h>
#include <ctre/phoenix/Platform/SysWatchdog.h>

namespace ctrerobot_control
{
const int pidIdx = 0; //0 for primary closed-loop, 1 for cascaded closed-loop
const int timeoutMs = 0; //If nonzero, function will wait for config success and report an error if it times out. If zero, no blocking or checking is performed

CTRERobotHWInterface::CTRERobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::CTRERobotInterface(nh, urdf_model)
{
    enable_ = nh.subscribe("joy", 1, &CTRERobotHWInterface::enableJoy, this);
}

CTRERobotHWInterface::~CTRERobotHWInterface()
{
}






void CTRERobotHWInterface::init(const char * interface)
{
    ROS_ERROR("IN INIT");
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	CTRERobotInterface::init();

    if(interface != "auto") {
        ctre::phoenix::platform::can::CANProtocol::GetInstance(interface);
    }


    for (size_t i = 0; i < num_can_talon_srxs_; i++)
	{
		ROS_INFO_STREAM_NAMED("ctrerobot_hw_interface",
							  "Loading joint " << i << "=" << can_talon_srx_names_[i] <<
							  " as CAN id " << can_talon_srx_can_ids_[i]);
		can_talons_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_talon_srx_can_ids_[i]));
		can_talons_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
		ROS_INFO_STREAM_NAMED("ctrerobot_hw_interface",
							  "\tTalon SRX firmware version " << can_talons_[i]->GetFirmwareVersion());
	}

	ROS_INFO_NAMED("ctrerobot_hw_interface", "CTRERobotHWInterface Ready.");
}
void CTRERobotHWInterface::enableJoy(const sensor_msgs::Joy& joy)
{
    //ROS_WARN_STREAM("b4: " << joy.buttons[4] << " stamp sync: " << (ros::Time::now() - joy.header.stamp));
    if(joy.buttons[4] && ((ros::Time::now() - joy.header.stamp) < ros::Duration(0.1))) { //TODO: fix
        ctre::phoenix::platform::SysWatchdog::GetInstance().Feed(100);
    }
}

void CTRERobotHWInterface::read(ros::Duration &/*elapsed_time*/)
{
    for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		auto &talon = can_talons_[joint_id];
		if (!talon) // skip unintialized Talons
			continue;

		auto &ts = talon_state_[joint_id];

		// read position and velocity from can_talons_[joint_id]
		// convert to whatever units make sense
		const hardware_interface::FeedbackDevice encoder_feedback = ts.getEncoderFeedback();
		const hardware_interface::TalonMode talon_mode = ts.getTalonMode();
		if (talon_mode == hardware_interface::TalonMode_Follower)
			continue;

		const int encoder_ticks_per_rotation = ts.getEncoderTicksPerRotation();
		const double conversion_factor = ts.getConversionFactor();

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity)* conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;

        ctre::phoenix::motion::MotionProfileStatus talon_status;
		safeTalonCall(talon->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

		hardware_interface::MotionProfileStatus internal_status;
		internal_status.topBufferRem = talon_status.topBufferRem;
		internal_status.topBufferCnt = talon_status.topBufferCnt;
		internal_status.btmBufferCnt = talon_status.btmBufferCnt;
		internal_status.hasUnderrun = talon_status.hasUnderrun;
		internal_status.isUnderrun = talon_status.isUnderrun;
		internal_status.activePointValid = talon_status.activePointValid;
		internal_status.isLast = talon_status.isLast;
		internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
		internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
		internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
		internal_status.timeDurMs = talon_status.timeDurMs;
		ts.setMotionProfileStatus(internal_status);
		
        const double position = talon->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorPosition");
		ts.setPosition(position);

		const double speed = talon->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeTalonCall(talon->GetLastError(), "GetSelectedSensorVelocity");
		ts.setSpeed(speed);

		const double output_current = talon->GetOutputCurrent();
		safeTalonCall(talon->GetLastError(), "GetOutputCurrent");
		ts.setOutputCurrent(output_current);

		const double bus_voltage = talon->GetBusVoltage();
		safeTalonCall(talon->GetLastError(), "GetBusVoltage");
		ts.setBusVoltage(bus_voltage);

		const double motor_output_percent = talon->GetMotorOutputPercent();
		safeTalonCall(talon->GetLastError(), "GetMotorOutputPercent");
		ts.setMotorOutputPercent(motor_output_percent);

		const double output_voltage = talon->GetMotorOutputVoltage();
		safeTalonCall(talon->GetLastError(), "GetMotorOutputVoltage");
		ts.setOutputVoltage(output_voltage);

		const double temperature = talon->GetTemperature(); //returns in Celsius
		safeTalonCall(talon->GetLastError(), "GetTemperature");
		ts.setTemperature(temperature);

		
		const double closed_loop_error = talon->GetClosedLoopError(pidIdx) * closed_loop_scale;
		safeTalonCall(talon->GetLastError(), "GetClosedLoopError");
		ts.setClosedLoopError(closed_loop_error);
		const double integral_accumulator = talon->GetIntegralAccumulator(pidIdx) * closed_loop_scale;
		safeTalonCall(talon->GetLastError(), "GetIntegralAccumulator");
		ts.setIntegralAccumulator(integral_accumulator);

		const double error_derivative = talon->GetErrorDerivative(pidIdx) * closed_loop_scale;
		safeTalonCall(talon->GetLastError(), "GetErrorDerivative");
		ts.setErrorDerivative(error_derivative);

		const double closed_loop_target = talon->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
		safeTalonCall(talon->GetLastError(), "GetClosedLoopTarget");
		ts.setClosedLoopTarget(closed_loop_target);

		const double active_trajectory_position = talon->GetActiveTrajectoryPosition() * radians_scale;
		safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryPosition");
		ts.setActiveTrajectoryPosition(active_trajectory_position);
		const double active_trajectory_velocity = talon->GetActiveTrajectoryVelocity() * radians_per_second_scale;
		safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryVelocity");
		ts.setActiveTrajectoryVelocity(active_trajectory_velocity);
		const double active_trajectory_heading = talon->GetActiveTrajectoryHeading() * 2.*M_PI / 360.; //returns in degrees
		safeTalonCall(talon->GetLastError(), "GetActiveTrajectoryHeading");
		ts.setActiveTrajectoryHeading(active_trajectory_heading);
		ts.setMotionProfileTopLevelBufferCount(talon->GetMotionProfileTopLevelBufferCount());

		safeTalonCall(talon->GetLastError(), "IsMotionProfileTopLevelBufferFull");

		ctre::phoenix::motorcontrol::Faults faults;
		safeTalonCall(talon->GetFaults(faults), "GetFaults");
		ts.setFaults(faults.ToBitfield());

		// Grab limit switch and softlimit here
		auto sensor_collection = talon->GetSensorCollection();
		ts.setForwardLimitSwitch(sensor_collection.IsFwdLimitSwitchClosed());
		ts.setReverseLimitSwitch(sensor_collection.IsRevLimitSwitchClosed());

		ts.setForwardSoftlimitHit(faults.ForwardSoftLimit);
		ts.setReverseSoftlimitHit(faults.ReverseSoftLimit);
		
        ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
		safeTalonCall(talon->GetStickyFaults(sticky_faults), "GetStickyFaults");
		ts.setStickyFaults(sticky_faults.ToBitfield());
	}
}

double CTRERobotHWInterface::getConversionFactor(int encoder_ticks_per_rotation,
						hardware_interface::FeedbackDevice encoder_feedback,
						hardware_interface::TalonMode talon_mode)
{
	if(talon_mode == hardware_interface::TalonMode_Position)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else if(talon_mode == hardware_interface::TalonMode_Velocity)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2 * M_PI / encoder_ticks_per_rotation / .1;
			case hardware_interface::FeedbackDevice_Analog:
				return 2 * M_PI / 1024 / .1;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device. Unable to convert units.");
				return 1.;
		}
	}
	else
	{
		//ROS_WARN_STREAM("Unable to convert closed loop units.");
		return 1.;
	}
}

bool CTRERobotHWInterface::safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name)
{
	std::string error_name;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull/CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
 
		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "case";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
			break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
	//ROS_ERROR_STREAM("Error calling " << talon_method_name << " : " << error_name);
	return false;
}

void CTRERobotHWInterface::write(ros::Duration &elapsed_time)
{
	for (std::size_t joint_id = 0; joint_id < num_can_talon_srxs_; ++joint_id)
	{
		//TODO : skip over most or all of this if the talon is in follower mode
		//       Only do the Set() call and then never do anything else?

		// Save some typing by making references to commonly
		// used variables
		auto &talon = can_talons_[joint_id];

		if (!talon) // skip unintialized Talons
			continue;

		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];
		
		hardware_interface::FeedbackDevice internal_feedback_device;
		double feedback_coefficient;

		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (tc.encoderFeedbackChanged(internal_feedback_device, feedback_coefficient) &&
			convertFeedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			//ROS_WARN("feedback");
			safeTalonCall(talon->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, timeoutMs),"ConfigSelectedFeedbackSensor");
			safeTalonCall(talon->ConfigSelectedFeedbackCoefficient(feedback_coefficient, pidIdx, timeoutMs),"ConfigSelectedFeedbackCoefficient");
 			ts.setEncoderFeedback(internal_feedback_device);
			ts.setFeedbackCoefficient(feedback_coefficient);
		}

		// Get mode that is about to be commanded
		const hardware_interface::TalonMode talon_mode = tc.getMode();
		const int encoder_ticks_per_rotation = tc.getEncoderTicksPerRotation();
		ts.setEncoderTicksPerRotation(encoder_ticks_per_rotation);

		double conversion_factor;
		if (tc.conversionFactorChanged(conversion_factor))
			ts.setConversionFactor(conversion_factor);

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode) * conversion_factor;

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
		    (talon_mode == hardware_interface::TalonMode_Velocity) ||
		    (talon_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			     (talon_mode == hardware_interface::TalonMode_MotionMagic))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);
			
			double p;
			double i;
			double d;
			double f;
			int    iz;
			int    allowable_closed_loop_error;
			double max_integral_accumulator;
			double closed_loop_peak_output;
			int    closed_loop_period;
 
			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot) || ros::Time::now().toSec() - can_talon_srx_run_profile_stop_time_[joint_id] < .2)
			{
				//ROS_WARN("PIDF");
				safeTalonCall(talon->Config_kP(slot, p, timeoutMs),"Config_kP");
				safeTalonCall(talon->Config_kI(slot, i, timeoutMs),"Config_kI");
				safeTalonCall(talon->Config_kD(slot, d, timeoutMs),"Config_kD");
				safeTalonCall(talon->Config_kF(slot, f, timeoutMs),"Config_kF");
				safeTalonCall(talon->Config_IntegralZone(slot, iz, timeoutMs),"Config_IntegralZone");
				// TODO : Scale these two?
				safeTalonCall(talon->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error, timeoutMs),"ConfigAllowableClosedloopError");
				safeTalonCall(talon->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, timeoutMs),"ConfigMaxIntegralAccumulator");
				safeTalonCall(talon->ConfigClosedLoopPeakOutput(slot, closed_loop_peak_output, timeoutMs),"ConfigClosedLoopPeakOutput");
				safeTalonCall(talon->ConfigClosedLoopPeriod(slot, closed_loop_period, timeoutMs),"ConfigClosedLoopPeriod");

				ts.setPidfP(p, slot);
				ts.setPidfI(i, slot);
				ts.setPidfD(d, slot);
				ts.setPidfF(f, slot);
				ts.setPidfIzone(iz, slot);
				ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
				ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
				ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
				ts.setClosedLoopPeriod(closed_loop_period, slot);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" PIDF slot " << slot << " config values");
			}
 
			bool aux_pid_polarity;
			if (tc.auxPidPolarityChanged(aux_pid_polarity))
			{
				safeTalonCall(talon->ConfigAuxPIDPolarity(aux_pid_polarity, timeoutMs), "ConfigAuxPIDPolarity");
				ts.setAuxPidPolarity(aux_pid_polarity);
			}

			if (slot_changed)
			{
				//ROS_WARN("slot");
				ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);

				safeTalonCall(talon->SelectProfileSlot(slot, pidIdx),"SelectProfileSlot");
				ts.setSlot(slot);
			}
		}

		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			//ROS_WARN("invvert");
			talon->SetInverted(invert);
			safeTalonCall(talon->GetLastError(), "SetInverted");
			talon->SetSensorPhase(sensor_phase);
			safeTalonCall(talon->GetLastError(), "SetSensorPhase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" invert / phase");
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
		if (tc.neutralModeChanged(neutral_mode) &&
			convertNeutralMode(neutral_mode, ctre_neutral_mode))
		{
			//ROS_WARN("neutral2");
			talon->SetNeutralMode(ctre_neutral_mode);
			safeTalonCall(talon->GetLastError(), "SetNeutralMode");
			ts.setNeutralMode(neutral_mode);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral mode");
		}

		if (tc.neutralOutputChanged())
		{
			//ROS_WARN("neutral");
			talon->NeutralOutput();
			safeTalonCall(talon->GetLastError(), "NeutralOutput");
			ts.setNeutralOutput(true);
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" neutral output");
		}

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			//ROS_WARN("iaccum");
			safeTalonCall(talon->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, timeoutMs), "SetIntegralAccumulator");
			//The units on this aren't really right

			// Do not set talon state - this changes
			// dynamically so read it in read() above instead
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" integral accumulator");
		}

		double closed_loop_ramp;
		double open_loop_ramp;
		double peak_output_forward;
		double peak_output_reverse;
		double nominal_output_forward;
		double nominal_output_reverse;
		double neutral_deadband;
		if (tc.outputShapingChanged(closed_loop_ramp,
									open_loop_ramp,
									peak_output_forward,
									peak_output_reverse,
									nominal_output_forward,
									nominal_output_reverse,
									neutral_deadband))
		{
			//ROS_WARN("output shape");
			safeTalonCall(talon->ConfigOpenloopRamp(open_loop_ramp, timeoutMs),"ConfigOpenloopRamp");
			safeTalonCall(talon->ConfigClosedloopRamp(closed_loop_ramp, timeoutMs),"ConfigClosedloopRamp");
			safeTalonCall(talon->ConfigPeakOutputForward(peak_output_forward, timeoutMs),"ConfigPeakOutputForward");          // 100
			safeTalonCall(talon->ConfigPeakOutputReverse(peak_output_reverse, timeoutMs),"ConfigPeakOutputReverse");          // -100
			safeTalonCall(talon->ConfigNominalOutputForward(nominal_output_forward, timeoutMs),"ConfigNominalOutputForward"); // 0
			safeTalonCall(talon->ConfigNominalOutputReverse(nominal_output_reverse, timeoutMs),"ConfigNominalOutputReverse"); // 0
			safeTalonCall(talon->ConfigNeutralDeadband(neutral_deadband, timeoutMs),"ConfigNeutralDeadband");                 // 0

			ts.setOpenloopRamp(open_loop_ramp);
			ts.setClosedloopRamp(closed_loop_ramp);
			ts.setPeakOutputForward(peak_output_forward);
			ts.setPeakOutputReverse(peak_output_reverse);
			ts.setNominalOutputForward(nominal_output_forward);
			ts.setNominalOutputReverse(nominal_output_reverse);
			ts.setNeutralDeadband(neutral_deadband);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" output shaping");
		}
		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
										  v_measurement_filter,
										  v_c_enable))
		{
			//ROS_WARN("volt comp");
			safeTalonCall(talon->ConfigVoltageCompSaturation(v_c_saturation, timeoutMs),"ConfigVoltageCompSaturation");
			safeTalonCall(talon->ConfigVoltageMeasurementFilter(v_measurement_filter, timeoutMs),"ConfigVoltageMeasurementFilter");
			talon->EnableVoltageCompensation(v_c_enable);
			safeTalonCall(talon->GetLastError(), "EnableVoltageCompensation");

			ts.setVoltageCompensationSaturation(v_c_saturation);
			ts.setVoltageMeasurementFilter(v_measurement_filter);
			ts.setVoltageCompensationEnable(v_c_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" voltage compensation");
		}

		hardware_interface::VelocityMeasurementPeriod internal_v_m_period;
		ctre::phoenix::motorcontrol::VelocityMeasPeriod phoenix_v_m_period;
		int v_m_window;

		if (tc.velocityMeasurementChanged(internal_v_m_period, v_m_window) &&
			convertVelocityMeasurementPeriod(internal_v_m_period, phoenix_v_m_period))
		{
			safeTalonCall(talon->ConfigVelocityMeasurementPeriod(phoenix_v_m_period, timeoutMs),"ConfigVelocityMeasurementPeriod");
			safeTalonCall(talon->ConfigVelocityMeasurementWindow(v_m_window, timeoutMs),"ConfigVelocityMeasurementWindow");

			ts.setVelocityMeasurementPeriod(internal_v_m_period);
			ts.setVelocityMeasurementWindow(v_m_window);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" velocity measurement period / window");
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			//ROS_WARN("pos");
			safeTalonCall(talon->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, timeoutMs),
					"SetSelectedSensorPosition");
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" selected sensor position");

		}

		hardware_interface::LimitSwitchSource internal_local_forward_source;
		hardware_interface::LimitSwitchNormal internal_local_forward_normal;
		hardware_interface::LimitSwitchSource internal_local_reverse_source;
		hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
		ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
		if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
										  internal_local_reverse_source, internal_local_reverse_normal) &&
				convertLimitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				convertLimitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				convertLimitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				convertLimitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
		{
			//ROS_WARN("limit_switch");
			safeTalonCall(talon->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, timeoutMs),"ConfigForwardLimitSwitchSource");
			safeTalonCall(talon->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, timeoutMs),"ConfigReverseLimitSwitchSource");
			ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
			ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" limit switches");
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (tc.SoftLimitChanged(softlimit_forward_threshold,
				softlimit_forward_enable,
				softlimit_reverse_threshold,
				softlimit_reverse_enable,
				softlimit_override_enable))
		{
			//ROS_WARN("soft limit");
			double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
			double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
			talon->OverrideSoftLimitsEnable(softlimit_override_enable);
			safeTalonCall(talon->GetLastError(), "OverrideSoftLimitsEnable");
			safeTalonCall(talon->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, timeoutMs),"ConfigForwardSoftLimitThreshold");
			safeTalonCall(talon->ConfigForwardSoftLimitEnable(softlimit_forward_enable, timeoutMs),"ConfigForwardSoftLimitEnable");
			safeTalonCall(talon->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, timeoutMs),"ConfigReverseSoftLimitThreshold");
			safeTalonCall(talon->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, timeoutMs),"ConfigReverseSoftLimitEnable");

			ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
			ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
			ts.setForwardSoftLimitEnable(softlimit_forward_enable);
			ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
			ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" soft limits " <<
					std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
					std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
					std::endl << "\toverride_enable=" << softlimit_override_enable);
		}

		int peak_amps;
		int peak_msec;
		int continuous_amps;
		bool enable;
		if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
		{
			//ROS_WARN("cur limit");
			safeTalonCall(talon->ConfigPeakCurrentLimit(peak_amps, timeoutMs),"ConfigPeakCurrentLimit");
			safeTalonCall(talon->ConfigPeakCurrentDuration(peak_msec, timeoutMs),"ConfigPeakCurrentDuration");
			safeTalonCall(talon->ConfigContinuousCurrentLimit(continuous_amps, timeoutMs),"ConfigContinuousCurrentLimit");
			talon->EnableCurrentLimit(enable);
			safeTalonCall(talon->GetLastError(), "EnableCurrentLimit");

			ts.setPeakCurrentLimit(peak_amps);
			ts.setPeakCurrentDuration(peak_msec);
			ts.setContinuousCurrentLimit(continuous_amps);
			ts.setCurrentLimitEnable(enable);
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" peak current");
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration))
			{
				//ROS_WARN("magic changed");
				//converted from rad/sec to native units
				safeTalonCall(talon->ConfigMotionCruiseVelocity((motion_cruise_velocity / radians_per_second_scale), timeoutMs),"ConfigMotionCruiseVelocity(");
				safeTalonCall(talon->ConfigMotionAcceleration((motion_acceleration / radians_per_second_scale), timeoutMs),"ConfigMotionAcceleration(");

				ts.setMotionCruiseVelocity(motion_cruise_velocity);
				ts.setMotionAcceleration(motion_acceleration);

				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" cruise velocity / acceleration");
			}

#if 0 // DISABLE FOR NOW UNTIL WE CAN FIND A SAFE DEFAULT
			// Do this before rest of motion profile stuff
			// so it takes effect before starting a buffer?
			int motion_control_frame_period;
			if (tc.motionControlFramePeriodChanged(motion_control_frame_period))
			{
				//ROS_WARN("profile frame period");
				safeTalonCall(talon->ChangeMotionControlFramePeriod(motion_control_frame_period),"ChangeMotionControlFramePeriod");
				ts.setMotionControlFramePeriod(motion_control_frame_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion control frame period");
			}
#endif

			int motion_profile_trajectory_period;
			if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
			{
				//ROS_WARN("profile frame period");
				safeTalonCall(talon->ConfigMotionProfileTrajectoryPeriod(motion_profile_trajectory_period, timeoutMs),"ConfigMotionProfileTrajectoryPeriod");
				ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectory period");
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
			{
				//ROS_WARN("clear points");
				safeTalonCall(talon->ClearMotionProfileTrajectories(), "ClearMotionProfileTrajectories");
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
			}

			if (tc.clearMotionProfileHasUnderrunChanged())
			{
				//ROS_WARN("clear underrun");
				safeTalonCall(talon->ClearMotionProfileHasUnderrun(timeoutMs),"ClearMotionProfileHasUnderrun");
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile underrun changed");
			}

			// TODO : check that Talon motion buffer is not full
			// before writing, communicate how many have been written
			// - and thus should be cleared - from the talon_command
			// list of requests.
		}

		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;
		if (tc.motionProfileTrajectoriesChanged(trajectory_points))
		{
			//ROS_INFO_STREAM("Pre buffer");
			//ROS_WARN("point_buffer");
			//int i = 0;
			for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
			{
				ctre::phoenix::motion::TrajectoryPoint pt;
				pt.position = it->position / radians_scale;
				pt.velocity = it->velocity / radians_per_second_scale;
				pt.headingDeg = it->headingRad * 180. / M_PI;
				pt.auxiliaryPos = it->auxiliaryPos; // TODO : unit conversion?
				pt.profileSlotSelect0 = it->profileSlotSelect0;
				pt.profileSlotSelect1 = it->profileSlotSelect1;
				pt.isLastPoint = it->isLastPoint;
				pt.zeroPos = it->zeroPos;
				pt.timeDur = static_cast<ctre::phoenix::motion::TrajectoryDuration>(it->trajectoryDuration);
				safeTalonCall(talon->PushMotionProfileTrajectory(pt),"PushMotionProfileTrajectory");
				//ROS_INFO_STREAM("id: " << joint_id << " pos: " << pt.position << " i: " << i++);
			}
			//ROS_INFO_STREAM("Post buffer");
			// Copy the 1st profile trajectory point from
			// the top level buffer to the talon
			// Subsequent points will be copied by
			// the process_motion_profile_buffer_thread code
			//talon->ProcessMotionProfileBuffer();

			ROS_INFO_STREAM("Added joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" motion profile trajectories");
		}

		// Set new motor setpoint if either the mode or
		// the setpoint has been changed
		double command;
		hardware_interface::TalonMode in_mode;

		const bool b1 = tc.newMode(in_mode);
		const bool b2 = tc.commandChanged(command) || ts.getCANID() ==51 ;

		hardware_interface::DemandType demand1_type_internal;
		double demand1_value;
		const bool b3 = tc.demand1Changed(demand1_type_internal, demand1_value);

		if (b1 || b2 || b3 || ros::Time::now().toSec() - can_talon_srx_run_profile_stop_time_[joint_id] < .2)
		{
			ctre::phoenix::motorcontrol::ControlMode out_mode;
			if ((b1 || b2) && convertControlMode(in_mode, out_mode))
			{
				ts.setTalonMode(in_mode);
				ts.setSetpoint(command);

				ts.setNeutralOutput(false); // maybe make this a part of setSetpoint?

				switch (out_mode)
				{
					case ctre::phoenix::motorcontrol::ControlMode::Velocity:
						command /= radians_per_second_scale;
						break;
					case ctre::phoenix::motorcontrol::ControlMode::Position:
						command /= radians_scale;
						break;
					case ctre::phoenix::motorcontrol::ControlMode::MotionMagic:
						command /= radians_scale;
						break;
				}
			
			}

			ts.setDemand1Type(demand1_type_internal);
			ts.setDemand1Value(demand1_value);

			//ROS_INFO_STREAM c("in mode: " << in_mode);
			if (b3 &&
				(demand1_type_internal > hardware_interface::DemandType::DemandType_Neutral) &&
				(demand1_type_internal < hardware_interface::DemandType::DemandType_Last) )
			{
				ctre::phoenix::motorcontrol::DemandType demand1_type_phoenix;
				switch (demand1_type_internal)
				{
					case hardware_interface::DemandType::DemandType_AuxPID:
						demand1_type_phoenix = ctre::phoenix::motorcontrol::DemandType::DemandType_AuxPID;
						break;
					case hardware_interface::DemandType::DemandType_ArbitraryFeedForward:
						demand1_type_phoenix = ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward;
						break;
				}

				talon->Set(out_mode, command, demand1_type_phoenix, demand1_value);
			}
			else
				talon->Set(out_mode, command);

			//ROS_WARN_STREAM("set at: " << ts.getCANID() << " new mode: " << b1 << " command_changed: " << b2 << " cmd: " << command);
		}
		
		if (tc.clearStickyFaultsChanged())
		{
			//ROS_WARN("sticky");
			safeTalonCall(talon->ClearStickyFaults(timeoutMs), "ClearStickyFaults");
			ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_talon_srx_names_[joint_id] <<" sticky_faults");
		}
	}
}

// Convert from internal version of hardware mode ID
// to one to write to actual Talon hardware
// Return true if conversion is OK, false if
// an unknown mode is hit.
bool CTRERobotHWInterface::convertControlMode(
	const hardware_interface::TalonMode input_mode,
	ctre::phoenix::motorcontrol::ControlMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::TalonMode_PercentOutput:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
		break;
	case hardware_interface::TalonMode_Position:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Position;
		break;
	case hardware_interface::TalonMode_Velocity:      // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Velocity;
		break;
	case hardware_interface::TalonMode_Current:       // CloseLoop
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Current;
		break;
	case hardware_interface::TalonMode_Follower:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Follower;
		break;
	case hardware_interface::TalonMode_MotionProfile:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionProfile;
		break;
	case hardware_interface::TalonMode_MotionMagic:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::MotionMagic;
		break;
	case hardware_interface::TalonMode_Disabled:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::ControlMode::Disabled;
		ROS_WARN("Unknown mode seen in HW interface");
		return false;
	}
	return true;
}

bool CTRERobotHWInterface::convertNeutralMode(
	const hardware_interface::NeutralMode input_mode,
	ctre::phoenix::motorcontrol::NeutralMode &output_mode)
{
	switch (input_mode)
	{
	case hardware_interface::NeutralMode_EEPROM_Setting:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		break;
	case hardware_interface::NeutralMode_Coast:
		output_mode = ctre::phoenix::motorcontrol::Coast;
		break;
	case hardware_interface::NeutralMode_Brake:
		output_mode = ctre::phoenix::motorcontrol::Brake;
		break;
	default:
		output_mode = ctre::phoenix::motorcontrol::EEPROMSetting;
		ROS_WARN("Unknown neutral mode seen in HW interface");
		return false;
	}

	return true;
}

bool CTRERobotHWInterface::convertFeedbackDevice(
	const hardware_interface::FeedbackDevice input_fd,
	ctre::phoenix::motorcontrol::FeedbackDevice &output_fd)
{
	switch (input_fd)
	{
	case hardware_interface::FeedbackDevice_QuadEncoder:
		output_fd = ctre::phoenix::motorcontrol::QuadEncoder;
		break;
	case hardware_interface::FeedbackDevice_Analog:
		output_fd = ctre::phoenix::motorcontrol::Analog;
		break;
	case hardware_interface::FeedbackDevice_Tachometer:
		output_fd = ctre::phoenix::motorcontrol::Tachometer;
		break;
	case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
		output_fd = ctre::phoenix::motorcontrol::PulseWidthEncodedPosition;
		break;
	case hardware_interface::FeedbackDevice_SensorSum:
		output_fd = ctre::phoenix::motorcontrol::SensorSum;
		break;
	case hardware_interface::FeedbackDevice_SensorDifference:
		output_fd = ctre::phoenix::motorcontrol::SensorDifference;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor0:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor0;
		break;
	case hardware_interface::FeedbackDevice_RemoteSensor1:
		output_fd = ctre::phoenix::motorcontrol::RemoteSensor1;
		break;
	case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
		output_fd = ctre::phoenix::motorcontrol::SoftwareEmulatedSensor;
		break;
	default:
		ROS_WARN("Unknown feedback device seen in HW interface");
		return false;
	}
	return true;
}

bool CTRERobotHWInterface::convertLimitSwitchSource(
	const hardware_interface::LimitSwitchSource input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchSource_FeedbackConnector:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector;
		break;
	case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteTalonSRX;
		break;
	case hardware_interface::LimitSwitchSource_RemoteCANifier:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_RemoteCANifier;
		break;
	case hardware_interface::LimitSwitchSource_Deactivated:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
		break;
	default:
		ROS_WARN("Unknown limit switch source seen in HW interface");
		return false;
	}
	return true;
}

bool CTRERobotHWInterface::convertLimitSwitchNormal(
	const hardware_interface::LimitSwitchNormal input_ls,
	ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls)
{
	switch (input_ls)
	{
	case hardware_interface::LimitSwitchNormal_NormallyOpen:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen;
		break;
	case hardware_interface::LimitSwitchNormal_NormallyClosed:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed;
		break;
	case hardware_interface::LimitSwitchNormal_Disabled:
		output_ls = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
		break;
	default:
		ROS_WARN("Unknown limit switch normal seen in HW interface");
		return false;
	}
	return true;

}

bool CTRERobotHWInterface::convertVelocityMeasurementPeriod(const hardware_interface::VelocityMeasurementPeriod input_v_m_p, ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period)
{
	switch(input_v_m_p)
	{
		case hardware_interface::VelocityMeasurementPeriod::Period_1Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_1Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_2Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_2Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_5Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_5Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_10Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_10Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_20Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_20Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_25Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_25Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_50Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_50Ms;
			break;
		case hardware_interface::VelocityMeasurementPeriod::Period_100Ms:
			output_v_m_period = ctre::phoenix::motorcontrol::VelocityMeasPeriod::Period_100Ms;
			break;
		default:
			ROS_WARN("Unknown velocity measurement period seen in HW interface");
			return false;
	}
	return true;
}

} // namespace 
