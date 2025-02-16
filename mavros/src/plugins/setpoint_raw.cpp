/**
 * @brief SetpointRAW plugin
 * @file setpoint_raw.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/AttitudeTargetOmni.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint RAW plugin
 *
 * Send position setpoints and publish current state (return loop).
 * User can decide what set of filed needed for operation via IGNORE bits.
 */
class SetpointRawPlugin : public plugin::PluginBase,
	private plugin::SetPositionTargetLocalNEDMixin<SetpointRawPlugin>,
	private plugin::SetPositionTargetGlobalIntMixin<SetpointRawPlugin>,
	private plugin::SetOmniAttitudeTargetMixin<SetpointRawPlugin>,
	private plugin::SetAttitudeTargetMixin<SetpointRawPlugin> {
public:
	SetpointRawPlugin() : PluginBase(),
		sp_nh("~setpoint_raw")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		local_sub = sp_nh.subscribe("local", 10, &SetpointRawPlugin::local_cb, this);
		global_sub = sp_nh.subscribe("global", 10, &SetpointRawPlugin::global_cb, this);
		attitude_sub = sp_nh.subscribe("attitude", 10, &SetpointRawPlugin::attitude_cb, this);
		omni_attitude_sub = sp_nh.subscribe("omni_attitude", 10, &SetpointRawPlugin::omni_attitude_cb, this);
		target_local_pub = sp_nh.advertise<mavros_msgs::PositionTarget>("target_local", 10);
		target_global_pub = sp_nh.advertise<mavros_msgs::GlobalPositionTarget>("target_global", 10);
		target_attitude_pub = sp_nh.advertise<mavros_msgs::AttitudeTarget>("target_attitude", 10);

		// Set Thrust scaling in px4_config.yaml, setpoint_raw block.
		if (!sp_nh.getParam("thrust_scaling", thrust_scaling))
		{
			ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw", "thrust_scaling parameter is unset. Attitude (and angular rate/thrust) setpoints will be ignored.");
			thrust_scaling = -1.0;
		}
	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&SetpointRawPlugin::handle_position_target_local_ned),
				make_handler(&SetpointRawPlugin::handle_position_target_global_int),
				make_handler(&SetpointRawPlugin::handle_attitude_target),
		};
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	friend class SetPositionTargetGlobalIntMixin;
	friend class SetAttitudeTargetMixin;
	friend class SetOmniAttitudeTargetMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber local_sub, global_sub, attitude_sub, omni_attitude_sub;
	ros::Publisher target_local_pub, target_global_pub, target_attitude_pub;

	double thrust_scaling;

	/* -*- message handlers -*- */
	void handle_position_target_local_ned(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_LOCAL_NED &tgt)
	{
		// Transform desired position,velocities,and accels from ENU to NED frame
		auto position = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.x, tgt.y, tgt.z));
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
		Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
		auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
		float yaw_rate = ang_vel_enu.z();

		auto target = boost::make_shared<mavros_msgs::PositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		tf::pointEigenToMsg(position, target->position);
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_local_pub.publish(target);
	}

	void handle_position_target_global_int(const mavlink::mavlink_message_t *msg, mavlink::common::msg::POSITION_TARGET_GLOBAL_INT &tgt)
	{
		// Transform desired velocities from ENU to NED frame
		auto velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.vx, tgt.vy, tgt.vz));
		auto af = ftf::transform_frame_ned_enu(Eigen::Vector3d(tgt.afx, tgt.afy, tgt.afz));
		float yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, tgt.yaw))));
		Eigen::Vector3d ang_vel_ned(0.0, 0.0, tgt.yaw_rate);
		auto ang_vel_enu = ftf::transform_frame_ned_enu(ang_vel_ned);
		float yaw_rate = ang_vel_enu.z();

		auto target = boost::make_shared<mavros_msgs::GlobalPositionTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->coordinate_frame = tgt.coordinate_frame;
		target->type_mask = tgt.type_mask;
		target->latitude = tgt.lat_int / 1e7;
		target->longitude = tgt.lon_int / 1e7;
		target->altitude = tgt.alt;
		tf::vectorEigenToMsg(velocity, target->velocity);
		tf::vectorEigenToMsg(af, target->acceleration_or_force);
		target->yaw = yaw;
		target->yaw_rate = yaw_rate;

		target_global_pub.publish(target);
	}

	void handle_attitude_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::ATTITUDE_TARGET &tgt)
	{
		// Transform orientation from baselink -> ENU
		// to aircraft -> NED
		auto orientation = ftf::transform_orientation_ned_enu(
					ftf::transform_orientation_baselink_aircraft(
						ftf::mavlink_to_quaternion(tgt.q)));

		auto body_rate = ftf::transform_frame_baselink_aircraft(Eigen::Vector3d(tgt.body_roll_rate, tgt.body_pitch_rate, tgt.body_yaw_rate));

		auto target = boost::make_shared<mavros_msgs::AttitudeTarget>();

		target->header.stamp = m_uas->synchronise_stamp(tgt.time_boot_ms);
		target->type_mask = tgt.type_mask;
		tf::quaternionEigenToMsg(orientation, target->orientation);
		tf::vectorEigenToMsg(body_rate, target->body_rate);
		target->thrust = tgt.thrust;

		target_attitude_pub.publish(target);
	}

	/* -*- callbacks -*- */

	void local_cb(const mavros_msgs::PositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d position, velocity, af;
		float yaw, yaw_rate;

		tf::pointMsgToEigen(req->position, position);
		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		if (req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_NED || req->coordinate_frame == mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED) {
			position = ftf::transform_frame_baselink_aircraft(position);
			velocity = ftf::transform_frame_baselink_aircraft(velocity);
			af = ftf::transform_frame_baselink_aircraft(af);
			yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_absolute_frame_aircraft_baselink(
							ftf::quaternion_from_rpy(0.0, 0.0, req->yaw)));
		} else {
			position = ftf::transform_frame_enu_ned(position);
			velocity = ftf::transform_frame_enu_ned(velocity);
			af = ftf::transform_frame_enu_ned(af);
			yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
		}

		Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
		auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
		yaw_rate = ang_vel_ned.z();

		set_position_target_local_ned(
					req->header.stamp.toNSec() / 1000000,
					req->coordinate_frame,
					req->type_mask,
					position,
					velocity,
					af,
					yaw, yaw_rate);
	}

	void global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr &req)
	{
		Eigen::Vector3d velocity, af;
		float yaw, yaw_rate;

		tf::vectorMsgToEigen(req->velocity, velocity);
		tf::vectorMsgToEigen(req->acceleration_or_force, af);

		// Transform frame ENU->NED
		velocity = ftf::transform_frame_enu_ned(velocity);
		af = ftf::transform_frame_enu_ned(af);
		yaw = ftf::quaternion_get_yaw(
					ftf::transform_orientation_aircraft_baselink(
						ftf::transform_orientation_ned_enu(
							ftf::quaternion_from_rpy(0.0, 0.0, req->yaw))));
		Eigen::Vector3d ang_vel_enu(0.0, 0.0, req->yaw_rate);
		auto ang_vel_ned = ftf::transform_frame_ned_enu(ang_vel_enu);
		yaw_rate = ang_vel_ned.z();

		set_position_target_global_int(
					req->header.stamp.toNSec() / 1000000,
					req->coordinate_frame,
					req->type_mask,
					req->latitude * 1e7,
					req->longitude * 1e7,
					req->altitude,
					velocity,
					af,
					yaw, yaw_rate);
	}

	void attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr &req)
	{
		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d baselink_angular_rate;
		Eigen::Vector3d body_rate;
		double thrust;

		// ignore thrust is false by default, unless no thrust scaling is set or thrust is zero
		auto ignore_thrust = req->thrust != 0.0 && thrust_scaling < 0.0;

		if (ignore_thrust) {
			// I believe it's safer without sending zero thrust, but actually ignoring the actuation.
			ROS_FATAL_THROTTLE_NAMED(5, "setpoint_raw", "Recieved thrust, but ignore_thrust is true: "
				"the most likely cause of this is a failure to specify the thrust_scaling parameters "
				"on px4/apm_config.yaml. Actuation will be ignored.");
			return;
		} else {
			if (thrust_scaling == 0.0) {
				ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw", "thrust_scaling parameter is set to zero.");
			}
			thrust = std::min(1.0, std::max(0.0, req->thrust * thrust_scaling));
		}

		// Take care of attitude setpoint
		desired_orientation = ftf::to_eigen(req->orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(desired_orientation));

		body_rate = ftf::transform_frame_baselink_aircraft(
			ftf::to_eigen(req->body_rate));

		set_attitude_target(
					req->header.stamp.toNSec() / 1000000,
					req->type_mask,
					ned_desired_orientation,
					body_rate,
					thrust);

	}

	void omni_attitude_cb(const mavros_msgs::AttitudeTargetOmni::ConstPtr &req)
	{
		Eigen::Quaterniond desired_orientation;
		Eigen::Vector3d baselink_angular_rate;
		Eigen::Vector3d body_rate;
		Eigen::Vector3d thrust;

		// ignore thrust is false by default, unless no thrust scaling is set or thrust is zero
		auto ignore_thrust = req->thrust.z != 0.0 && thrust_scaling < 0.0;

		if (ignore_thrust) {
			// I believe it's safer without sending zero thrust, but actually ignoring the actuation.
			ROS_FATAL_THROTTLE_NAMED(5, "setpoint_raw", "Recieved thrust, but ignore_thrust is true: "
				"the most likely cause of this is a failure to specify the thrust_scaling parameters "
				"on px4/apm_config.yaml. Actuation will be ignored.");
			return;
		} else {
			if (thrust_scaling == 0.0) {
				ROS_WARN_THROTTLE_NAMED(5, "setpoint_raw", "thrust_scaling parameter is set to zero.");
			}
			thrust[0] = req->thrust.x * thrust_scaling;
			thrust[1] = req->thrust.y * thrust_scaling;
			thrust[2] = std::min(1.0, std::max(0.0, req->thrust.z * thrust_scaling));
		}

		// Take care of attitude setpoint
		desired_orientation = ftf::to_eigen(req->orientation);

		// Transform desired orientation to represent aircraft->NED,
		// MAVROS operates on orientation of base_link->ENU
		auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(desired_orientation));

		body_rate = ftf::transform_frame_baselink_aircraft(
			ftf::to_eigen(req->body_rate));

		set_omni_attitude_target(
					req->header.stamp.toNSec() / 1000000,
					req->type_mask,
					ned_desired_orientation,
					body_rate,
					thrust);

	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointRawPlugin, mavros::plugin::PluginBase)
