/**
 * @brief SetpointOmniAttitude plugin
 * @file setpoint_omni_attitude.cpp
 * @author Li Zhengchen <lizc@hnu.edu.cn>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ThrustOmni.h>

namespace mavros {
namespace std_plugins {

using SyncPoseThrustOmniPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, mavros_msgs::ThrustOmni>;
using SyncTwistThrustOmniPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, mavros_msgs::ThrustOmni>;
using SyncPoseThrustOmni = message_filters::Synchronizer<SyncPoseThrustOmniPolicy>;
using SyncTwistThrustOmni = message_filters::Synchronizer<SyncTwistThrustOmniPolicy>;

/**
 * @brief Setpoint omni attitude plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointOmniAttitudePlugin : public plugin::PluginBase,
	private plugin::SetOmniAttitudeTargetMixin<SetpointOmniAttitudePlugin>,
	private plugin::TF2ListenerMixin<SetpointOmniAttitudePlugin> {
public:
	SetpointOmniAttitudePlugin() : PluginBase(),
		sp_nh("~setpoint_omni_attitude"),
		tf_listen(false),
		tf_rate(50.0),
		use_quaternion(false),
		reverse_thrust(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// main params
		sp_nh.param("use_quaternion", use_quaternion, false);
		sp_nh.param("reverse_thrust", reverse_thrust, false);
		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "target_attitude");
		sp_nh.param("tf/rate_limit", tf_rate, 50.0);

		// thrust_omni msg subscriber to sync
		th_sub.subscribe(sp_nh, "omni_thrust", 1);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("attitude",
						"Listen to desired attitude transform "
						<< tf_frame_id << " -> " << tf_child_frame_id);

			tf2_start<mavros_msgs::ThrustOmni>("AttitudeSpTFSync", th_sub, &SetpointOmniAttitudePlugin::transform_cb);
		}
		else if (use_quaternion) {
			/**
			 * @brief Use message_filters to sync attitude and thrust msg coming from different topics
			 */
			pose_sub.subscribe(sp_nh, "omni_attitude", 1);

			/**
			 * @brief Matches messages, even if they have different time stamps,
			 * by using an adaptative algorithm <http://wiki.ros.org/message_filters/ApproximateTime>
			 */
			sync_pose.reset(new SyncPoseThrustOmni(SyncPoseThrustOmniPolicy(10), pose_sub, th_sub));
			sync_pose->registerCallback(boost::bind(&SetpointOmniAttitudePlugin::attitude_pose_cb, this, _1, _2));
		}
		else {
			twist_sub.subscribe(sp_nh, "omni_cmd_vel", 1);
			sync_twist.reset(new SyncTwistThrustOmni(SyncTwistThrustOmniPolicy(10), twist_sub, th_sub));
			sync_twist->registerCallback(boost::bind(&SetpointOmniAttitudePlugin::attitude_twist_cb, this, _1, _2));
		}
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetOmniAttitudeTargetMixin;
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	message_filters::Subscriber<mavros_msgs::ThrustOmni> th_sub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
	message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub;

	std::unique_ptr<SyncPoseThrustOmni> sync_pose;
	std::unique_ptr<SyncTwistThrustOmni> sync_twist;

	std::string tf_frame_id;
	std::string tf_child_frame_id;

	bool tf_listen;
	double tf_rate;

	bool use_quaternion;

	bool reverse_thrust;
	float normalized_thrust;

	/**
	 * @brief Function to verify if the thrust values are normalized;
	 * considers also the reversed trust values
	 */
	inline bool is_normalized(float thrust){
		if (reverse_thrust) {
			if (thrust < -1.0) {
				ROS_WARN_NAMED("attitude", "Not normalized reversed thrust! Thd(%f) < Min(%f)", thrust, -1.0);
				return false;
			}
		}
		else {
			if (thrust < 0.0) {
				ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) < Min(%f)", thrust, 0.0);
				return false;
			}
		}

		if (thrust > 1.0) {
			ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) > Max(%f)", thrust, 1.0);
			return false;
		}
		return true;
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint and thrust to FCU attitude controller
	 */
	void send_attitude_quaternion_omni(const ros::Time &stamp, const Eigen::Affine3d &tr, const Eigen::Vector3d &thrust)
	{
		/**
		 * @note RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_q_and_thrust = (7 << 0);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
					);

		set_omni_attitude_target(stamp.toNSec() / 1000000,
					ignore_all_except_q_and_thrust,
					q,
					Eigen::Vector3d::Zero(),
					thrust);
	}

	/**
	 * @brief Send angular velocity setpoint and thrust to FCU attitude controller
	 */
	void send_attitude_ang_velocity_omni(const ros::Time &stamp, const Eigen::Vector3d &ang_vel, const Eigen::Vector3d &thrust)
	{
		/**
		 * @note Q, also bits noumbering started from 1 in docs
		 */
		const uint8_t ignore_all_except_rpy = (1 << 7);

		auto av = ftf::transform_frame_ned_enu(ang_vel);

		set_omni_attitude_target(stamp.toNSec() / 1000000,
					ignore_all_except_rpy,
					Eigen::Quaterniond::Identity(),
					av,
					thrust);
	}

	/* -*- callbacks -*- */

	void transform_cb(const geometry_msgs::TransformStamped &transform, const mavros_msgs::ThrustOmni::ConstPtr &thrust_msg) {
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);
		Eigen::Vector3d tr2;
		tf::vectorMsgToEigen(thrust_msg->thrust, tr2);

		send_attitude_quaternion_omni(transform.header.stamp, tr, tr2);
	}

	void attitude_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const mavros_msgs::ThrustOmni::ConstPtr &thrust_msg) {
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(pose_msg->pose, tr);
		Eigen::Vector3d tr2;
		tf::vectorMsgToEigen(thrust_msg->thrust, tr2);

		if (is_normalized(thrust_msg->thrust.z))
			send_attitude_quaternion_omni(pose_msg->header.stamp, tr, tr2);
	}

	void attitude_twist_cb(const geometry_msgs::TwistStamped::ConstPtr &req, const mavros_msgs::ThrustOmni::ConstPtr &thrust_msg) {
		Eigen::Vector3d ang_vel;
		tf::vectorMsgToEigen(req->twist.angular, ang_vel);
		Eigen::Vector3d tr2;
		tf::vectorMsgToEigen(thrust_msg->thrust, tr2);

		if (is_normalized(thrust_msg->thrust.z))
			send_attitude_ang_velocity_omni(req->header.stamp, ang_vel, tr2);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointOmniAttitudePlugin, mavros::plugin::PluginBase)
