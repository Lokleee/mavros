/**
 * @brief HoverThrustEstimate plugin
 * @file hover_thrust_estimate.cpp
 * @author Li Zhengchen <lizc@hnu.edu.cn>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Marcel Stüttgen <stuettgen@fh-aachen.de>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/HoverThrustEstimate.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief HoverThrustEstimate plugin
 *
 * Sends estimate hover thrust to FCU controller.
 */
class HoverThrustEstimatePlugin : public plugin::PluginBase {
public:
	HoverThrustEstimatePlugin() : PluginBase(),
		nh("~hover_thrust_estimate")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		target_hover_thrust_pub = nh.advertise<mavros_msgs::HoverThrustEstimate>("target_hover_thrust", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&HoverThrustEstimatePlugin::handle_hover_thrust_target),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher target_hover_thrust_pub;

	/* -*- rx handlers -*- */

	void handle_hover_thrust_target(const mavlink::mavlink_message_t *msg, mavlink::common::msg::HOVER_THRUST_ESTIMATE &hover_thrust_target)
	{
		auto hover_thrust_target_msg = boost::make_shared<mavros_msgs::HoverThrustEstimate>();
		hover_thrust_target_msg->header.stamp = m_uas->synchronise_stamp(hover_thrust_target.time_boot_ms);
		hover_thrust_target_msg->thrust = hover_thrust_target.thrust;
		target_hover_thrust_pub.publish(hover_thrust_target_msg);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::HoverThrustEstimatePlugin, mavros::plugin::PluginBase)
