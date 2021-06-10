

#ifndef PATHREPORTER_RECEIVEFSM_H
#define PATHREPORTER_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_PathReporter/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_PathReporter/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include "PathReporter_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>

#include <geographic_msgs/msg/geo_path.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

namespace urn_jaus_jss_iop_PathReporter
{

class DllExport PathReporter_ReceiveFSM : public JTS::StateMachine
{
public:
	PathReporter_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~PathReporter_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();

	/// Action Methods
	virtual void sendReportPathAction(QueryPath msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportPathReporterCapabilitiesAction(QueryPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isSupported(QueryPath msg);



	PathReporter_ReceiveFSMContext *context;

protected:

	/// References to parent FSMs
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;

	ReportPath p_report_global_path;
	ReportPath p_report_local_path;
	ReportPath p_report_global_historical_path;
	ReportPath p_report_local_historical_path;

	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr p_sub_global_path;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr p_sub_local_path;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr p_sub_pose;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr p_sub_odom;
	std::string p_tf_frame_world;
	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;
	std::string p_utm_zone;
	std::unique_ptr<tf2_ros::Buffer> p_tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> p_tf_listener;
	int p_maximum_points;
	bool p_use_tf_for_historical;
	double p_min_dist;
	geometry_msgs::msg::PoseStamped p_last_hist_local_pose;
	geometry_msgs::msg::PoseStamped p_last_hist_global_pose;
	iop::Timer p_tf_timer;

	void p_tf_callback();
	void p_ros_local_path(const nav_msgs::msg::Path::SharedPtr msg);
	void p_ros_global_path(const nav_msgs::msg::Path::SharedPtr msg);
	void p_ros_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void p_ros_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
	void p_apply_path2event(ReportPath& report_path, unsigned short path_type);
	bool p_different_pose(geometry_msgs::msg::PoseStamped &first, geometry_msgs::msg::PoseStamped &second);
	bool p_transform_pose(geometry_msgs::msg::PoseStamped& pose, std::string target_frame);
};

}

#endif // PATHREPORTER_RECEIVEFSM_H
