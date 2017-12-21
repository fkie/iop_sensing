

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

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <iop_builder_fkie/timestamp.h>

#include "PathReporter_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_PathReporter
{

class DllExport PathReporter_ReceiveFSM : public JTS::StateMachine
{
public:
	PathReporter_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM);
	virtual ~PathReporter_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendReportPathAction(QueryPath msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportPathReporterCapabilitiesAction(QueryPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isSupported(QueryPath msg);



	PathReporter_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;

	ReportPathReporterCapabilities p_report_cap;
	ReportPath p_report_path;
	ReportPath p_report_global_historical_path;

	ros::Subscriber p_sub_path;
	ros::Subscriber p_sub_pose;
	ros::Subscriber p_sub_odom;
	std::string p_tf_frame_world;
	std::string p_tf_frame_robot;
	std::string p_utm_zone;
	tf::TransformListener tfListener;
	int p_maximum_points;
	bool p_use_tf_for_historical;
	double p_min_dist;
	geometry_msgs::PoseStamped p_last_hist_global_pose;
	ros::Timer p_tf_timer;

	void tfCallback(const ros::TimerEvent& e);
	void pRosPath(const nav_msgs::Path::ConstPtr& msg);
	void pRosPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pRosOdom(const nav_msgs::Odometry::ConstPtr& msg);
	void pApplyPath2Event(ReportPath& report_path, unsigned short path_type);
	bool pDifferentPose(geometry_msgs::PoseStamped &first, geometry_msgs::PoseStamped &second);

};

};

#endif // PATHREPORTER_RECEIVEFSM_H
