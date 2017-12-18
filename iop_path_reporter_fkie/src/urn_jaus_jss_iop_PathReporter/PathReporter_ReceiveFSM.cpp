

#include "urn_jaus_jss_iop_PathReporter/PathReporter_ReceiveFSM.h"

#include <gps_common/conversions.h>
#include <tf/transform_datatypes.h>
#include <iop_component_fkie/iop_component.h>


using namespace JTS;

namespace urn_jaus_jss_iop_PathReporter
{



PathReporter_ReceiveFSM::PathReporter_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PathReporter_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	p_tf_frame_world = "/world";
	p_tf_frame_robot = "base_link";
	p_utm_zone = "32U";
	p_maximum_points = 15;
	ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList::PathReporterCapabilitiesRec cap_global_plan;
	cap_global_plan.setMaxTargetResolution(0);
	cap_global_plan.setMinTargetResolution(0);
	cap_global_plan.setPathType(2);
	p_report_cap.getBody()->getPathReporterCapabilitiesList()->addElement(cap_global_plan);
}



PathReporter_ReceiveFSM::~PathReporter_ReceiveFSM()
{
	delete context;
}

void PathReporter_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_PathReporter_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_PathReporter_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "PathReporter_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "PathReporter_ReceiveFSM");

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPath::ID);
	iop::Config cfg("~PathReporter");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("utm_zone", p_utm_zone, p_utm_zone);
	cfg.param("maximum_points", p_maximum_points, p_maximum_points);
	p_sub_path = cfg.subscribe<nav_msgs::Path>("planned_global_path", 1, &PathReporter_ReceiveFSM::pRosPath, this);
	p_sub_pose = cfg.subscribe<geometry_msgs::PoseStamped>("historical_pose", 1, &PathReporter_ReceiveFSM::pRosPose, this);
}

void PathReporter_ReceiveFSM::sendReportPathAction(QueryPath msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PathReporter", "send path with %d global planned points to %s", p_report_path.getBody()->getPathVar()->getPlannedGlobalPath()->getNumberOfElements(), sender.str().c_str());
	ReportPath reply;
	// todo: apply other filter
	if (msg.getBody()->getQueryPathRec()->getPathType() == 0) {
		reply.getBody()->getPathVar()->setHistoricalGlobalPath(*p_report_global_historical_path.getBody()->getPathVar()->getHistoricalGlobalPath());
	}
	if (msg.getBody()->getQueryPathRec()->getPathType() == 2) {
		reply.getBody()->getPathVar()->setPlannedGlobalPath(*p_report_path.getBody()->getPathVar()->getPlannedGlobalPath());
	}
	sendJausMessage(reply, sender);
}

void PathReporter_ReceiveFSM::sendReportPathReporterCapabilitiesAction(QueryPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PathReporter", "send capabilities to %s", sender.str().c_str());
	sendJausMessage(p_report_cap, sender);
}

bool PathReporter_ReceiveFSM::isSupported(QueryPath msg)
{
	// supports only global path
	return (msg.getBody()->getQueryPathRec()->getPathType() == 0
			|| msg.getBody()->getQueryPathRec()->getPathType() == 2);
}

void PathReporter_ReceiveFSM::pRosPath(const nav_msgs::Path::ConstPtr& msg)
{
	ROS_DEBUG_NAMED("PathReporter", "update path with new count: %d", (int)msg->poses.size());
	ReportPath::Body::PathVar gpath;
	bool transformed = false;
	for (unsigned int i = 0; i < msg->poses.size(); i++) {
		try {
			ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec grec;
			geometry_msgs::PoseStamped pose_in = msg->poses[i];
			if (pose_in.header.frame_id.empty()) {
				pose_in.header = msg->header;
			}
			tfListener.waitForTransform(p_tf_frame_world, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
			geometry_msgs::PoseStamped pose_out;
			tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
			double lat, lon = 0;
			gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
			grec.setLatitude(lat);
			grec.setLongitude(lon);
			grec.setAltitude(pose_out.pose.position.z);
			double roll, pitch, yaw;
			tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
			tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				grec.setRoll(roll);
				grec.setPitch(pitch);
				grec.setYaw(yaw);
			}
			transformed = true;
			gpath.getPlannedGlobalPath()->addElement(grec);
		} catch (tf::TransformException &ex) {
			printf ("Failure %s\n", ex.what()); //Print exception which was caught
		}
	}
	if (transformed || msg->poses.size() == 0) {
		p_report_path.getBody()->setPathVar(gpath);
		ROS_INFO_NAMED("PathReporter", "  global planned path updated, new count: %d", p_report_path.getBody()->getPathVar()->getPlannedGlobalPath()->getNumberOfElements());
		pApplyPath2Event(p_report_path, 2);
	}
}

void PathReporter_ReceiveFSM::pRosPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	try {
		ReportPath::Body::PathVar::HistoricalGlobalPath* hgpath = p_report_global_historical_path.getBody()->getPathVar()->getHistoricalGlobalPath();
		ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec grec;
		geometry_msgs::PoseStamped pose_in = *msg;
		if (pose_in.header.frame_id.empty()) {
			pose_in.header = msg->header;
		}
		tfListener.waitForTransform(p_tf_frame_world, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(0.3));
		geometry_msgs::PoseStamped pose_out;
		tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
		double lat, lon = 0;
		gps_common::UTMtoLL(pose_out.pose.position.y, pose_out.pose.position.x, p_utm_zone.c_str(), lat, lon);
		grec.setLatitude(lat);
		grec.setLongitude(lon);
		grec.setAltitude(pose_out.pose.position.z);
		double roll, pitch, yaw;
		tf::Quaternion quat(pose_out.pose.orientation.x, pose_out.pose.orientation.y, pose_out.pose.orientation.z, pose_out.pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		if (!isnan(yaw)) {
			grec.setRoll(roll);
			grec.setPitch(pitch);
			grec.setYaw(yaw);
		}
		hgpath->addElement(grec);
		if (hgpath->getNumberOfElements() > p_maximum_points) {
			hgpath->deleteElement(0);
		}
		ROS_DEBUG_NAMED("PathReporter", "  global historical path updated, new count: %d", hgpath->getNumberOfElements());
		pApplyPath2Event(p_report_global_historical_path, 0);
	} catch (tf::TransformException &ex) {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	}
}

void PathReporter_ReceiveFSM::pApplyPath2Event(ReportPath& report_path, unsigned short path_type)
{
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage> queries;
	pEvents_ReceiveFSM->get_event_handler().get_queries(QueryPath::ID, queries);
	std::map<jUnsignedByte, urn_jaus_jss_core_Events::CreateEvent::Body::CreateEventRec::QueryMessage>::iterator it;
	for (it = queries.begin(); it != queries.end(); ++it) {
		QueryPath qr;
		qr.decode(it->second.getData());
		if (qr.getBody()->getQueryPathRec()->getPathType() == path_type) {
			// todo: apply other filter
			pEvents_ReceiveFSM->get_event_handler().send_report(it->first, report_path, path_type);
		}
	}
}

};
