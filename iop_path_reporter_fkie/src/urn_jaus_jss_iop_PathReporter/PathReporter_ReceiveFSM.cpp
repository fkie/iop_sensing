#include <cmath>
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
	p_use_tf_for_historical = false;
	p_min_dist = 0.25;
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
	// TODO:
//	cfg.param("use_tf_for_historical_path", p_use_tf_for_historical, p_use_tf_for_historical);
	cfg.param("utm_zone", p_utm_zone, p_utm_zone);
	cfg.param("min_dist", p_min_dist, p_min_dist);
	cfg.param("maximum_points", p_maximum_points, p_maximum_points);
	p_sub_path = cfg.subscribe<nav_msgs::Path>("planned_global_path", 1, &PathReporter_ReceiveFSM::pRosPath, this);
	if (!p_use_tf_for_historical) {
		p_sub_pose = cfg.subscribe<geometry_msgs::PoseStamped>("historical_pose", 1, &PathReporter_ReceiveFSM::pRosPose, this);
		p_sub_odom = cfg.subscribe<nav_msgs::Odometry>("historical_odom", 1, &PathReporter_ReceiveFSM::pRosOdom, this);
	} else {
		cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
		ros::NodeHandle nh;
		double tf_hz = 10.0;
		cfg.param("tf_hz", tf_hz, tf_hz);
		if (tf_hz == 0.0) {
			tf_hz = 10.0;
		}
		p_tf_timer = nh.createTimer(ros::Duration(1.0 / tf_hz), &PathReporter_ReceiveFSM::tfCallback, this);
	}
}

void PathReporter_ReceiveFSM::sendReportPathAction(QueryPath msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ROS_DEBUG_NAMED("PathReporter", "send path with %d global planned points to %s", p_report_path.getBody()->getPathVar()->getPlannedGlobalPath()->getNumberOfElements(), sender.str().c_str());
	ReportPath reply;
	// todo: apply other filter
	if (msg.getBody()->getQueryPathRec()->getPathType() == 0) {
		reply.getBody()->getPathVar()->setFieldValue(0);
		reply.getBody()->getPathVar()->setHistoricalGlobalPath(*p_report_global_historical_path.getBody()->getPathVar()->getHistoricalGlobalPath());
	}
	if (msg.getBody()->getQueryPathRec()->getPathType() == 2) {
		reply.getBody()->getPathVar()->setFieldValue(2);
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
	gpath.setFieldValue(2);
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
		if (pDifferentPose(p_last_hist_global_pose, pose_out)) {
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
			p_last_hist_global_pose = pose_out;
		}
	} catch (tf::TransformException &ex) {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	}
}

void PathReporter_ReceiveFSM::pRosOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	try {
		ReportPath::Body::PathVar::HistoricalGlobalPath* hgpath = p_report_global_historical_path.getBody()->getPathVar()->getHistoricalGlobalPath();
		ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec grec;
		geometry_msgs::PoseStamped pose_in;
		pose_in.header = msg->header;
		pose_in.pose = msg->pose.pose;
		tfListener.waitForTransform(p_tf_frame_world, msg->header.frame_id, msg->header.stamp, ros::Duration(0.3));
		geometry_msgs::PoseStamped pose_out;
		tfListener.transformPose(p_tf_frame_world, pose_in, pose_out);
		if (pDifferentPose(p_last_hist_global_pose, pose_out)) {
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
			p_last_hist_global_pose = pose_out;
		}
	} catch (tf::TransformException &ex) {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	}
}

void PathReporter_ReceiveFSM::tfCallback(const ros::TimerEvent& e)
{
//	try {
//		tfListener.waitForTransform(p_tf_frame_world, p_tf_frame_robot, ros::Time(0), ros::Duration(0.3));
//		tf::StampedTransform transform;
//		tfListener.lookupTransform(p_tf_frame_world, p_tf_frame_robot, ros::Time(0), transform);
//		ReportPath::Body::PathVar::HistoricalGlobalPath::GlobalPoseRec grec;
//		p_report_local_pose.getBody()->getLocalPoseRec()->setX(transform.getOrigin().x());
//		p_report_local_pose.getBody()->getLocalPoseRec()->setY(transform.getOrigin().y());
//		p_report_local_pose.getBody()->getLocalPoseRec()->setZ(transform.getOrigin().z());
//		try {
//			tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(),
//					transform.getRotation().getZ(), transform.getRotation().getW());
//			tf::Matrix3x3 m(q);
//			double yaw, pitch, roll;
//			m.getRPY(roll, pitch, yaw);
//			p_report_local_pose.getBody()->getLocalPoseRec()->setRoll(roll);
//			p_report_local_pose.getBody()->getLocalPoseRec()->setPitch(pitch);
//			p_report_local_pose.getBody()->getLocalPoseRec()->setYaw(yaw);
//		} catch (const std::exception& e) {
//			ROS_WARN_NAMED("PathReporter", "Error while get yaw, pitch, roll from quaternion: %s", e.what());
//		}
//		// set timestamp
//		ReportLocalPose::Body::LocalPoseRec::TimeStamp ts;
//		// current date/time based on current system
//		iop::Timestamp stamp(ros::Time::now());
//		ts.setDay(stamp.days);
//		ts.setHour(stamp.hours);
//		ts.setMinutes(stamp.minutes);
//		ts.setSeconds(stamp.seconds);
//		ts.setMilliseconds(stamp.milliseconds);
//		p_report_local_pose.getBody()->getLocalPoseRec()->setTimeStamp(ts);
//		pEvents_ReceiveFSM->get_event_handler().set_report(QueryLocalPose::ID, &p_report_local_pose);
//	} catch (tf::TransformException &ex){
//		ROS_WARN_STREAM_THROTTLE(1.0, "Could not lookup transform from " << p_tf_frame_robot << " to " << p_tf_frame_world << ": " << ex.what());
//	}

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

bool PathReporter_ReceiveFSM::pDifferentPose(geometry_msgs::PoseStamped &first, geometry_msgs::PoseStamped &second)
{
	double dist = std::sqrt(std::pow(first.pose.position.x - second.pose.position.x, 2) +
			std::pow(first.pose.position.y - second.pose.position.y, 2) +
			std::pow(first.pose.position.z - second.pose.position.z, 2));
	if (dist > p_min_dist) {
		return true;
	}
	try {
		double roll1, pitch1, yaw1;
		tf::Quaternion quat(first.pose.orientation.x, first.pose.orientation.y, first.pose.orientation.z, first.pose.orientation.w);
		tf::Matrix3x3(quat).getRPY(roll1, pitch1, yaw1);
		if (!isnan(yaw1)) {
			double roll2, pitch2, yaw2;
			tf::Quaternion quat2(second.pose.orientation.x, second.pose.orientation.y, second.pose.orientation.z, second.pose.orientation.w);
			tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
			if (!isnan(yaw1)) {
				return std::abs(yaw1 - yaw2) > 0.1;
			} else {
				return true;
			}
		}
	} catch (std::exception &e) {
		return false;
	}
	return false;
}

};
