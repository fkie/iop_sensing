#include <cmath>
#include "urn_jaus_jss_iop_PathReporter/PathReporter_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

#include <fkie_iop_component/gps_conversions.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace JTS;

namespace urn_jaus_jss_iop_PathReporter
{



PathReporter_ReceiveFSM::PathReporter_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("PathReporter")),
  p_tf_timer(std::chrono::seconds(1), std::bind(&PathReporter_ReceiveFSM::p_tf_callback, this), false)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new PathReporter_ReceiveFSMContext(*this);

	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_world = "/world";
	p_tf_frame_odom = "odom";
	p_tf_frame_robot = "base_link";
	p_utm_zone = "32U";
	p_maximum_points = 15;
	p_use_tf_for_historical = false;
	p_min_dist = 0.25;
	p_report_local_historical_path.getBody()->getPathVar()->setFieldValue(1);
	p_report_global_path.getBody()->getPathVar()->setFieldValue(2);
	p_report_local_path.getBody()->getPathVar()->setFieldValue(3);
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
}


void PathReporter_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "PathReporter");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryPath::ID);
	cfg.declare_param<std::string>("tf_frame_world", p_tf_frame_world, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"TF frame id used in ROS for global coordinates.",
		"Default: '/world'");
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Frame id of local coordinates published to IOP.",
		"Default: 'odom'");
	cfg.declare_param<std::string>("utm_zone", p_utm_zone, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"The UTM zone is used for translation of ROS global position coordinates into Lat/Lon coordinates.",
		"Default: '32U'");
	cfg.declare_param<int>("maximum_points", p_maximum_points, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"The maximum points in the local history list",
		"Default: 15");
	cfg.declare_param<double>("min_dist", p_min_dist, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
		"This value is used for historical path. A new point is only added if a distance to previous point is more then min_dist.",
		"Default: 0.25");
	cfg.param("tf_frame_world", p_tf_frame_world, p_tf_frame_world);
	cfg.param("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
//	cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	cfg.param("utm_zone", p_utm_zone, p_utm_zone);
	cfg.param("min_dist", p_min_dist, p_min_dist);
	cfg.param("maximum_points", p_maximum_points, p_maximum_points);
	p_sub_local_path = cfg.create_subscription<nav_msgs::msg::Path>("planned_local_path", 1, std::bind(&PathReporter_ReceiveFSM::p_ros_local_path, this, std::placeholders::_1));
	p_sub_global_path = cfg.create_subscription<nav_msgs::msg::Path>("planned_global_path", 1, std::bind(&PathReporter_ReceiveFSM::p_ros_global_path, this, std::placeholders::_1));
	if (!p_use_tf_for_historical) {
		p_sub_pose = cfg.create_subscription<geometry_msgs::msg::PoseStamped>("historical_pose", 1, std::bind(&PathReporter_ReceiveFSM::p_ros_pose, this, std::placeholders::_1));
		p_sub_odom = cfg.create_subscription<nav_msgs::msg::Odometry>("historical_odom", 1, std::bind(&PathReporter_ReceiveFSM::p_ros_odom, this, std::placeholders::_1));
	} else {
		// cfg.param("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
		double tf_hz = 10.0;
		cfg.param("tf_hz", tf_hz, tf_hz);
		p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
		p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);
		if (tf_hz == 0.0) {
			tf_hz = 10.0;
		}
		p_tf_timer.set_rate(tf_hz);
		p_tf_timer.start();
	}
}

void PathReporter_ReceiveFSM::sendReportPathAction(QueryPath msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	ReportPath reply;
	RCLCPP_DEBUG(logger, "received query for %d from %s", msg.getBody()->getQueryPathRec()->getPathType(), sender.str().c_str());
//	if (msg.getBody()->getQueryPathRec()->getPathType() == 0) {
//		reply.getBody()->getPathVar()->setFieldValue(0);
//		reply.getBody()->getPathVar()->setHistoricalGlobalPath(*p_report_global_historical_path.getBody()->getPathVar()->getHistoricalGlobalPath());
//	}
	if (msg.getBody()->getQueryPathRec()->getPathType() == 1) {
		reply.getBody()->getPathVar()->setFieldValue(1);
		reply.getBody()->getPathVar()->setHistoricalLocalPath(*p_report_local_historical_path.getBody()->getPathVar()->getHistoricalLocalPath());
	}
	if (msg.getBody()->getQueryPathRec()->getPathType() == 2) {
		RCLCPP_DEBUG(logger, "send path with %d global planned points to %s", p_report_global_path.getBody()->getPathVar()->getPlannedGlobalPath()->getNumberOfElements(), sender.str().c_str());
		reply.getBody()->getPathVar()->setFieldValue(2);
		reply.getBody()->getPathVar()->setPlannedGlobalPath(*p_report_global_path.getBody()->getPathVar()->getPlannedGlobalPath());
	}
	if (msg.getBody()->getQueryPathRec()->getPathType() == 3) {
		RCLCPP_DEBUG(logger, "send path with %d local planned points to %s", p_report_local_path.getBody()->getPathVar()->getPlannedLocalPath()->getNumberOfElements(), sender.str().c_str());
		reply.getBody()->getPathVar()->setFieldValue(3);
		reply.getBody()->getPathVar()->setPlannedLocalPath(*p_report_local_path.getBody()->getPathVar()->getPlannedLocalPath());
	}
	sendJausMessage(reply, sender);
}

void PathReporter_ReceiveFSM::sendReportPathReporterCapabilitiesAction(QueryPathReporterCapabilities msg, Receive::Body::ReceiveRec transportData)
{
	JausAddress sender = transportData.getAddress();
	RCLCPP_DEBUG(logger, "send capabilities to %s", sender.str().c_str());
	ReportPathReporterCapabilities report_cap;
	if (p_sub_pose->get_publisher_count() > 0 || p_sub_odom->get_publisher_count() > 0) {
		ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList::PathReporterCapabilitiesRec cap_hist_local_plan;
		cap_hist_local_plan.setMaxTargetResolution(0);
		cap_hist_local_plan.setMinTargetResolution(0);
		cap_hist_local_plan.setPathType(1);
		report_cap.getBody()->getPathReporterCapabilitiesList()->addElement(cap_hist_local_plan);
	}
	if (p_sub_global_path->get_publisher_count() > 0) {
		ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList::PathReporterCapabilitiesRec cap_global_plan;
		cap_global_plan.setMaxTargetResolution(0);
		cap_global_plan.setMinTargetResolution(0);
		cap_global_plan.setPathType(2);
		report_cap.getBody()->getPathReporterCapabilitiesList()->addElement(cap_global_plan);
	}
	if (p_sub_local_path->get_publisher_count() > 0) {
		ReportPathReporterCapabilities::Body::PathReporterCapabilitiesList::PathReporterCapabilitiesRec cap_local_plan;
		cap_local_plan.setMaxTargetResolution(0);
		cap_local_plan.setMinTargetResolution(0);
		cap_local_plan.setPathType(3);
		report_cap.getBody()->getPathReporterCapabilitiesList()->addElement(cap_local_plan);
	}
	sendJausMessage(report_cap, sender);
}

bool PathReporter_ReceiveFSM::isSupported(QueryPath msg)
{
	// supports only global path
	return ((msg.getBody()->getQueryPathRec()->getPathType() == 1 && (p_sub_pose->get_publisher_count() > 0 || p_sub_odom->get_publisher_count() > 0))
			|| (msg.getBody()->getQueryPathRec()->getPathType() == 2 && p_sub_global_path->get_publisher_count() > 0)
			|| (msg.getBody()->getQueryPathRec()->getPathType() == 3 && p_sub_local_path->get_publisher_count() > 0));
}

void PathReporter_ReceiveFSM::p_ros_local_path(const nav_msgs::msg::Path::SharedPtr msg)
{
	RCLCPP_DEBUG(logger, "update local path with new count: %d", (int)msg->poses.size());
	ReportPath::Body::PathVar gpath;
	gpath.setFieldValue(2);
	bool transformed = false;
	for (unsigned int i = 0; i < msg->poses.size(); i++) {
		ReportPath::Body::PathVar::PlannedLocalPath::LocalPoseRec lrec;
		auto pose = msg->poses[i];
		if (pose.header.frame_id.empty()) {
			pose.header = msg->header;
		}
		if (p_transform_pose(pose, p_tf_frame_odom)) {
			lrec.setX(pose.pose.position.x);
			lrec.setY(pose.pose.position.y);
			lrec.setZ(pose.pose.position.z);
			double roll, pitch, yaw;
			tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
			tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				lrec.setRoll(roll);
				lrec.setPitch(pitch);
				lrec.setYaw(yaw);
			}
			transformed = true;
			gpath.getPlannedLocalPath()->addElement(lrec);
		}
	}
	if (transformed || msg->poses.size() == 0) {
		p_report_local_path.getBody()->setPathVar(gpath);
		RCLCPP_DEBUG(logger, "  local planned path updated, new count: %d", p_report_local_path.getBody()->getPathVar()->getPlannedLocalPath()->getNumberOfElements());
		p_apply_path2event(p_report_local_path, 3);
	}
}

void PathReporter_ReceiveFSM::p_ros_global_path(const nav_msgs::msg::Path::SharedPtr msg)
{
	RCLCPP_DEBUG(logger, "update global path with new count: %d", (int)msg->poses.size());
	ReportPath::Body::PathVar gpath;
	gpath.setFieldValue(2);
	bool transformed = false;
	for (unsigned int i = 0; i < msg->poses.size(); i++) {
		ReportPath::Body::PathVar::PlannedGlobalPath::GlobalPoseRec grec;
		auto pose = msg->poses[i];
		if (pose.header.frame_id.empty()) {
			pose.header = msg->header;
		}
		if (p_transform_pose(pose, p_tf_frame_world)) {
			double lat, lon = 0;
			gps_common::UTMtoLL(pose.pose.position.y, pose.pose.position.x, p_utm_zone.c_str(), lat, lon);
			grec.setLatitude(lat);
			grec.setLongitude(lon);
			grec.setAltitude(pose.pose.position.z);
			double roll, pitch, yaw;
			tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
			tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			if (!isnan(yaw)) {
				grec.setRoll(roll);
				grec.setPitch(pitch);
				grec.setYaw(yaw);
			}
			transformed = true;
			gpath.getPlannedGlobalPath()->addElement(grec);
		}
	}
	if (transformed || msg->poses.size() == 0) {
		p_report_global_path.getBody()->setPathVar(gpath);
		RCLCPP_DEBUG(logger, "  global planned path updated, new count: %d", p_report_global_path.getBody()->getPathVar()->getPlannedGlobalPath()->getNumberOfElements());
		p_apply_path2event(p_report_global_path, 2);
	}
}

void PathReporter_ReceiveFSM::p_ros_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	ReportPath::Body::PathVar::HistoricalLocalPath* hgpath = p_report_local_historical_path.getBody()->getPathVar()->getHistoricalLocalPath();
	ReportPath::Body::PathVar::HistoricalLocalPath::LocalPoseRec lrec;
	auto pose = *msg;
	bool transformed = p_transform_pose(pose, p_tf_frame_odom);
	if (transformed && p_different_pose(p_last_hist_local_pose, pose)) {
		lrec.setX(pose.pose.position.x);
		lrec.setY(pose.pose.position.y);
		lrec.setZ(pose.pose.position.z);
		double roll, pitch, yaw;
		tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		if (!isnan(yaw)) {
			lrec.setRoll(roll);
			lrec.setPitch(pitch);
			lrec.setYaw(yaw);
		}
		hgpath->addElement(lrec);
		if (hgpath->getNumberOfElements() > p_maximum_points) {
			hgpath->deleteElement(0);
		}
		RCLCPP_DEBUG(logger, "  local historical path updated, new count: %d", hgpath->getNumberOfElements());
		p_apply_path2event(p_report_local_historical_path, 1);
		p_last_hist_local_pose = pose;
	}
}

void PathReporter_ReceiveFSM::p_ros_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	ReportPath::Body::PathVar::HistoricalLocalPath* hgpath = p_report_local_historical_path.getBody()->getPathVar()->getHistoricalLocalPath();
	ReportPath::Body::PathVar::HistoricalLocalPath::LocalPoseRec lrec;
	auto pose = geometry_msgs::msg::PoseStamped();
	pose.header = msg->header;
	pose.pose = msg->pose.pose;
	bool transformed = p_transform_pose(pose, p_tf_frame_odom);
	if (transformed && p_different_pose(p_last_hist_local_pose, pose)) {
		lrec.setX(pose.pose.position.x);
		lrec.setY(pose.pose.position.y);
		lrec.setZ(pose.pose.position.z);
		double roll, pitch, yaw;
		tf2::Quaternion quat(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		if (!isnan(yaw)) {
			lrec.setRoll(roll);
			lrec.setPitch(pitch);
			lrec.setYaw(yaw);
		}
		hgpath->addElement(lrec);
		if (hgpath->getNumberOfElements() > p_maximum_points) {
			hgpath->deleteElement(0);
		}
		RCLCPP_DEBUG(logger, "  local historical path updated, new count: %d", hgpath->getNumberOfElements());
		p_apply_path2event(p_report_local_historical_path, 1);
		p_last_hist_local_pose = pose;
	}
}

void PathReporter_ReceiveFSM::p_tf_callback()
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
//			RCLCPP_WARN(logger, "Error while get yaw, pitch, roll from quaternion: %s", e.what());
//		}
//		// set timestamp
//		ReportLocalPose::Body::LocalPoseRec::TimeStamp ts;
//		// current date/time based on current system
//		iop::Timestamp stamp(cmp->now());
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

void PathReporter_ReceiveFSM::p_apply_path2event(ReportPath& report_path, unsigned short path_type)
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
			RCLCPP_DEBUG(logger, "  p_apply_path2event, %d added to events", path_type);
		} else {
			RCLCPP_DEBUG(logger, "  p_apply_path2event, %d not in query, only %d", path_type, qr.getBody()->getQueryPathRec()->getPathType());
		}
	}
}

bool PathReporter_ReceiveFSM::p_different_pose(geometry_msgs::msg::PoseStamped &first, geometry_msgs::msg::PoseStamped &second)
{
	double dist = std::sqrt(std::pow(first.pose.position.x - second.pose.position.x, 2) +
			std::pow(first.pose.position.y - second.pose.position.y, 2) +
			std::pow(first.pose.position.z - second.pose.position.z, 2));
	if (dist > p_min_dist) {
		return true;
	}
	try {
		double roll1, pitch1, yaw1;
		tf2::Quaternion quat(first.pose.orientation.x, first.pose.orientation.y, first.pose.orientation.z, first.pose.orientation.w);
		tf2::Matrix3x3(quat).getRPY(roll1, pitch1, yaw1);
		if (!isnan(yaw1)) {
			double roll2, pitch2, yaw2;
			tf2::Quaternion quat2(second.pose.orientation.x, second.pose.orientation.y, second.pose.orientation.z, second.pose.orientation.w);
			tf2::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
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

bool PathReporter_ReceiveFSM::p_transform_pose(geometry_msgs::msg::PoseStamped& pose, std::string target_frame)
{
	try {
		if (pose.header.frame_id.compare(target_frame) != 0) {
			p_tf_buffer->lookupTransform(p_tf_frame_robot, pose.header.frame_id, pose.header.stamp, rclcpp::Duration(0.3));
			auto pose_out = geometry_msgs::msg::PoseStamped();
			p_tf_buffer->transform(pose, pose, p_tf_frame_world);
		}
		return true;
	} catch (tf2::TransformException &ex) {
		printf ("Failure %s\n", ex.what()); //Print exception which was caught
	}
	return false;
}

}
