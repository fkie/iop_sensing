/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */


#include "urn_jaus_jss_iop_CostMap2D/CostMap2D_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>


using namespace JTS;

namespace urn_jaus_jss_iop_CostMap2D
{



CostMap2D_ReceiveFSM::CostMap2D_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("CostMap2D"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new CostMap2D_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
	p_tf_frame_odom = "odom";
	p_tf_frame_robot = "base_link";
	offset_yaw = 0;
	p_map_max_edge_size = 255;
}



CostMap2D_ReceiveFSM::~CostMap2D_ReceiveFSM()
{
	delete context;
}

void CostMap2D_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_CostMap2D_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_CostMap2D_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_CostMap2D_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_CostMap2D_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "CostMap2D_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "CostMap2D_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "CostMap2D_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "CostMap2D_ReceiveFSM");
}


void CostMap2D_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "CostMap2D");
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryCostMap2D::ID);
	cfg.declare_param<std::string>("tf_frame_odom", p_tf_frame_odom, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the odometry frame id.",
		"Default: 'odom'");
	cfg.declare_param<std::string>("tf_frame_robot", p_tf_frame_robot, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_STRING,
		"Defines the robot frame id.",
		"Default: 'base_link'");
	cfg.declare_param<int32_t>("map_max_edge_size", p_map_max_edge_size, true,
		rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
		"Width and height of the map reported by this service. The actual size depends also on resolution of OccupancyGrid.",
		"Default: 255");
	cfg.param<std::string>("tf_frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param<std::string>("tf_frame_robot", p_tf_frame_robot, p_tf_frame_robot);
	p_tf_buffer = std::make_unique<tf2_ros::Buffer>(cmp->get_clock());
	p_tf_listener = std::make_shared<tf2_ros::TransformListener>(*p_tf_buffer);

//	std::string prefix = tf::getPrefixParam(pnh);
//	p_tf_frame_odom = tf::resolve(prefix, p_tf_frame_odom);
//	p_tf_frame_robot = tf::resolve(prefix, p_tf_frame_robot);
	cfg.param("offset_yaw", offset_yaw, offset_yaw);
	cfg.param("map_max_edge_size", p_map_max_edge_size, p_map_max_edge_size);
	//ROS subscriber:
	costmap_sub = cfg.create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&CostMap2D_ReceiveFSM::pMapCallback, this, std::placeholders::_1));

}

void CostMap2D_ReceiveFSM::addNoGoZoneAction(AddNoGoZone msg)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "NoGoZones are not supported! (addNoGoZoneAction)");
}

void CostMap2D_ReceiveFSM::removeNoGoZoneAction(RemoveNoGoZone msg)
{
	/// Insert User Code HERE
	RCLCPP_WARN(logger, "NoGoZones are not supported! (removeNoGoZoneAction)");
}

void CostMap2D_ReceiveFSM::sendAddNoGoZoneResponseAction(AddNoGoZone msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	// report: NoGoZones are not supported
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "sendAddNoGoZoneResponseAction to %d.%d.%d", subsystem_id, node_id, component_id);
	AddNoGoZoneResponse response;
	unsigned short int request_id = msg.getBody()->getNoGoZoneSeq()->getRequestIDRec()->getRequestID();
	response.getBody()->getAddNoGoZoneResponseRec()->setRequestID(request_id);
	response.getBody()->getAddNoGoZoneResponseRec()->setResponseCode(1); // GlobalVerticesNotSupported
	this->sendJausMessage(response, sender);
	response.getBody()->getAddNoGoZoneResponseRec()->setResponseCode(2); // LocalVerticesNotSupported
	this->sendJausMessage(response, sender);
}

void CostMap2D_ReceiveFSM::sendReportCostMap2DAction(QueryCostMap2D msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "report cost map to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = p_costmap_msg.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
	RCLCPP_DEBUG(logger, "   local position of the map %f . %f . %f\n",
			map_pose->getCostMap2DLocalPoseRec()->getMapCenterX(),
			map_pose->getCostMap2DLocalPoseRec()->getMapCenterY(),
			map_pose->getCostMap2DLocalPoseRec()->getMapRotation());
	this->sendJausMessage(p_costmap_msg, sender);
}

void CostMap2D_ReceiveFSM::sendReportNoGoZonesAction(QueryNoGoZones msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	RCLCPP_DEBUG(logger, "sendReportNoGoZonesAction to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportNoGoZones response;
	this->sendJausMessage(response, sender);
}



bool CostMap2D_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}

bool CostMap2D_ReceiveFSM::isSupported(AddNoGoZone msg)
{
	/// Insert User Code HERE
	return false;
}

bool CostMap2D_ReceiveFSM::zoneExists(RemoveNoGoZone msg)
{
	/// Insert User Code HERE
	return false;
}

void CostMap2D_ReceiveFSM::pMapCallback (const nav_msgs::msg::OccupancyGrid::SharedPtr map_in) {
	try {
		RCLCPP_DEBUG(logger, "Map received, transform...");
		//get current robot to map transform:
		geometry_msgs::msg::TransformStamped tf_pose;
		p_tf_buffer->lookupTransform(map_in->header.frame_id, p_tf_frame_robot, map_in->header.stamp, rclcpp::Duration(1.5));
		RCLCPP_DEBUG(logger, "  map origin: %.2f, %.2f", map_in->info.origin.position.x, map_in->info.origin.position.y);
		RCLCPP_DEBUG(logger, "  robot position from tf: %.2f, %.2f", map_in->info.origin.position.x, map_in->info.origin.position.y);
		// apply map origin to robot position
		tf2::Quaternion q(map_in->info.origin.orientation.x, map_in->info.origin.orientation.y, map_in->info.origin.orientation.z, map_in->info.origin.orientation.w);
		tf2::Vector3 r(map_in->info.origin.position.x, map_in->info.origin.position.y, map_in->info.origin.position.z);
		tf2::Transform transform(q, r);
		tf2::Quaternion qr(tf_pose.transform.rotation.x, tf_pose.transform.rotation.y, tf_pose.transform.rotation.z, tf_pose.transform.rotation.w);
		tf2::Vector3 rr(tf_pose.transform.translation.x, tf_pose.transform.translation.y, tf_pose.transform.translation.z);
		tf2::Transform transform_rob(qr, rr);
		tf2::Transform tr_result = transform.inverse() * transform_rob;
		RCLCPP_DEBUG(logger, "  transformed robot position %.2f, %.2f", tr_result.getOrigin().getX(), tr_result.getOrigin().getY());
		ReportCostMap2D map;
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DRec *map_size = map.getBody()->getCostMap2DSeq()->getCostMap2DRec();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = map.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
		// use local pose
		map_pose->setFieldValue(1);
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar *map_data = map.getBody()->getCostMap2DSeq()->getCostMap2DDataVar();
		// set map size
		int x_res = tr_result.getOrigin().getX() / map_in->info.resolution;
		int map_width = map_in->info.width;
		if (map_in->info.width > p_map_max_edge_size) {
			map_width = p_map_max_edge_size;
		}
		int idx_width_start = x_res - map_width / 2;
		int idx_width_end = x_res + map_width / 2;
		int x_offset_res = 0;
		// check boundaries for width
		if (idx_width_start < 0) {
			// robot on the left border, add offset to the center of the map
			x_offset_res = -idx_width_start;
			idx_width_end -= idx_width_start;
			idx_width_start = 0;
		}
		if (idx_width_end > map_in->info.width) {
			// robot on the right border, add offset to the center of the map
			x_offset_res = map_in->info.width - idx_width_end;
			idx_width_start += x_offset_res;
			idx_width_end = map_in->info.width;
		}

		int y_res = tr_result.getOrigin().getY() / map_in->info.resolution;
		int map_height = map_in->info.height;
		if (map_in->info.height > p_map_max_edge_size) {
			map_height = p_map_max_edge_size;
		}
		int idx_height_start = y_res + map_height / 2;
		int idx_height_end = y_res - map_height / 2;
		int y_offset_res = 0;
		// check boundaries for height
		if (idx_height_end < 0) {
			// robot on the bottom border, add offset to the center of the map
			y_offset_res = -idx_height_end;
			idx_height_start -= idx_height_end;
			idx_height_end = 0;
		}
		if (idx_height_start > map_in->info.height) {
			// robot on the top border, add offset to the center of the map
			y_offset_res = map_in->info.height - idx_height_start;
			idx_height_end += y_offset_res;
			idx_height_start = map_in->info.height;
		}

		RCLCPP_DEBUG(logger, "  map size original [px]: %d x %d, used %d x %d", map_in->info.width, map_in->info.height, map_width, map_height);
		RCLCPP_DEBUG(logger, "  robot position [px]:  %d, %d, offset %d, %d", x_res, y_res, x_offset_res, y_offset_res);
		RCLCPP_DEBUG(logger, "  estimated robot offset if robot on boundary: %.2f, %.2f", x_offset_res * map_in->info.resolution, y_offset_res * map_in->info.resolution);
		RCLCPP_DEBUG(logger, "  used index width [px]: %d - %d, height: %d - %d", idx_width_start, idx_width_end, idx_height_start, idx_height_end);

		// set map dimensions in the IOP message
		map_size->setNumberOfColumns(idx_width_end - idx_width_start);
		map_size->setNumberOfRows(idx_height_start - idx_height_end);
		map_size->setMapWidth((idx_width_end - idx_width_start) * map_in->info.resolution);
		map_size->setMapHeight((idx_height_start - idx_height_end) * map_in->info.resolution);
		// set map data
		// We have to flip around the y axis, y for jaus costmap starts at the top and y for map at the bottom
		// ROS: origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise
		// rotation (yaw=0 means no rotation)
		for (int y_i = idx_height_start-1; y_i >= idx_height_end; y_i--) {
			int idx_map_y = map_in->info.width * y_i;
			for (int x_i = idx_width_start; x_i < idx_width_end; x_i++) {
				ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar::CostDataList::CostDataRec datarec;
				datarec.setCost(255);
				if ( 0 <= idx_map_y + x_i and idx_map_y + x_i < map_in->data.size()) {
					int map_val = map_in->data[idx_map_y + x_i];
					if (map_val  >= 0) {
						datarec.setCost(map_val * 2);
					}
				}
				map_data->getCostDataList()->addElement(datarec);
			}
		}
		// get orientation of the map relative to odometry
		double x_center = tf_pose.transform.translation.x + x_offset_res * map_in->info.resolution;
		double y_center = tf_pose.transform.translation.y + y_offset_res * map_in->info.resolution;
		tf_pose = p_tf_buffer->lookupTransform(p_tf_frame_odom, map_in->header.frame_id, map_in->header.stamp, rclcpp::Duration(0.5));
		tf2::Quaternion q_odom_map(tf_pose.transform.rotation.x, tf_pose.transform.rotation.y, tf_pose.transform.rotation.z, tf_pose.transform.rotation.w);
		double map_yaw = tf2::getYaw(q_odom_map); // TODO: invert because IOP uses clockwise map rotation?
		geometry_msgs::msg::PoseStamped pose_center;
		pose_center.header = map_in->header;
		pose_center.pose.position.x = x_center;
		pose_center.pose.position.y = y_center;
		pose_center.pose.orientation.w = 1.0;
		p_tf_buffer->transform(pose_center, pose_center, p_tf_frame_odom);
		map_pose->getCostMap2DLocalPoseRec()->setMapRotation(map_yaw);
		// get point of the robot position relative to odometry -> this is the local center coordinate of the map
		RCLCPP_DEBUG(logger, "  estimated map center %.2f, %.2f, yaw: %.2f", pose_center.pose.position.x, pose_center.pose.position.y, map_yaw);
		map_pose->getCostMap2DLocalPoseRec()->setMapCenterX(pose_center.pose.position.x);
		map_pose->getCostMap2DLocalPoseRec()->setMapCenterY(pose_center.pose.position.y);
		// TODO: add global coordinate to the map
		// set map
		p_costmap_msg = map;
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryCostMap2D::ID, &p_costmap_msg);
	} catch (tf2::TransformException& ex) {
		RCLCPP_WARN(logger, "TF Exception %s", ex.what());
	}
}


}
