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
#include <iop_component_fkie/iop_config.h>



using namespace JTS;

namespace urn_jaus_jss_iop_CostMap2D
{



CostMap2D_ReceiveFSM::CostMap2D_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new CostMap2D_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	tfListener = new tf::TransformListener(ros::Duration(120.0));
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

	pEvents_ReceiveFSM->get_event_handler().register_query(QueryCostMap2D::ID);
	iop::Config cfg("~CostMap2D");
	cfg.param<std::string>("frame_odom", p_tf_frame_odom, p_tf_frame_odom);
	cfg.param<std::string>("frame_robot", p_tf_frame_robot, p_tf_frame_robot);
//	std::string prefix = tf::getPrefixParam(pnh);
//	p_tf_frame_odom = tf::resolve(prefix, p_tf_frame_odom);
//	p_tf_frame_robot = tf::resolve(prefix, p_tf_frame_robot);
	cfg.param("offset_yaw", offset_yaw, offset_yaw);
	cfg.param("map_max_edge_size", p_map_max_edge_size, p_map_max_edge_size);
	//ROS subscriber:
	costmap_sub = cfg.subscribe<nav_msgs::OccupancyGrid>("map", 1, &CostMap2D_ReceiveFSM::pMapCallback, this);

}

void CostMap2D_ReceiveFSM::addNoGoZoneAction(AddNoGoZone msg)
{
	/// Insert User Code HERE
  ROS_WARN_NAMED("CostMap2D", "NoGoZones are not supported! (addNoGoZoneAction)");
}

void CostMap2D_ReceiveFSM::removeNoGoZoneAction(RemoveNoGoZone msg)
{
	/// Insert User Code HERE
  ROS_WARN_NAMED("CostMap2D", "NoGoZones are not supported! (removeNoGoZoneAction)");
}

void CostMap2D_ReceiveFSM::sendAddNoGoZoneResponseAction(AddNoGoZone msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	// report: NoGoZones are not supported
	uint16_t subsystem_id = transportData.getSrcSubsystemID();
	uint8_t node_id = transportData.getSrcNodeID();
	uint8_t component_id = transportData.getSrcComponentID();
	JausAddress sender(subsystem_id, node_id, component_id);
	ROS_DEBUG_NAMED("CostMap2D", "sendAddNoGoZoneResponseAction to %d.%d.%d", subsystem_id, node_id, component_id);
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
	ROS_DEBUG_NAMED("CostMap2D", "report cost map to %d.%d.%d", subsystem_id, node_id, component_id);
	ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = p_costmap_msg.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
	ROS_DEBUG_NAMED("CostMap2D", "   local position of the map %f . %f . %f\n",
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
	ROS_DEBUG_NAMED("CostMap2D", "sendReportNoGoZonesAction to %d.%d.%d", subsystem_id, node_id, component_id);
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

void CostMap2D_ReceiveFSM::pMapCallback (const nav_msgs::OccupancyGrid::ConstPtr& map_in) {
	try {
		tf::StampedTransform tf_pose;
		//get current robot to map transform:
		if(!tfListener->canTransform(map_in->header.frame_id, p_tf_frame_robot, map_in->header.stamp)) {
			tfListener->waitForTransform(map_in->header.frame_id, p_tf_frame_robot, map_in->header.stamp, ros::Duration(0.5));
		}
		tfListener->lookupTransform(map_in->header.frame_id, p_tf_frame_robot, map_in->header.stamp, tf_pose);
		ReportCostMap2D map;
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DRec *map_size = map.getBody()->getCostMap2DSeq()->getCostMap2DRec();
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DPoseVar *map_pose = map.getBody()->getCostMap2DSeq()->getCostMap2DPoseVar();
		// use local pose
		map_pose->setFieldValue(1);
		ReportCostMap2D::Body::CostMap2DSeq::CostMap2DDataVar *map_data = map.getBody()->getCostMap2DSeq()->getCostMap2DDataVar();
		// set map size
		ROS_DEBUG_NAMED("CostMap2D", "Reduce map to %dx%d!", p_map_max_edge_size, p_map_max_edge_size);
		int idx_width_start = tf_pose.getOrigin().getX() / map_in->info.resolution - p_map_max_edge_size / 2.0 + map_in->info.width / 2.0;
		int idx_width_end = idx_width_start + p_map_max_edge_size;
		int map_width = p_map_max_edge_size;
		int idx_height_end = tf_pose.getOrigin().getY() / map_in->info.resolution - p_map_max_edge_size / 2.0 + map_in->info.height / 2.0;
		int idx_height_start = idx_height_end + p_map_max_edge_size;
		int map_height = p_map_max_edge_size;
		// set map dimensions in the IOP message
		map_size->setNumberOfColumns(map_width);
		map_size->setNumberOfRows(map_height);
		map_size->setMapWidth(map_width * map_in->info.resolution);
		map_size->setMapHeight(map_height * map_in->info.resolution);
		// set map data
		// We have to flip around the y axis, y for jaus costmap starts at the top and y for map at the bottom
		// ROS: origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise
		// rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.
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
		tfListener->lookupTransform(p_tf_frame_odom, map_in->header.frame_id, map_in->header.stamp, tf_pose);
		tf::Quaternion q_odom_map(tf_pose.getRotation().getX(), tf_pose.getRotation().getY(), tf_pose.getRotation().getZ(), tf_pose.getRotation().getW());
		double map_yaw = tf::getYaw(q_odom_map);
		map_pose->getCostMap2DLocalPoseRec()->setMapRotation(map_yaw);
		// get point of the robot relative to odometry
		tfListener->lookupTransform(p_tf_frame_odom, p_tf_frame_robot, map_in->header.stamp, tf_pose);
		map_pose->getCostMap2DLocalPoseRec()->setMapCenterX(tf_pose.getOrigin().getX());
		map_pose->getCostMap2DLocalPoseRec()->setMapCenterY(tf_pose.getOrigin().getY());
		// set map
		p_costmap_msg = map;
		ROS_DEBUG_NAMED("CostMap2D", "Forward the map with origin %.2f, %.2f, yaw: %.2f", tf_pose.getOrigin().getX(), tf_pose.getOrigin().getY(), map_yaw);
		pEvents_ReceiveFSM->get_event_handler().set_report(QueryCostMap2D::ID, &p_costmap_msg);
	} catch (tf::TransformException& ex) {
		ROS_WARN_NAMED("CostMap2D", "TF Exception %s", ex.what());
	}
}


};
