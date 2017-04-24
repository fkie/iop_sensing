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


#ifndef COSTMAP2D_RECEIVEFSM_H
#define COSTMAP2D_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_CostMap2D_1_0/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_CostMap2D_1_0/InternalEvents/InternalEventsSet.h"

typedef JTS::Receive_1_0 Receive;
typedef JTS::Send_1_0 Send;

#include "urn_jaus_jss_core_Transport_1_0/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events_1_0/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_AccessControl_1_0/AccessControl_ReceiveFSM.h"


#include "CostMap2D_ReceiveFSM_sm.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace urn_jaus_jss_iop_CostMap2D_1_0
{

class DllExport CostMap2D_ReceiveFSM : public JTS::StateMachine
{
public:
	CostMap2D_ReceiveFSM(urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM);
	virtual ~CostMap2D_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void addNoGoZoneAction(AddNoGoZone msg);
	virtual void removeNoGoZoneAction(RemoveNoGoZone msg);
	virtual void sendAddNoGoZoneResponseAction(AddNoGoZone msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportCostMap2DAction(QueryCostMap2D msg, Receive::Body::ReceiveRec transportData);
	virtual void sendReportNoGoZonesAction(QueryNoGoZones msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isSupported(AddNoGoZone msg);
	virtual bool zoneExists(RemoveNoGoZone msg);



	CostMap2D_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport_1_0::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events_1_0::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_AccessControl_1_0::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;

	tf::TransformListener* tfListener;
	std::string p_tf_frame_odom;
	std::string p_tf_frame_robot;
	double offset_yaw;
	int p_map_max_edge_size;
	ros::Subscriber costmap_sub;
	ReportCostMap2D p_costmap_msg;

	void pMapCallback (const nav_msgs::OccupancyGrid::ConstPtr& map_in);
};

};

#endif // COSTMAP2D_RECEIVEFSM_H
