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




#ifndef MEASUREMENTSENSOR_RECEIVEFSM_H
#define MEASUREMENTSENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_MeasurementSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_MeasurementSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"

#include <ros/ros.h>
#include <iop_msgs_fkie/Measurement.h>

#include "MeasurementSensor_ReceiveFSM_sm.h"

namespace urn_jaus_jss_iop_MeasurementSensor
{

class DllExport MeasurementSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	MeasurementSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM);
	virtual ~MeasurementSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();

	/// Action Methods
	virtual void sendReportMeasurementAction(Receive::Body::ReceiveRec transportData);


	/// Guard Methods



	MeasurementSensor_ReceiveFSMContext *context;

protected:

    /// References to parent FSMs
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;

	ros::Subscriber p_measurement_sub;
	ReportMeasurement p_report_measurement;

	void measurementReceived(const iop_msgs_fkie::Measurement::ConstPtr& measurement);

};

};

#endif // MEASUREMENTSENSOR_RECEIVEFSM_H

