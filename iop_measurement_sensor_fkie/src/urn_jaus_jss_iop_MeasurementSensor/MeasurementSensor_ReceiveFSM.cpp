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

#include "urn_jaus_jss_iop_MeasurementSensor/MeasurementSensor_ReceiveFSM.h"

#include <iop_component_fkie/iop_config.h>


using namespace JTS;

namespace urn_jaus_jss_iop_MeasurementSensor
{



MeasurementSensor_ReceiveFSM::MeasurementSensor_ReceiveFSM(urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM)
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new MeasurementSensor_ReceiveFSMContext(*this);

	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
}



MeasurementSensor_ReceiveFSM::~MeasurementSensor_ReceiveFSM()
{
	delete context;
}

void MeasurementSensor_ReceiveFSM::setupNotifications()
{
	pEvents_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_MeasurementSensor_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	pEvents_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_MeasurementSensor_ReceiveFSM_Receiving_Ready", "Events_ReceiveFSM");
	registerNotification("Receiving_Ready", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving_Ready", "MeasurementSensor_ReceiveFSM");
	registerNotification("Receiving", pEvents_ReceiveFSM->getHandler(), "InternalStateChange_To_Events_ReceiveFSM_Receiving", "MeasurementSensor_ReceiveFSM");

	iop::Config cfg("~MeasurementSensor");
	p_measurement_sub = cfg.subscribe<iop_msgs_fkie::Measurement>("measurement", 1, &MeasurementSensor_ReceiveFSM::measurementReceived, this);
	pEvents_ReceiveFSM->get_event_handler().register_query(QueryMeasurement::ID);
}

void MeasurementSensor_ReceiveFSM::sendReportMeasurementAction(Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
	ROS_DEBUG("request measurement from %d.%d.%d",
			transportData.getSrcSubsystemID(), transportData.getSrcNodeID(), transportData.getSrcComponentID());
	JausAddress sender = JausAddress(transportData.getSrcSubsystemID(),
			transportData.getSrcNodeID(),
			transportData.getSrcComponentID());
	sendJausMessage(p_report_measurement, sender);
}

void MeasurementSensor_ReceiveFSM::measurementReceived(const iop_msgs_fkie::Measurement::ConstPtr& measurement)
{
	std::cout << "received measurement" << std::endl;
	ReportMeasurement msg;
	msg.getBody()->getMeasurementSeq()->getDeviceRec()->setDeviceName(measurement->device_name);
	if (!measurement->device_designation.empty()) {
		msg.getBody()->getMeasurementSeq()->getDeviceRec()->setDeviceDesignation(measurement->device_designation);
	}
	if (!measurement->classification.empty()) {
		msg.getBody()->getMeasurementSeq()->getDeviceRec()->setClassification(measurement->classification);
	}
	if (!isnan(measurement->latitude) && !isnan(measurement->longitude)) {
		msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setLatitude(measurement->latitude);
		msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setLongitude(measurement->longitude);
	}
	if (!isnan(measurement->altitude)) {
		msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setAltitude(measurement->altitude);
	}
	for (unsigned int i = 0; i < measurement->values.size() && i < 255; i++) {
		ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq valseq;
		iop_msgs_fkie::MeasurementValue mval = measurement->values[i];
		valseq.getReadingRec()->setSensor(mval.sensor);
		valseq.getReadingRec()->setSource(mval.source);
		valseq.getReadingRec()->setType(mval.type);
		valseq.getReadingRec()->setUnit(mval.unit);
		valseq.getReadingRec()->setMinimum(mval.min);
		valseq.getReadingRec()->setMaximum(mval.max);
		valseq.getReadingRec()->setAvarage(mval.avg);
		valseq.getReadingRec()->setAlertLevel(mval.alert_level);
		valseq.getReadingRec()->setAlertExplanation(mval.alert_explanation);
		valseq.getReadingRec()->setExtendedInfo(mval.extended_info);
		for (unsigned int v = 0; v < mval.value.size() && v < 255; v++) {
			ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq::ValueList::ValueRec valrec;
			valrec.setValue(mval.value[v]);
			valseq.getValueList()->addElement(valrec);
		}
		msg.getBody()->getMeasurementSeq()->getReadingsList()->addElement(valseq);
	}
	p_report_measurement = msg;
	pEvents_ReceiveFSM->get_event_handler().set_report(QueryMeasurement::ID, &p_report_measurement);
}

};
