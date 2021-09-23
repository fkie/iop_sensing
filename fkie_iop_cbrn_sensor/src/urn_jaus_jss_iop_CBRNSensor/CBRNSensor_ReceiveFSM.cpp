

#include "urn_jaus_jss_iop_CBRNSensor/CBRNSensor_ReceiveFSM.h"
#include <fkie_iop_component/iop_config.hpp>




using namespace JTS;

namespace urn_jaus_jss_iop_CBRNSensor
{



CBRNSensor_ReceiveFSM::CBRNSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM)
: logger(cmp->get_logger().get_child("CBRNSensor"))
{

	/*
	 * If there are other variables, context must be constructed last so that all
	 * class variables are available if an EntryAction of the InitialState of the
	 * statemachine needs them.
	 */
	context = new CBRNSensor_ReceiveFSMContext(*this);

	this->pAccessControl_ReceiveFSM = pAccessControl_ReceiveFSM;
	this->pEvents_ReceiveFSM = pEvents_ReceiveFSM;
	this->pTransport_ReceiveFSM = pTransport_ReceiveFSM;
	this->cmp = cmp;
}


CBRNSensor_ReceiveFSM::~CBRNSensor_ReceiveFSM()
{
	delete context;
}

void CBRNSensor_ReceiveFSM::setupNotifications()
{
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_NotControlled", ieHandler, "InternalStateChange_To_CBRNSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready_Controlled", ieHandler, "InternalStateChange_To_CBRNSensor_ReceiveFSM_Receiving_Ready_Controlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving_Ready", ieHandler, "InternalStateChange_To_CBRNSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	pAccessControl_ReceiveFSM->registerNotification("Receiving", ieHandler, "InternalStateChange_To_CBRNSensor_ReceiveFSM_Receiving_Ready_NotControlled", "AccessControl_ReceiveFSM");
	registerNotification("Receiving_Ready_NotControlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_NotControlled", "CBRNSensor_ReceiveFSM");
	registerNotification("Receiving_Ready_Controlled", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready_Controlled", "CBRNSensor_ReceiveFSM");
	registerNotification("Receiving_Ready", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving_Ready", "CBRNSensor_ReceiveFSM");
	registerNotification("Receiving", pAccessControl_ReceiveFSM->getHandler(), "InternalStateChange_To_AccessControl_ReceiveFSM_Receiving", "CBRNSensor_ReceiveFSM");

}


void CBRNSensor_ReceiveFSM::setupIopConfiguration()
{
	iop::Config cfg(cmp, "CBRNSensor");
	p_measurement_sub = cfg.create_subscription<fkie_iop_msgs::msg::Measurement>("measurements", 1, std::bind(&CBRNSensor_ReceiveFSM::measurementReceived, this, std::placeholders::_1));
	//pEvents_ReceiveFSM->get_event_handler().register_query(QueryMeasurement::ID);
}



void CBRNSensor_ReceiveFSM::applyCBRNCommandAction(CBRNCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::applyCBRNConfigurationAction(CBRNConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::applyCBRNDisplayConfigurationAction(CBRNDisplayConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::applyCBRNScanWindowConfigurationAction(CBRNScanWindowConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBDetailedMeasurementAction(QueryCBDetailedMeasurement msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRN4Action(QueryCBRN4 msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRN4Action(Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNConfigurationAction(QueryCBRNConfiguration msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNConfigurationStateAction(CBRNConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNConfigurationStateAction(CBRNDisplayConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNDetectionAction(QueryCBRNDetection msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNDisplayConfigurationAction(QueryCBRNOperatingState msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNOperatingStateAction(CBRNCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNOperatingStateAction(QueryCBRNOperatingState msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNScanWindowOperatingStateAction(CBRNScanWindowConfigurationCommand msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendCBRNScanWindowOperatingStateAction(QueryCBRNScanWindowOperatingState msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}

void CBRNSensor_ReceiveFSM::sendRNDetailedMeasurementAction(QueryRNDetailedMeasurement msg, Receive::Body::ReceiveRec transportData)
{
	/// Insert User Code HERE
}



bool CBRNSensor_ReceiveFSM::isControllingClient(Receive::Body::ReceiveRec transportData)
{
	//// By default, inherited guards call the parent function.
	//// This can be replaced or modified as needed.
	return pAccessControl_ReceiveFSM->isControllingClient(transportData );
}
bool CBRNSensor_ReceiveFSM::isSupported(CBRNCommand msg)
{
	/// Insert User Code HERE
	return false;
}
bool CBRNSensor_ReceiveFSM::isSupported(CBRNConfigurationCommand msg)
{
	/// Insert User Code HERE
	return false;
}
bool CBRNSensor_ReceiveFSM::isSupported(CBRNDisplayConfigurationCommand msg)
{
	/// Insert User Code HERE
	return false;
}
bool CBRNSensor_ReceiveFSM::isSupported(CBRNScanWindowConfigurationCommand msg)
{
	/// Insert User Code HERE
	return false;
}

void CBRNSensor_ReceiveFSM::measurementReceived(const fkie_iop_msgs::msg::Measurement::SharedPtr measurement)
{
	// ReportMeasurement msg;
	// msg.getBody()->getMeasurementSeq()->getDeviceRec()->setDeviceName(measurement->device_name);
	// if (!measurement->device_designation.empty()) {
	// 	msg.getBody()->getMeasurementSeq()->getDeviceRec()->setDeviceDesignation(measurement->device_designation);
	// }
	// if (!measurement->classification.empty()) {
	// 	msg.getBody()->getMeasurementSeq()->getDeviceRec()->setClassification(measurement->classification);
	// }
	// if (!isnan(measurement->latitude) && !isnan(measurement->longitude)) {
	// 	msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setLatitude(measurement->latitude);
	// 	msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setLongitude(measurement->longitude);
	// }
	// if (!isnan(measurement->altitude)) {
	// 	msg.getBody()->getMeasurementSeq()->getGlobalPoseRec()->setAltitude(measurement->altitude);
	// }
	// for (unsigned int i = 0; i < measurement->values.size() && i < 255; i++) {
	// 	ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq valseq;
	// 	auto mval = measurement->values[i];
	// 	valseq.getReadingRec()->setSensor(mval.sensor);
	// 	valseq.getReadingRec()->setSource(mval.source);
	// 	valseq.getReadingRec()->setType(mval.type);
	// 	valseq.getReadingRec()->setUnit(mval.unit);
	// 	valseq.getReadingRec()->setMinimum(mval.min);
	// 	valseq.getReadingRec()->setMaximum(mval.max);
	// 	valseq.getReadingRec()->setAvarage(mval.avg);
	// 	valseq.getReadingRec()->setAlertLevel(mval.alert_level);
	// 	valseq.getReadingRec()->setAlertExplanation(mval.alert_explanation);
	// 	valseq.getReadingRec()->setExtendedInfo(mval.extended_info);
	// 	for (unsigned int v = 0; v < mval.value.size() && v < 255; v++) {
	// 		ReportMeasurement::Body::MeasurementSeq::ReadingsList::ReadingSeq::ValueList::ValueRec valrec;
	// 		valrec.setValue(mval.value[v]);
	// 		valseq.getValueList()->addElement(valrec);
	// 	}
	// 	msg.getBody()->getMeasurementSeq()->getReadingsList()->addElement(valseq);
	// }
	// p_report_measurement = msg;
	// pEvents_ReceiveFSM->get_event_handler().set_report(QueryMeasurement::ID, &p_report_measurement);
}

}
