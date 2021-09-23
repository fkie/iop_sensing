

#ifndef CBRNSENSOR_RECEIVEFSM_H
#define CBRNSENSOR_RECEIVEFSM_H

#include "JausUtils.h"
#include "InternalEvents/InternalEventHandler.h"
#include "Transport/JausTransport.h"
#include "JTSStateMachine.h"
#include "urn_jaus_jss_iop_CBRNSensor/Messages/MessageSet.h"
#include "urn_jaus_jss_iop_CBRNSensor/InternalEvents/InternalEventsSet.h"

#include "InternalEvents/Receive.h"
#include "InternalEvents/Send.h"

#include "urn_jaus_jss_core_AccessControl/AccessControl_ReceiveFSM.h"
#include "urn_jaus_jss_core_Events/Events_ReceiveFSM.h"
#include "urn_jaus_jss_core_Transport/Transport_ReceiveFSM.h"


#include <fkie_iop_msgs/msg/measurement.hpp>

#include "CBRNSensor_ReceiveFSM_sm.h"
#include <rclcpp/rclcpp.hpp>
#include <fkie_iop_component/iop_component.hpp>


namespace urn_jaus_jss_iop_CBRNSensor
{

class DllExport CBRNSensor_ReceiveFSM : public JTS::StateMachine
{
public:
	CBRNSensor_ReceiveFSM(std::shared_ptr<iop::Component> cmp, urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM, urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM, urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM);
	virtual ~CBRNSensor_ReceiveFSM();

	/// Handle notifications on parent state changes
	virtual void setupNotifications();
	virtual void setupIopConfiguration();


	/// Action Methods
	virtual void applyCBRNCommandAction(CBRNCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void applyCBRNConfigurationAction(CBRNConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void applyCBRNDisplayConfigurationAction(CBRNDisplayConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void applyCBRNScanWindowConfigurationAction(CBRNScanWindowConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBDetailedMeasurementAction(QueryCBDetailedMeasurement msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRN4Action(QueryCBRN4 msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRN4Action(Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNConfigurationAction(QueryCBRNConfiguration msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNConfigurationStateAction(CBRNConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNConfigurationStateAction(CBRNDisplayConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNDetectionAction(QueryCBRNDetection msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNDisplayConfigurationAction(QueryCBRNOperatingState msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNOperatingStateAction(CBRNCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNOperatingStateAction(QueryCBRNOperatingState msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNScanWindowOperatingStateAction(CBRNScanWindowConfigurationCommand msg, Receive::Body::ReceiveRec transportData);
	virtual void sendCBRNScanWindowOperatingStateAction(QueryCBRNScanWindowOperatingState msg, Receive::Body::ReceiveRec transportData);
	virtual void sendRNDetailedMeasurementAction(QueryRNDetailedMeasurement msg, Receive::Body::ReceiveRec transportData);


	/// Guard Methods
	virtual bool isControllingClient(Receive::Body::ReceiveRec transportData);
	virtual bool isSupported(CBRNCommand msg);
	virtual bool isSupported(CBRNConfigurationCommand msg);
	virtual bool isSupported(CBRNDisplayConfigurationCommand msg);
	virtual bool isSupported(CBRNScanWindowConfigurationCommand msg);


	CBRNSensor_ReceiveFSMContext *context;
	
protected:

	/// References to parent FSMs
	urn_jaus_jss_core_AccessControl::AccessControl_ReceiveFSM* pAccessControl_ReceiveFSM;
	urn_jaus_jss_core_Events::Events_ReceiveFSM* pEvents_ReceiveFSM;
	urn_jaus_jss_core_Transport::Transport_ReceiveFSM* pTransport_ReceiveFSM;

	std::shared_ptr<iop::Component> cmp;
	rclcpp::Logger logger;
	rclcpp::Subscription<fkie_iop_msgs::msg::Measurement>::SharedPtr p_measurement_sub;

	void measurementReceived(const fkie_iop_msgs::msg::Measurement::SharedPtr measurement);

};

}

#endif // CBRNSENSOR_RECEIVEFSM_H
