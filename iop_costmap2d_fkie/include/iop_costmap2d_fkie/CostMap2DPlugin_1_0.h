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


#ifndef COSTMAP2DPLUGIN_1_0_H
#define COSTMAP2DPLUGIN_1_0_H

#include "urn_jaus_jss_iop_CostMap2D_1_0/CostMap2DService.h"
#include "urn_jaus_jss_core_AccessControl_1_0/AccessControlService.h"
#include "urn_jaus_jss_core_Events_1_0/EventsService.h"
#include "urn_jaus_jss_core_Transport_1_0/TransportService.h"

#include <iop_component_fkie/iop_plugin_interface.h>

namespace iop
{

class DllExport CostMap2DPlugin_1_0 : public PluginInterface
{
public:
	CostMap2DPlugin_1_0();

	JTS::Service* get_service();
	void create_service(JTS::JausRouter* jaus_router);

protected:
	urn_jaus_jss_iop_CostMap2D_1_0::CostMap2DService *p_my_service;
	urn_jaus_jss_core_AccessControl_1_0::AccessControlService *p_base_service;
	urn_jaus_jss_core_Events_1_0::EventsService *p_events_service;
	urn_jaus_jss_core_Transport_1_0::TransportService *p_transport_service;

};

};

#endif
