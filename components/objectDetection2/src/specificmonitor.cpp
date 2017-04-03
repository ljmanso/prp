/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificmonitor.h"
/**
* \brief Default constructor
*/
SpecificMonitor::SpecificMonitor(GenericWorker *_worker,Ice::CommunicatorPtr _communicator):GenericMonitor(_worker, _communicator)
{
		ready = false;
}
/**
* \brief Default destructor
*/
SpecificMonitor::~SpecificMonitor()
{

}

void SpecificMonitor::run()
{
	initialize();
	ready = true;
	forever
	{
		//rDebug("specific monitor run");
		this->sleep(period);
	}
}

/**
 * \brief Reads components parameters and checks set integrity before signaling the Worker thread to start running
 *   (1) Ice parameters
 *   (2) Local component parameters read at start
 *
 */
void SpecificMonitor::initialize()
{
	rInfo("Starting monitor ...");
	initialTime=QTime::currentTime();
	RoboCompCommonBehavior::ParameterList params;
	readPConfParams(params);
	readConfig(params);
	if(!sendParamsToWorker(params))
	{
		rError("Error reading config parameters. Exiting");
		killYourSelf();
	}
	state = RoboCompCommonBehavior::Running;
}

bool SpecificMonitor::sendParamsToWorker(RoboCompCommonBehavior::ParameterList params)
{
	if(checkParams(params))
	{
		//Set params to worker
		if(worker->setParams(params)) 
			return true;
	}
	else
	{
		rError("Incorrect parameters");
	}
	return false;

}

///Local Component parameters read at start
///Reading parameters from config file or passed in command line, with Ice machinery
///We need to supply a list of accepted values to each call
void SpecificMonitor::readConfig(RoboCompCommonBehavior::ParameterList &params )
{
	RoboCompCommonBehavior::Parameter aux;
	aux.editable = true;
	string name = PROGRAM_NAME;
	
	configGetString(name,"innermodel", aux.value, "default");
	//Check valid ranges
	if( aux.value =="default")
	{
		std::cout << __FUNCTION__ << name<<".innermodel value is default" << std::endl;
		exit(-1);
	}
	params[name+".innermodel"] = aux;
// 	----------------------------------
	configGetString(name,"id_robot", aux.value, "default");
	//Check valid ranges
	if( aux.value =="default")
	{
		std::cout << __FUNCTION__ << name<<".id_robot value is default" << std::endl;
		aux.value="robot";	
	}
	params[name+".id_robot"] = aux;
// 	----------------------------------	
	configGetString(name,"id_camera_transform", aux.value, "default");
	//Check valid ranges
	if( aux.value =="default")
	{
		std::cout << __FUNCTION__ << name<<".id_camera_transform value is default" << std::endl;
		aux.value="rgbd_transform";
	}
	params[name+".id_camera_transform"] = aux;
// 	----------------------------------
	configGetString(name,"id_camera", aux.value, "default");
	//Check valid ranges
	if( aux.value =="default")
	{
		std::cout << __FUNCTION__ << name<<".id_camera value is default" << std::endl;
		aux.value="rgbd";
	}
	params[name+".id_camera"] = aux;
// 	----------------------------------
	configGetString(name,"type_features", aux.value, "default");
	//Check valid ranges
	if( aux.value =="default"||(aux.value !="VFH"&&aux.value !="CVFH"&&aux.value !="OUR-CVFH"))
	{
		std::cout << __FUNCTION__ << name<<".type_features value is default" << std::endl;
		aux.value="VFH";	
	}
	params[name+".type_features"] = aux;
	
	configGetString(name,"pathLoadDescriptors", aux.value, "default");
	//Check valid ranges
	params[name+".pathLoadDescriptors"] = aux;
	
	configGetString(name,"test", aux.value, "default");
	//Check valid ranges
	params[name+".test"] = aux;
}

//comprueba que los parametros sean correctos y los transforma a la estructura del worker
bool SpecificMonitor::checkParams(RoboCompCommonBehavior::ParameterList l)
{
	bool correct = true;
	return correct;
}

