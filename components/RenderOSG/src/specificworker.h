/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <boost/filesystem.hpp>


#include "Renderer.hpp"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	bool first;
	
	int viewport_size_x = 480; ///image width
	int viewport_size_y = 480; ///image height
	int Nbin_azimuth=10;
	int start_azimuth=-180;   ///azimuth=start_azimuth+360*(current_bin/Nbin_azimuth) 
	int Nbin_elevation=10;    ///all angles in degrees
	int start_elevation=-180;
	int Nbin_yaw=10;
	int start_yaw=-180;
    std::string modelfile_name="";
    std::string output_image_directory_name="";
    std::string output_depth_directory_name="";	

void read_renderer_config(const std::string config_file);
public slots:
	void compute(); 	

private:
	
};

#endif

