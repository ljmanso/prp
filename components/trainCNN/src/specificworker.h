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

#include <boost/filesystem.hpp>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void read_CNN_config(const string config_file);
	bool first;
	std::string img_dir_file = "data/images";
	std::string train_list_file = "data/train.txt";
	std::string train_list_lmdb = "data/train.lmdb"; 
	std::string test_list_file = "data/test.txt";
	std::string mean_file= "mean.binaryproto";
	std::string solver_file = "model/myquick_solver.prototxt";
	std::string weights_file = "model/VGG_ILSVRC_16_layers.caffemodel";
	std::string model_file = "model/mytrain_val.prototxt";
	std::string gpuids = "";
	std::string snapshot_file = "";
	

public slots:
	void compute(); 	

private:
	
};

#endif

