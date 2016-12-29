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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	QLineEdit* x[9]={x1,x2,x3,x4,x5,x6,x7,x8,x9};
	QLineEdit* y[9]={y1,y2,y3,y4,y5,y6,y7,y8,y9};
	QLineEdit* z[9]={z1,z2,z3,z4,z5,z6,z7,z8,z9};
public slots:
	void compute(); 
	void grabThePointCloud();	
	void ransac();
	void projectInliers();
	void convexHull();
	void extractPolygon();
	void euclideanExtract();
	void findTheObject();
	void getPose();
	void getRotation();
	void reloadVFH();
	void fullRun();
private:
	
};

#endif

