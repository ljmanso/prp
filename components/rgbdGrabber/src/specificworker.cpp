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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	frame = 0;
	connect(pushButton, SIGNAL(clicked()), this, SLOT(store()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
}


void writePCD(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	printf("Writing: %s  width:%d height:%d points:%d\n", path.c_str(), (int)cloud->width, (int)cloud->height, (int)cloud->points.size());
	cloud->width = 1;
	cloud->height = cloud->points.size();
	static pcl::PCDWriter writer;
	if (not cloud->empty()) writer.writeASCII(path, *cloud);
}


void SpecificWorker::store()
{
	RoboCompRGBD::ColorSeq color;
	RoboCompRGBD::DepthSeq depth;
	RoboCompRGBD::PointSeq points;
	RoboCompJointMotor::MotorStateMap hState;
	RoboCompDifferentialRobot::TBaseState bState;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	rgbd_proxy->getImage(color, depth, points, hState, bState);

	cloud->height = 1;
	cloud->width = points.size();
	cloud->points.resize(points.size());
	for (uint32_t ioi=0; ioi<points.size(); ioi++)
	{
		cloud->points[ioi].x =  points[ioi].x;
		cloud->points[ioi].y =  points[ioi].y;
		cloud->points[ioi].z = -points[ioi].z;
		uint32_t rgb = ((uint32_t)color[ioi].red << 16 | (uint32_t)color[ioi].green << 8 | (uint32_t)color[ioi].blue);
		cloud->points[ioi].rgb = *reinterpret_cast<float*>(&rgb);
	}
	
	
	QString numberToStr = QString("%1").arg(frame++, 5, 10, QChar('0'));
	writePCD(numberToStr.toStdString()+".pcd", cloud);
	QImage((uchar *)&(color[0]), 640, 480, QImage::Format_RGB888).save(numberToStr+".png");
		
}




