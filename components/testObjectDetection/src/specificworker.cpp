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
	connect(grab, SIGNAL(clicked()), this, SLOT(grabThePointCloud()));
	connect(rans, SIGNAL(clicked()), this, SLOT(ransac()));
	connect(project, SIGNAL(clicked()), this, SLOT(projectInliers()));
	connect(convex, SIGNAL(clicked()), this, SLOT(convexHull()));
	connect(extract, SIGNAL(clicked()), this, SLOT(extractPolygon()));
	connect(euclidean, SIGNAL(clicked()), this, SLOT(euclideanExtract()));
	connect(findObject, SIGNAL(clicked()), this, SLOT(findTheObject()));
	connect(pose, SIGNAL(clicked()), this, SLOT(getPose()));
	connect(rotation, SIGNAL(clicked()), this, SLOT(getRotation()));
	connect(reload, SIGNAL(clicked()), this, SLOT(reloadVFH()));
	connect(go, SIGNAL(clicked()), this, SLOT(fullRun()));
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

void  SpecificWorker::grabThePointCloud()
{
	objectdetection_proxy->grabThePointCloud("image.png", "rgbd.pcd");
}

void  SpecificWorker::ransac()
{
	objectdetection_proxy->ransac("plane");
}

void SpecificWorker::projectInliers()
{
	objectdetection_proxy->projectInliers("plane");
}
	
void SpecificWorker::convexHull()
{
	objectdetection_proxy->convexHull("plane");
}

	
void SpecificWorker::extractPolygon()
{
	objectdetection_proxy->extractPolygon("plane");
}

void SpecificWorker::euclideanExtract()
{
	int numOfClusters = 0;
	objectdetection_proxy->euclideanClustering(numOfClusters);
}

void SpecificWorker::reloadVFH()
{
	objectdetection_proxy->reloadVFH("/home/robocomp/robocomp/components/prp/experimentFiles/vfhSignatures/");
}

void SpecificWorker::findTheObject()
{
	std::string object = text_object->toPlainText().toStdString();
	objectdetection_proxy->findTheObject(object);
}

void SpecificWorker::getPose()
{
	float x, y, z;
	objectdetection_proxy->getPose(x, y, z);
}

void SpecificWorker::getRotation()
{
	float rx, ry, rz;
	objectdetection_proxy->getRotation(rx, ry, rz);
}

void SpecificWorker::getCanonicalPose()
{
	
}

void SpecificWorker::compute()
{
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}







