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
 	objectdetection_proxy->reloadVFH("/home/robocomp/robocomp/components/prp/objects/");
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
	x_object->setText(QString::number(x));
	y_object->setText(QString::number(y));
	z_object->setText(QString::number(z));
}

void SpecificWorker::getRotation()
{
	float rx, ry, rz;
	objectdetection_proxy->getRotation(rx, ry, rz);
	qDebug()<<rx<<", "<<ry<<", "<<rz;
}

// void SpecificWorker::getCanonicalPose()
// {
// 	
// }

void SpecificWorker::fullRun()
{
	string label=label_le->text().toStdString();
	char *c;
	pose6D tags[9],guess;
	int numOfClusters = 0;
	objectdetection_proxy->grabThePointCloud("image.png", "rgbd.pcd");
	objectdetection_proxy->ransac("plane");
	objectdetection_proxy->projectInliers("plane");
	objectdetection_proxy->convexHull("plane");
	objectdetection_proxy->extractPolygon("plane");
	objectdetection_proxy->euclideanClustering(numOfClusters);
	for(int i=0; i<9; i++)
	{
		tags[i].tx=0;
		tags[i].ty=0;
		tags[i].tz=0;
		
		tags[i].rx=0;
		tags[i].ry=0;
		tags[i].rz=0;
	}
	string s="mkdir /home/robocomp/robocomp/components/prp/objects/"+label;
	c= &s[0u];
	/*{
	tags[0].tx=-172;tags[0].ty=0;tags[0].tz=172;
	tags[0].rx=0;tags[0].ry=0;tags[0].rz=0;
	
	tags[1].tx=0;tags[1].ty=0;tags[1].tz=172;
	tags[1].rx=0;tags[1].ry=0;tags[1].rz=0;
	
	tags[2].tx=172;tags[2].ty=0;tags[2].tz=172;
	tags[2].rx=0;tags[2].ry=0;tags[2].rz=0;
	
	tags[3].tx=-172;tags[3].ty=0;tags[3].tz=0;
	tags[3].rx=0;tags[3].ry=0;tags[3].rz=0;
	
	tags[4].tx=0;tags[4].ty=0;tags[4].tz=0;
	tags[4].rx=0;tags[4].ry=0;tags[4].rz=0;
	
	tags[5].tx=172;tags[5].ty=0;tags[5].tz=0;
	tags[5].rx=0;tags[5].ry=0;tags[5].rz=0;
	
	tags[6].tx=-172;tags[6].ty=0;tags[6].tz=-172;
	tags[6].rx=0;tags[6].ry=0;tags[6].rz=0;
	
	tags[7].tx=0;tags[7].ty=0;tags[7].tz=-172;
	tags[7].rx=0;tags[7].ry=0;tags[7].rz=0;
	
	tags[8].tx=172;tags[8].ty=0;tags[8].tz=-172;
	tags[8].rx=0;tags[8].ry=0;tags[8].rz=0;
	}
	*/
	system(c);
	if(canonPoseRb->isChecked())
		objectdetection_proxy->saveCanonPose(label,ob_to_save->value(),tags[0],tags[1],tags[2],tags[3],tags[4],tags[5],tags[6],tags[7],tags[8]);
	if(regularPose->isChecked())
		objectdetection_proxy->saveRegPose(label,ob_to_save->value(),tags[0],tags[1],tags[2],tags[3],tags[4],tags[5],tags[6],tags[7],tags[8]);
	if(ObtainPose->isChecked())
		objectdetection_proxy->guessPose(label,guess);
	std::cout<<guess.tx<<endl;
	std::cout<<guess.ty<<endl;
	std::cout<<guess.tz<<endl;
	std::cout<<guess.rx<<endl;
	std::cout<<guess.ry<<endl;
	std::cout<<guess.tz<<endl;
// 	int i=0;
// 	for (auto ap : tags)
// 	{
// 		x[i]->setText(QString::number(ap.tx));
// 		y[i]->setText(QString::number(ap.ty));
// 		z[i]->setText(QString::number(ap.tz));
// 		i++;
// 	}
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







