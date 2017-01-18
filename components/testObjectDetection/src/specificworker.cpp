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
// 	connect(grab, SIGNAL(clicked()), this, SLOT(grabThePointCloud()));
// 	connect(rans, SIGNAL(clicked()), this, SLOT(ransac()));
// 	connect(project, SIGNAL(clicked()), this, SLOT(projectInliers()));
// 	connect(convex, SIGNAL(clicked()), this, SLOT(convexHull()));
// 	connect(extract, SIGNAL(clicked()), this, SLOT(extractPolygon()));
// 	connect(euclidean, SIGNAL(clicked()), this, SLOT(euclideanExtract()));
	connect(findObject, SIGNAL(clicked()), this, SLOT(findTheObject()));
// 	connect(pose, SIGNAL(clicked()), this, SLOT(getPose()));
// 	connect(rotation, SIGNAL(clicked()), this, SLOT(getRotation()));
	connect(reload, SIGNAL(clicked()), this, SLOT(reloadVFH()));
//  	connect(go, SIGNAL(clicked()), this, SLOT(fullRun()));
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	reloadVFH();
	timer.start(5000);
	return true;
}
/*
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
}*/

void SpecificWorker::reloadVFH()
{
 	objectdetection_proxy->reloadVFH("/home/robocomp/robocomp/components/prp/objects/");
}

void SpecificWorker::findTheObject()
{
	std::string object = text_object->toPlainText().toStdString();
	pose6D poseObj;
	bool result=objectdetection_proxy->findTheObject(object, poseObj);
	if(object!="")
	{
		isObject->setVisible(true);
		if(result)
		{
			isObject->setText("El Objeto SI esta en la mesa.");
			x_object->setText(QString::number(poseObj.tx));
			y_object->setText(QString::number(poseObj.ty));
			z_object->setText(QString::number(poseObj.tz));
			rx_object->setText(QString::number(poseObj.rx));
			ry_object->setText(QString::number(poseObj.ry));
			rz_object->setText(QString::number(poseObj.rz));
		}
		else
			isObject->setText("El Objeto NO esta en la mesa.");
	}
	else
		isObject->setVisible(false);
}

// void SpecificWorker::getPose()
// {
// 	pose6D poseObj;
// 	poseObj=objectdetection_proxy->getPose();
// 	x_object->setText(QString::number(poseObj.tx));
// 	y_object->setText(QString::number(poseObj.ty));
// 	z_object->setText(QString::number(poseObj.tz));
// 	rx_object->setText(QString::number(poseObj.rx));
// 	ry_object->setText(QString::number(poseObj.ry));
// 	rz_object->setText(QString::number(poseObj.rz));
// }
// 
// void SpecificWorker::getRotation()
// {
// // 	float rx, ry, rz;
// // 	objectdetection_proxy->getRotation(rx, ry, rz);
// // // 	qDebug()<<rx<<", "<<ry<<", "<<rz;
// // 	rx_object->setText(QString::number(rx));
// // 	ry_object->setText(QString::number(ry));
// // 	rz_object->setText(QString::number(rz));
// 	
// }
// 
// // void SpecificWorker::getCanonicalPose()
// // {
// // 	
// // }

void SpecificWorker::fullRun()
{
	string label=label_le->text().toStdString();
	char *c;
	pose6D guess;
// 	int numOfClusters = 0;
// 	objectdetection_proxy->grabThePointCloud("image.png", "rgbd.pcd");
// 	objectdetection_proxy->ransac("plane");
// 	objectdetection_proxy->projectInliers("plane");
// 	objectdetection_proxy->convexHull("plane");
// 	objectdetection_proxy->extractPolygon("plane");
// 	objectdetection_proxy->euclideanClustering(numOfClusters);
	string s="mkdir /home/robocomp/robocomp/components/prp/objects/"+label;
	c= &s[0u];
	
	system(c);
	if(canonPoseRb->isChecked())
		objectdetection_proxy->saveCanonPose(label,ob_to_save->value());
	if(regularPose->isChecked())
		objectdetection_proxy->saveRegPose(label,ob_to_save->value());
// 	if(ObtainPose->isChecked())
// 		objectdetection_proxy->guessPose(label,guess);
// 	std::cout<<guess.tx<<endl;
// 	std::cout<<guess.ty<<endl;
// 	std::cout<<guess.tz<<endl;
// 	std::cout<<guess.rx<<endl;
// 	std::cout<<guess.ry<<endl;
// 	std::cout<<guess.tz<<endl;
}


void SpecificWorker::compute()
{
// 	try
// 	{
// 		fullRun();
// 		findTheObject();
// 	}
// 	catch(...){}
}







