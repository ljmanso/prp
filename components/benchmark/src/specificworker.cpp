/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
	connect(mugButton,       SIGNAL(clicked()), this, SLOT(on_mugButton_clicked()));
	connect(laptopButton,    SIGNAL(clicked()), this, SLOT(on_laptopButton_clicked()));
	connect(wrenchButton,    SIGNAL(clicked()), this, SLOT(on_wrenchButton_clicked()));
	connect(ballButton,      SIGNAL(clicked()), this, SLOT(on_ballButton_clicked()));
	connect(glassesButton,   SIGNAL(clicked()), this, SLOT(on_glassesButton_clicked()));
	connect(keysButton,      SIGNAL(clicked()), this, SLOT(on_keysButton_clicked()));
	connect(bottleButton,    SIGNAL(clicked()), this, SLOT(on_bottleButton_clicked()));
	connect(canButton,       SIGNAL(clicked()), this, SLOT(on_canButton_clicked()));
	connect(bookButton,      SIGNAL(clicked()), this, SLOT(on_bookButton_clicked()));
	connect(cellphoneButton, SIGNAL(clicked()), this, SLOT(on_cellphoneButton_clicked()));
	connect(appleButton,     SIGNAL(clicked()), this, SLOT(on_appleButton_clicked()));
	connect(walletButton,    SIGNAL(clicked()), this, SLOT(on_walletButton_clicked()));


	cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
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
	static RoboCompDifferentialRobot::TBaseState bState;
	static RoboCompJointMotor::MotorStateMap hState;
	static RoboCompRGBD::ColorSeq rgbMatrix;
	static RoboCompRGBD::DepthSeq distanceMatrix;
	
	try
	{
		qDebug() << "read frame";

		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points,         hState, bState);
// 		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points,  h, b);
		
		frame = cv::Mat(480, 640, CV_8UC3,  &(rgbMatrix)[0]);
//         imshow("3D viewer",frame);
		
		QImage img = QImage((uint8_t *)&rgbMatrix[0], 640, 480, QImage::Format_RGB888);
		label->setPixmap(QPixmap::fromImage(img));
		label->resize(label->pixmap()->size());

		cloud->points.resize(points.size());
		for (unsigned int i=0; i<points.size(); i++)
		{
	// 		memcpy(&cloud->points[i], &points[i],3*sizeof(float));
			
			cloud->points[i].x = points[i].x;
			cloud->points[i].y = points[i].y;
			cloud->points[i].z = points[i].z;
			cloud->points[i].r=rgbMatrix[i].red;
			cloud->points[i].g=rgbMatrix[i].green;
			cloud->points[i].b=rgbMatrix[i].blue;
		}
		cloud->width = 1;
		cloud->height = points.size();


	}
	catch(const Ice::Exception &ex)
	{
		std::cout << ex << std::endl;
	}
}



void SpecificWorker::save(std::string base)
{
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string basename = lineEdit->text().toStdString() + "_" + base  + "_" + QString::number(ts.tv_sec).toStdString();
	writer.write<PointT>(basename + ".pcd", *cloud, false);
	cv::imwrite(basename + ".png", frame);
}




