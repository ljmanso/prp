#ifndef _VIEWER_H
#define _VIEWER_H

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <QTimer>
#include <qmat/qmat.h>
#include "config.h"
#include "pointcloud/pointcloud.h"
#include <pcl/common/common.h>

typedef pcl::PointXYZRGB PointT;

using namespace computepointcloud;

class Viewer
{
private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	QTimer timerrun;
	float MEDIDA;
public:
	Viewer(float medida);
	void addCoordinateSystem(float x,float y, float z, std::string id);
	void addCoordinateSystem(RMat::QMat tr, std::string id);
	void addPointCloud(pcl::PointCloud< PointT >::Ptr cloud, std::string id, int size, int r, int g, int b);
	void updatePointCloud(pcl::PointCloud< PointT >::Ptr cloud, std::string id);
	void updateCoordinateSystemPose(RMat::QMat tr, std::string id);
	void removePointCloud(std::string id);
	void removeCoordinateSystem(std::string id);
	void update();
	void addCube(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id);
	void removeCube(std::string id);
	void removeAllShapes();
	void addText3D(std::string text, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id);
	void removeText(std::string id);
};


#endif
