#include "viewer.h"



Viewer::Viewer(float medida)
{
#ifdef USE_QTGUI
	MEDIDA=medida;
	viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0.2, 0.2, 0.2);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (0.4);
#endif
}

void Viewer::addCoordinateSystem(float x,float y, float z, std::string id)
{
#ifdef USE_QTGUI
	viewer->addCoordinateSystem(0.25,x,y,z,id,0);
#endif
}

void Viewer::addCoordinateSystem(RMat::QMat tr, std::string id)
{
#ifdef USE_QTGUI
	Eigen::Affine3f t;
	for (int c=0; c<4; c++)
		for (int r=0; r<4; r++)
			t(r,c) = tr(r,c);
	viewer->addCoordinateSystem(0.25, t,id,0);
#endif
}

void Viewer::addPointCloud(pcl::PointCloud< PointT >::Ptr cloud, std::string id, int size, int r, int g, int b )
{
#ifdef USE_QTGUI
	pcl::PointCloud< PointT >::Ptr out;
	if(MEDIDA==1000.)
		out = copy_pointcloud(cloud);
	else
		out = PointCloudfrom_mm_to_Meters(copy_pointcloud(cloud));
	if(r==0 and b==0 and g==0)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(out);
		viewer->addPointCloud<pcl::PointXYZRGB> (out, rgb, id);
	}
	else
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb  (out, r, g, b);
		viewer->addPointCloud<pcl::PointXYZRGB> (out, rgb, id);
	}
// 	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
#endif
}

void Viewer::updatePointCloud(pcl::PointCloud< PointT >::Ptr cloud, std::string id)
{
#ifdef USE_QTGUI
	pcl::PointCloud< PointT >::Ptr out;
	if(MEDIDA==1000.)
		out = copy_pointcloud(cloud);
	else
		out = PointCloudfrom_mm_to_Meters(copy_pointcloud(cloud));
	viewer->updatePointCloud(out, id);
#endif
}

void Viewer::updateCoordinateSystemPose(RMat::QMat tr, std::string id)
{
#ifdef USE_QTGUI
	Eigen::Affine3f t;
	for (int c=0; c<4; c++)
		for (int r=0; r<4; r++)
			t(r,c) = tr(r,c);
	viewer->updateCoordinateSystemPose(id, t);
#endif
}

void Viewer::removePointCloud(std::string id)
{
#ifdef USE_QTGUI
	viewer->removePointCloud(id,0);
#endif
}

void Viewer::removeCoordinateSystem(std::string id)
{
#ifdef USE_QTGUI
	viewer->removeCoordinateSystem(id,0);
#endif
}

void Viewer::update()
{
#ifdef USE_QTGUI
	viewer->spinOnce (10);
#endif
}

void Viewer::addCube(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, std::string id)
{
#ifdef USE_QTGUI

	pcl::PointXYZ pt1 (min_x/1000., min_y/1000., min_z/1000.);
	pcl::PointXYZ pt2 (min_x/1000., min_y/1000., max_z/1000.);
	pcl::PointXYZ pt3 (max_x/1000., min_y/1000., max_z/1000.);
	pcl::PointXYZ pt4 (max_x/1000., min_y/1000., min_z/1000.);
	pcl::PointXYZ pt5 (min_x/1000., max_y/1000., min_z/1000.);
	pcl::PointXYZ pt6 (min_x/1000., max_y/1000., max_z/1000.);
	pcl::PointXYZ pt7 (max_x/1000., max_y/1000., max_z/1000.);
	pcl::PointXYZ pt8 (max_x/1000., max_y/1000., min_z/1000.);

	viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1" + id );
	viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2" + id );
	viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3" + id );
	viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4" + id );
	viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5" + id );
	viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6" + id );
	viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7" + id );
	viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8" + id );
	viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9" + id );
	viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10" + id );
	viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11" + id );
	viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12" + id );

#endif
}

void Viewer::removeAllShapes()
{
#ifdef USE_QTGUI
	viewer->removeAllShapes();
#endif
}

void Viewer::removeCube(std::string id)
{
	#ifdef USE_QTGUI
	for(int i = 1; i<=12; i++)
	{
		viewer->removeShape(QString::number(i).toStdString()+id);
	}
	#endif
}
