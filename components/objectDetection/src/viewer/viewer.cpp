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
		out = PointCloudfrom_mm_to_Meters(cloud);
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
		out = PointCloudfrom_mm_to_Meters(cloud);
	viewer->updatePointCloud(out, id);
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
