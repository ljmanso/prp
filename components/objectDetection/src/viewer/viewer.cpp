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

void Viewer::addCube(pcl::PointCloud< PointT >::Ptr cloud, std::string id)
{
#ifdef USE_QTGUI
	Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // move the points to the that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3,3>(0,0) = eigDx.transpose();
  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
  pcl::PointCloud<PointT> cPoints;
  pcl::transformPointCloud(*cloud, cPoints, p2w);

  PointT min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // final transform
  const Eigen::Quaternionf qfinal(eigDx);
  const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

  viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, id);
#endif
}

void Viewer::removeAllShapes()
{
#ifdef USE_QTGUI
	viewer->removeAllShapes();
#endif
}

void Viewer::removeShape(std::string id)
{
	#ifdef USE_QTGUI
		viewer->removeShape(id);
	#endif
}
