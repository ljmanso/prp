#ifndef _POINTCLOUD_H
#define _POINTCLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <qmat/qmat.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/time.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>



typedef pcl::PointXYZRGB PointT;

using namespace std;
using namespace RMat;

namespace computepointcloud
{
	pcl::PointCloud< PointT >::Ptr copy_pointcloud(pcl::PointCloud< PointT >::Ptr cloud);
	pcl::PointCloud< PointT >::Ptr PointCloudfrom_Meter_to_mm(pcl::PointCloud< PointT >::Ptr cloud);
	pcl::PointCloud< PointT >::Ptr PointCloudfrom_mm_to_Meters(pcl::PointCloud< PointT >::Ptr cloud);
	void moveToZero(pcl::PointCloud< PointT >::Ptr cloud, double &xMean, double &yMean, double &zMean);
	QMat fittingRSCP(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr reference,pcl::PointCloud<PointT>::Ptr &aligned);
	QMat fittingICP(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr reference,pcl::PointCloud<PointT>::Ptr &aligned);
	pcl::PointCloud< PointT >::Ptr VoxelGrid_filter(pcl::PointCloud< PointT >::Ptr cloud,float lx, float ly, float lz);
// 	void recuperateObjects(pcl::PointCloud< PointT >::Ptr scene, std::vector<pcl::PointCloud<PointT>::Ptr> cluster_clouds)
	pcl::PointCloud< PointT >::Ptr Filter_in_axis(pcl::PointCloud< PointT >::Ptr cloud, string axi, float min, float max, bool negative);
	void getBoundingBox(pcl::PointCloud< PointT >::Ptr cloud, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z);

};

#endif
