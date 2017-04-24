#include "pointcloud.h"

pcl::PointCloud< PointT >::Ptr computepointcloud::copy_pointcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
	pcl::PointCloud< PointT >::Ptr copy_cloud(new pcl::PointCloud<PointT>(*cloud));
	return copy_cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::PointCloudfrom_Meter_to_mm(pcl::PointCloud< PointT >::Ptr cloud)
{
	for(unsigned int i =0;i<cloud->points.size();i++)
	{
		cloud->points[i].x=cloud->points[i].x*1000.;
		cloud->points[i].y=cloud->points[i].y*1000.;
		cloud->points[i].z=cloud->points[i].z*1000.;
	}
	return cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::PointCloudfrom_mm_to_Meters(pcl::PointCloud< PointT >::Ptr cloud)
{
	for(unsigned int i =0;i<cloud->points.size();i++)
	{
		cloud->points[i].x=cloud->points[i].x/1000.;
		cloud->points[i].y=cloud->points[i].y/1000.;
		cloud->points[i].z=cloud->points[i].z/1000.;
	}
	return cloud;
}

void computepointcloud::moveToZero(pcl::PointCloud< PointT >::Ptr cloud, double &xMean, double &yMean, double &zMean)
{
// printf("%d\n", __LINE__);
	double xAcc=0, yAcc=0, zAcc=0;
	int acc=0;

// printf("%d\n", __LINE__);
	for (size_t i=0; i<cloud->points.size(); ++i)
	{
// printf("%d\n", __LINE__);
		if (isnan(cloud->points[i].x) or isnan(cloud->points[i].y) or isnan(cloud->points[i].z) )
			continue;
// printf("%d\n", __LINE__);
		acc  += 1;
		xAcc += cloud->points[i].x;
		yAcc += cloud->points[i].y;
		zAcc += cloud->points[i].z;
// printf("%d\n", __LINE__);
	}

// printf("%d\n", __LINE__);
	xMean = xAcc / double(acc);
	yMean = yAcc / double(acc);
	zMean = zAcc / double(acc);
// printf("%d\n", __LINE__);

	for (size_t i=0; i<cloud->points.size(); ++i)
	{
// printf("%d\n", __LINE__);
		if (isnan(cloud->points[i].x) or isnan(cloud->points[i].y) or isnan(cloud->points[i].z) )
			continue;
// printf("%d\n", __LINE__);
		cloud->points[i].x -= xMean;
		cloud->points[i].y -= yMean;
		cloud->points[i].z -= zMean;
// printf("%d\n", __LINE__);
	}
// printf("%d\n", __LINE__);
}

QMat computepointcloud::fittingICP(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr reference,pcl::PointCloud<PointT>::Ptr &aligned)
{
	pcl::PointCloud< PointT >::Ptr reference_copy = copy_pointcloud(reference);

	Eigen::Vector4f centroid_object, centroid_reference;
	pcl::compute3DCentroid (*object, centroid_object);
	pcl::compute3DCentroid (*reference_copy, centroid_reference);

	for(unsigned int i = 0; i< reference_copy->points.size(); i++)
	{
		reference_copy->points[i].x+=centroid_object[0] - centroid_reference[0];
		reference_copy->points[i].y+=centroid_object[1] - centroid_reference[1];
		reference_copy->points[i].z+=centroid_object[2] - centroid_reference[2];
	}
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputCloud(object);
	icp.setInputTarget(reference_copy);
	icp.setTransformationEpsilon (1e-12);
	icp.setEuclideanFitnessEpsilon (1e-12);
	icp.setMaxCorrespondenceDistance (0.05);
	icp.setMaximumIterations(500);
	icp.align(*aligned);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	Eigen::Matrix4f t=icp.getFinalTransformation();
	t(0,3)+= - centroid_object[0] + centroid_reference[0];
	t(1,3)+= - centroid_object[1] + centroid_reference[1];
	t(2,3)+= - centroid_object[2] + centroid_reference[2];
	QMat TM(4,4);
	for (int c=0; c<4; c++)
		for (int r=0; r<4; r++)
			TM(r,c) = t(r, c);
	return TM;
}

QMat computepointcloud::fittingRSCP(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr reference,pcl::PointCloud<PointT>::Ptr &aligned)
{
	pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);

	std::cout<<object->size()<<std::endl;
	std::cout<<reference->size()<<std::endl;
	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<PointT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (reference);
	grid.filter (*reference);

	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP< PointT ,pcl::Normal> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (object);
	nest.compute (*object_normals);
	nest.setInputCloud (reference);
	nest.compute (*scene_normals);

	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	pcl::FPFHEstimationOMP<PointT,pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object_normals);
	fest.compute (*object_features);
	fest.setInputCloud (reference);
	fest.setInputNormals (scene_normals);
	fest.compute (*scene_features);

	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (reference);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (100000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*aligned);
	}
	std::cout << "has converged:" << align.hasConverged() << " score: " << align.getFitnessScore() << std::endl;
	std::cout << align.getFinalTransformation() << std::endl;
	Eigen::Matrix4f t = align.getFinalTransformation();
	QMat TM(4,4);
	for (int c=0; c<4; c++)
		for (int r=0; r<4; r++)
			TM(r,c) = t(r, c);
	return TM;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::VoxelGrid_filter(pcl::PointCloud< PointT >::Ptr cloud, float lx, float ly, float lz)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*cloud);
	return cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::Filter_in_axis(pcl::PointCloud< PointT >::Ptr cloud, string axi, float min, float max, bool negative)
{
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName (axi);
	pass.setFilterLimits (min, max);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*cloud);
	return cloud;
}

void computepointcloud::getBoundingBox(pcl::PointCloud< PointT >::Ptr cloud, float &min_x, float &max_x, float &min_y, float &max_y, float &min_z, float &max_z)
{
	PointT min_point_AABB;
	PointT max_point_AABB;

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);

	min_x = min_point_AABB.x;
	max_x = max_point_AABB.x;
	min_y = min_point_AABB.y;
	max_y = max_point_AABB.y;
	min_z = min_point_AABB.z;
	max_z = max_point_AABB.z;
}
