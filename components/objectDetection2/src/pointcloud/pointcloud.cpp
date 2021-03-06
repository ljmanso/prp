#include "pointcloud.h"

pcl::PointCloud< PointT >::Ptr computepointcloud::copy_pointcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
	pcl::PointCloud< PointT >::Ptr copy_cloud(new pcl::PointCloud<PointT>(*cloud));
	return copy_cloud;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::PointCloudfrom_Meter_to_mm(pcl::PointCloud< PointT >::Ptr cloud)
{
	pcl::PointCloud< PointT >::Ptr output=copy_pointcloud(cloud);
	for(unsigned int i =0;i<output->points.size();i++)
	{
		output->points[i].x=cloud->points[i].x*1000.;
		output->points[i].y=cloud->points[i].y*1000.;
		output->points[i].z=cloud->points[i].z*1000.;
	}
	return output;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::PointCloudfrom_mm_to_Meters(pcl::PointCloud< PointT >::Ptr cloud)
{
	pcl::PointCloud< PointT >::Ptr output=copy_pointcloud(cloud);
	for(unsigned int i =0;i<cloud->points.size();i++)
	{
		output->points[i].x=cloud->points[i].x/1000.;
		output->points[i].y=cloud->points[i].y/1000.;
		output->points[i].z=cloud->points[i].z/1000.;
	}
	return output;
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

QMat computepointcloud::fitingICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &aligned)
{
// 	double xmean1,ymean1, zmean1, xmean2,ymean2, zmean2;
// 	moveToZero(object,xmean1,ymean1, zmean1);
// 	moveToZero(reference,xmean2,ymean2, zmean2);
// 	cout<<xmean1<<" "<<ymean1<<" "<<zmean1<<endl;
// 	cout<<xmean2<<" "<<ymean2<<" "<<zmean2<<endl;
	pcl::PointCloud< PointT >::Ptr object_copy    = copy_pointcloud(object);
	pcl::PointCloud< PointT >::Ptr reference_copy = copy_pointcloud(reference);

	
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(object_copy);
	icp.setInputTarget(reference_copy);
	icp.setTransformationEpsilon (1e-12);
	icp.setEuclideanFitnessEpsilon (1e-12);
	icp.setMaxCorrespondenceDistance (0.05);
	icp.setMaximumIterations(500);
	icp.align(*aligned);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	Eigen::Matrix4f t=icp.getFinalTransformation();
// 	t(0,3)+=xmean2-xmean1;
// 	t(1,3)+=ymean2-ymean1;
// 	t(2,3)+=zmean2-zmean1;
	QMat TM(4,4);
	for (int c=0; c<4; c++)
		for (int r=0; r<4; r++)
			TM(r,c) = t(r, c);	
	return TM;
}

QMat computepointcloud::fitingSCP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &aligned)
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
	pcl::PointCloud< PointT >::Ptr output=copy_pointcloud(cloud);
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (output);
	sor.setLeafSize (lx, ly, lz);
	sor.filter (*output);
	return output;
}

pcl::PointCloud< PointT >::Ptr computepointcloud::Filter_in_axis(pcl::PointCloud< PointT >::Ptr cloud, string axi, float min, float max, bool negative)
{
	pcl::PointCloud< PointT >::Ptr output=copy_pointcloud(cloud);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (output);
	pass.setFilterFieldName (axi);
	pass.setFilterLimits (min, max);
	pass.setFilterLimitsNegative (negative);
	pass.filter (*output);
	return output;
}

// void computepointcloud::recuperateObjects(pcl::PointCloud< PointT >::Ptr scene, std::vector<pcl::PointCloud<PointT>::Ptr> cluster_clouds)
// {
// 	
// }