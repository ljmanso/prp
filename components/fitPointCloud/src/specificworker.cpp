/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
	
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }


	
	
// 	timer.start(Period);
	compute();

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	qDebug()<<__LINE__;
	pcl::PointCloud<PointT>::Ptr object, scene;
	qDebug()<<__LINE__;
	object = readThePointCloud("/home/robocomp/robocomp/components/prp/objects/seen.pcd");
	qDebug()<<__LINE__;
	scene = readThePointCloud("/home/robocomp/robocomp/components/prp/objects/saved.pcd");
	qDebug()<<__LINE__;
	for (int i=0; i<10;i++)
		fiting(object,scene);
}

pcl::PointCloud<PointT>::Ptr SpecificWorker::readThePointCloud(const string &pcd)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	qDebug()<<__LINE__;
	
    if (pcl::io::loadPCDFile<PointT> (pcd, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd.pcd \n");
    }
    qDebug()<<__LINE__;
    return cloud;
}

void SpecificWorker::fiting(pcl::PointCloud< PointT >::Ptr object, pcl::PointCloud< PointT >::Ptr scene)
{
	static int i=0;
	pcl::PointCloud<PointT>::Ptr object_aligned (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr object_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
	
	std::cout<<object->size()<<std::endl;
	std::cout<<scene->size()<<std::endl;
	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<PointT> grid;
	const float leaf = 0.003f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);
	
	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP< PointT ,pcl::Normal> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (object);
	nest.compute (*object_normals);
	nest.setInputCloud (scene);
	nest.compute (*scene_normals);
	
	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	pcl::FPFHEstimationOMP<PointT,pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object_normals);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene_normals);
	fest.compute (*scene_features);
	
	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> align;
	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (100000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{	
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/testfiting/scene_"+QString::number(i).toStdString()+".pcd", *scene, false);
 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/testfiting/objectaling_"+QString::number(i).toStdString()+".pcd", *object_aligned, false);
	i++;
}







