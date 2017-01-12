#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <flann/flann.h>
#include <flann/algorithms/dist.h>
#include "string"


#define FILES_EXTENSION ".pcd"
std::string feature;
//Function that computes the Viewpoint Feature Histogram
void computeVFHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const boost::filesystem::path &filename)
{
// 	pcl::console::print_highlight ("Computing VFH for %s.\n", filename.string().c_str());
	//---compute normals---
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//create normal estimation class, and pass the input cloud
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	//Create empty kdetree representation, and pass it to the normal estimation object.
	//its content will be filled inside the object based on the given input.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	//set radious of the neighbors to use (1 cm)
	ne.setRadiusSearch(10);
	//computing normals
	ne.compute(*cloud_normals);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr vfhtree (new pcl::search::KdTree<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308> ());
	if(::feature=="vfh"){
		//---proceed to compute VFH---
		

		//Create the VFH estimation class and pas the input to it
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals (cloud_normals);

		//create an empty kdtree representation and pass it to the vfh estimation object
		//its content will be filled inside the object based on the given input.
		vfh.setSearchMethod (vfhtree);

		//compute the features
		vfh.compute (*vfhs);
	}
	else if(::feature=="cvfh")
	{
		// CVFH estimation object.
		pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
		cvfh.setInputCloud (cloud);
		cvfh.setInputNormals(cloud_normals);
		cvfh.setSearchMethod(vfhtree);
		// Set the maximum allowable deviation of the normals,
		// for the region segmentation step.
		cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
		// Set the curvature threshold (maximum disparity between curvatures),
		// for the region segmentation step.
		cvfh.setCurvatureThreshold(1.0);
		// Set to true to normalize the bins of the resulting histogram,
		// using the total number of points. Note: enabling it will make CVFH
		// invariant to scale just like VFH, but the authors encourage the opposite.
		cvfh.setNormalizeBins(false);
		cvfh.compute(*vfhs);
	}
	else if(::feature== "ourcvfh")
	{
		// OUR-CVFH estimation object.
		pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
		ourcvfh.setInputCloud(cloud);
		ourcvfh.setInputNormals(cloud_normals);
		ourcvfh.setSearchMethod(vfhtree);
		ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
		ourcvfh.setCurvatureThreshold(1.0);
		ourcvfh.setNormalizeBins(false);
		// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
		// this will decide if additional Reference Frames need to be created, if ambiguous.
		ourcvfh.setAxisRatio(0.8);
		ourcvfh.compute(*vfhs);
	}
	//save them to file
	pcl::PCDWriter writer;
	std::stringstream ss;
	ss << filename.branch_path().string() << "/" << boost::filesystem::basename(filename) <<"." << ::feature;
// 	pcl::console::print_highlight ("writing %s\n", ss.str().c_str());
	writer.write<pcl::VFHSignature308> (ss.str(), *vfhs, false);
	
}

//Function that recursively reads all files and computes the VFH for them
void readFilesAndComputeCVFH (const boost::filesystem::path &base_dir)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//Recursively read all files and compute VFH
	for(boost::filesystem::directory_iterator it (base_dir); it!=boost::filesystem::directory_iterator (); ++it)
	{
		std::stringstream ss;
		ss << it->path();
		//if its a directory just call back the function
		if (boost::filesystem::is_directory (it->status()))
		{
			pcl::console::print_highlight ("Entering directory %s.\n", ss.str().c_str());
			//call rescursively our function
			readFilesAndComputeCVFH(it->path());
		}
		//if not, go ahead and read and process the file
		if (boost::filesystem::is_regular_file (it->status()) && boost::filesystem::extension (it->path()) == FILES_EXTENSION )
		{
			if(pcl::io::loadPCDFile<pcl::PointXYZ> (it->path().string(), *cloud) == -1)
				PCL_ERROR ("Couldn't read the file %s.", it->path().string().c_str());
			else
			{
				//Finally compute the vfh and save it
				computeVFHistogram(cloud, *it);
			}
		}
	}
}


int main (int argc, char *argv[] )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	
	//Check params
	if ( argc != 3 )
	{
		std::cout<<"You need to specify exactly one parameter"<<std::endl;
		std::cout<<"It should be the path to the training data in pcd format"<<std::endl;
		return -1;
	}

	//----Reading poinclouds----
	const boost::filesystem::path base_dir = argv[1];
	//check if it's a valid path
	if(!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory(base_dir))
	{
		std::cout<<"Not a valid path to data, please make sure you specify a correct one"<<std::endl;
		return -1;
	}
	::feature=argv[2];
	
	if(::feature != "vfh" && ::feature != "cvfh" && ::feature != "ourcvfh")
	{
		std::cout<<"Not a valid feature type, please make sure you specify a correct one"<<std::endl;
		return -1;
	}
	
	//get into the directory and compute VFHs.
	readFilesAndComputeCVFH(argv[1]);

	//done ;)
	return 0;

}