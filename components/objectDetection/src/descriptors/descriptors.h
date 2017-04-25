#ifndef _DESCRIPTORS_H
#define _DESCRIPTORS_H

#include <fstream>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <QMutex>

#include <flann/flann.h>
#include <flann/io/hdf5.h>

#define DESCRIPTORS_FILES_EXTENSION ".pcd"

typedef std::pair<std::string, std::vector<float> > descriptors_model;
typedef pcl::PointXYZRGB PointT;

class DESCRIPTORS
{
	std::string kdtree_idx_file_name;
	std::string training_data_h5_file_name;
	std::string training_data_list_file_name;
	std::string type_feature;
	std::string h_extension;
	std::vector<descriptors_model> models;
	flann::Matrix<int> k_indices;
	flann::Matrix<float> k_distances;
	flann::Matrix<float> data;
	QMutex data_mutex;

public:
	struct file_dist_t
	{
		std::string file;
		std::string label;
		float dist;
	};
	void set_type_feature(std::string feature);

	//Loads descriptors histogram
	bool loadHist (const boost::filesystem::path &path, descriptors_model &descriptors);

	//computes an descriptors histogram over a point cloud
	void computeDESCRIPTORShistogram(pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors);

	//read a full directory and computes their descriptors
	bool readFilesAndComputeDESCRIPTORS (const boost::filesystem::path &base_dir, const boost::filesystem::path &original_dir);

	//loads all descriptors models in a directory
	void loadFeatureModels (const boost::filesystem::path &base_dir,const boost::filesystem::path &original_base_dir, const std::string &extension, std::vector<descriptors_model> &models);

	//loads all descriptors models found in a given directory
	void reloadDESCRIPTORS(std::string path_to_dir);

	/** \brief Load the list of file model names from an ASCII file
	* \param models the resultant list of model name
	* \param filename the input file name
	*/
	bool loadFileList(std::vector<descriptors_model> &models, const std::string &filename);

	//laod training data
	void loadTrainingData();

	/** \brief Search for the closest k neighbors
	* \param index the tree
	* \param model the query model
	* \param k the number of neighbors to search for
	* \param indices the resultant neighbor indices
	* \param distances the resultant neighbor distances
	*/
	void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const descriptors_model &model,
				int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

	//guess with trained data
	void doTheGuess(const pcl::PointCloud<PointT>::Ptr object, std::vector<file_dist_t> &guesses);
};

#endif
