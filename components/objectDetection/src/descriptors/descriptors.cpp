#include "descriptors.h"

void DESCRIPTORS::set_type_feature(std::__cxx11::string feature)
{
	type_feature=feature;
	if(type_feature=="VFH")
		h_extension="vfh";
	else if(type_feature=="CVFH")
		h_extension="cvfh";
	else if(type_feature== "OUR-CVFH")
		h_extension="ourcvfh";
}

bool DESCRIPTORS::loadHist (const boost::filesystem::path &path, descriptors_model &descriptors)
{
  int descriptors_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type;
		unsigned int idx;
		std::cout<<path.string()<<std::endl;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    descriptors_idx = pcl::getFieldIndex (cloud, "vfh");
		std::cout<<((int)cloud.width * cloud.height != 1)<<std::endl;
    if (descriptors_idx == -1)
      return (false);
    if (h_extension != "ourcvfh" && ((int)cloud.width * cloud.height != 1))
      return (false);
		if (h_extension == "ourcvfh" && ((int)cloud.width * cloud.height == 0 ))
			return (false);
  }
  catch (pcl::InvalidConversionException e)
  {
    return (false);
  }
  // Treat the DESCRIPTORS signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  descriptors.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  pcl::getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[descriptors_idx].count; ++i)
  {
    descriptors.second[i] = point.points[0].histogram[i];
  }
  descriptors.first = path.string ();
  return (true);
}

//Function that computes the Viewpoint Feature Histogram
void DESCRIPTORS::computeDESCRIPTORShistogram(pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors)
{
	//---compute normals---
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);



	pcl::search::KdTree<PointT>::Ptr descriptorstree (new pcl::search::KdTree<PointT> ());

	if(type_feature=="VFH")
	{
		//---compute normals---
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		//create normal estimation class, and pass the input cloud
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		//Create empty kdetree representation, and pass it to the normal estimation object.
		//its content will be filled inside the object based on the given input.
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
		ne.setSearchMethod (tree);
		//set radious of the neighbors to use (1 cm)
		ne.setRadiusSearch(10);
		//computing normals
		ne.compute(*cloud_normals);


		//---proceed to compute VFH---
		//Create the VFH estimation class and pas the input to it
		pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
		vfh.setInputCloud (cloud);
		vfh.setInputNormals (cloud_normals);

		//create an empty kdtree representation and pass it to the vfh estimation object
		//its content will be filled inside the object based on the given input.
		vfh.setSearchMethod (descriptorstree);

		//compute the features
		vfh.compute (*descriptors);
	}
	else if(type_feature=="CVFH")
	{
		//create normal estimation class, and pass the input cloud
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		//Create empty kdetree representation, and pass it to the normal estimation object.
		//its content will be filled inside the object based on the given input.
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
		ne.setSearchMethod (tree);
		//set radious of the neighbors to use (1 cm)
		ne.setRadiusSearch(10);
		//computing normals
		ne.compute(*cloud_normals);

		// CVFH estimation object.
		pcl::CVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> cvfh;
		cvfh.setInputCloud(cloud);
		cvfh.setInputNormals(cloud_normals);
		cvfh.setSearchMethod(descriptorstree);
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
		cvfh.compute(*descriptors);
	}
	else if(type_feature== "OUR-CVFH")
	{
		pcl::PointCloud<PointT>::Ptr cloud_c=PointCloudfrom_mm_to_Meters(copy_pointcloud(cloud));
		//create normal estimation class, and pass the input cloud
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (cloud_c);

		//Create empty kdetree representation, and pass it to the normal estimation object.
		//its content will be filled inside the object based on the given input.
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
		ne.setSearchMethod (tree);
		//set radious of the neighbors to use (1 cm)
		ne.setRadiusSearch(0.01);
		//computing normals
		ne.compute(*cloud_normals);

		// OUR-CVFH estimation object.
		pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourcvfh;
		ourcvfh.setInputCloud(cloud_c);
		ourcvfh.setInputNormals(cloud_normals);
		ourcvfh.setSearchMethod(descriptorstree);
		ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
		ourcvfh.setCurvatureThreshold(1.0);
		ourcvfh.setNormalizeBins(false);
		// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
		// this will decide if additional Reference Frames need to be created, if ambiguous.
		ourcvfh.setAxisRatio(1);
		ourcvfh.compute(*descriptors);
	}
}

//Function that recursively reads all files and computes the VFH for them
bool DESCRIPTORS::readFilesAndComputeDESCRIPTORS (const boost::filesystem::path &base_dir, const boost::filesystem::path &original_dir)
{
	if(!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory(base_dir))
	{
		std::cout<<"Not a valid path to data, please make sure you specify a correct one"<<std::endl;
		return false;
	}
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308> ());

	//Recursively read all files and compute VFH
	for(boost::filesystem::directory_iterator it (base_dir); it!=boost::filesystem::directory_iterator (); ++it)
	{
		std::stringstream ss,outpath;
		ss << it->path();
		//if its a directory just call back the function
		if (boost::filesystem::is_directory (it->status()))
		{
			pcl::console::print_highlight ("Entering directory %s.\n", ss.str().c_str());
			//call rescursively our function
			readFilesAndComputeDESCRIPTORS(it->path(), original_dir);
		}
		//if not, go ahead and read and process the file
		if (boost::filesystem::is_regular_file (it->status()) && boost::filesystem::extension (it->path()) == DESCRIPTORS_FILES_EXTENSION)
		{
			boost::filesystem::path filename=*it;
			outpath << filename.branch_path().string() << "/" << boost::filesystem::basename(filename) <<"." << h_extension;
			if (boost::filesystem::exists(outpath.str()) or original_dir==base_dir)
				continue;
			printf("%s doesn't exist: generating files...\n", outpath.str().c_str());
			if(pcl::io::loadPCDFile<PointT> (it->path().string(), *cloud) == -1)
				PCL_ERROR ("Couldn't read the file %s.", it->path().string().c_str());
			else
			{
				//Finally compute the vfh and save it
				computeDESCRIPTORShistogram(cloud, descriptors);
				pcl::PCDWriter writer;
				// 	pcl::console::print_highlight ("writing %s\n", ss.str().c_str());
				writer.write<pcl::VFHSignature308> (outpath.str(), *descriptors, false);
			}
		}
	}
	return true;
}

void DESCRIPTORS::loadFeatureModels (const boost::filesystem::path &base_dir, const boost::filesystem::path &original_base_dir, const std::string &extension,
                   std::vector<descriptors_model> &models)
{
	if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
		return;

	for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
	{
		if (boost::filesystem::is_directory (it->status ()))
		{
			std::stringstream ss;
			ss << it->path ();
			pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
			loadFeatureModels (it->path(), original_base_dir, extension, models);
		}
		if (original_base_dir!=base_dir and  boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
		{
			std::cout<<"in"<<std::endl;
			descriptors_model m;
			if (loadHist (base_dir / it->path ().filename (), m))
				models.push_back (m);
		}
	}
}

void DESCRIPTORS::reloadDESCRIPTORS(std::string path_to_dir)
{

	kdtree_idx_file_name = "kdtree.idx";
	training_data_h5_file_name = "training_data.h5";
	training_data_list_file_name = "training_data.list";

	std::vector<descriptors_model> models;

	std::string model_directory (path_to_dir);

	// Load the model histograms
	loadFeatureModels (model_directory, model_directory, "."+h_extension, models);

  pcl::console::print_highlight ("Loaded %d DESCRIPTORS models. Creating training data %s/%s.\n",
      (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

	flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data[i][j] = models[i].second[j];

	std::cout<<"Saving data to disk"<<std::endl;
  // Save data to disk (list of models)
  flann::save_to_file (data, training_data_h5_file_name, "training_data");
  std::ofstream fs;
  fs.open (training_data_list_file_name.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
  {
		std::cout<<models[i].first<<std::endl;
		fs << models[i].first << "\n";
  }
  fs.close ();

  // Build the tree index and save it to disk
  pcl::console::print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
  //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
  index.buildIndex ();
  index.save (kdtree_idx_file_name);
  delete[] data.ptr ();

}


bool DESCRIPTORS::loadFileList (std::vector<descriptors_model> &models, const std::string &filename)
{
	std::ifstream fs;
	fs.open (filename.c_str ());
	if (!fs.is_open () || fs.fail ())
		return (false);

	std::string line;
	while (!fs.eof ())
	{
		getline (fs, line);
		if (line.empty ())
			continue;
		descriptors_model m;
		m.first = line;
		models.push_back (m);
	}
	fs.close ();
	return (true);
}

void DESCRIPTORS::loadTrainingData()
{
	if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
	{
		pcl::console::print_error ("Could not find training data models files %s and %s!\n",
				training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
		return;
	}
	else
	{
		loadFileList (models, training_data_list_file_name);
		flann::load_from_file (data, training_data_h5_file_name, "training_data");
		pcl::console::print_highlight ("Training data found. Loaded %d DESCRIPTORS models from %s/%s.\n",
				(int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
	}

	if (!boost::filesystem::exists (kdtree_idx_file_name))
	{
		pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
		return;
	}
	else
	{
		flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name));
		index.buildIndex ();
	}
}

void DESCRIPTORS::nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const descriptors_model &model,
				int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
	// Query point
	flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
	memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

	indices = flann::Matrix<int>(new int[k], 1, k);
	distances = flann::Matrix<float>(new float[k], 1, k);
	index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
	delete[] p.ptr ();
}
void DESCRIPTORS::doTheGuess(const pcl::PointCloud<PointT>::Ptr object, std::vector<file_dist_t> &guesses)
{
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308> ());

	computeDESCRIPTORShistogram(object, descriptors);

	descriptors_model histogram;
	// pcl::PointCloud <pcl::VFHSignature308> point;
	histogram.second.resize (308);

	std::vector <pcl::PCLPointField> fields;
	int descriptors_idx = pcl::getFieldIndex (*descriptors, "vfh", fields);
	for (size_t i = 0; i < fields[descriptors_idx].count; ++i)
	{
		histogram.second[i] = descriptors->points[0].histogram[i];
// 		std::cout<<histogram.second[i]<<std::endl;
	}
	histogram.first = "cloud_from_object."+ h_extension;

	//let look for the match
	flann::Matrix<float> data_aux = *(new flann::Matrix<float>(data));

	flann::Index<flann::ChiSquareDistance<float> > index (data_aux, flann::SavedIndexParams ("kdtree.idx"));

	index.buildIndex ();

	nearestKSearch (index, histogram, models.size(), k_indices, k_distances);

	guesses.clear();

// 	pcl::console::print_highlight ("The closest 16 neighbors are:\n");
	for (unsigned int i = 0; i < models.size(); ++i)
	{
		file_dist_t dato;
		dato.file=models.at(k_indices[0][i]).first;
		dato.label =dato.file.substr(0, dato.file.find_last_of("/"));
		dato.label = dato.label.substr(dato.label.find_last_of("/")+1);
		dato.dist = k_distances[0][i];
		guesses.push_back(dato);
	}
	std::sort( guesses.begin(), guesses.end(), [](DESCRIPTORS::file_dist_t a, DESCRIPTORS::file_dist_t b){ return (a.dist < b.dist);}) ;
}
