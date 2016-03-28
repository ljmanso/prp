/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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

#include <boost/algorithm/string.hpp>

/**
* \brief Default constructor
*/

typedef std::map<std::string, double>::const_iterator MapIterator;

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx),
first(true)
{
	modifiedWorld = -1;
	image_segmented_counter = 0;
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
	worldModelTime = actionTime = QTime::currentTime();

	file.open("/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012.words", std::ifstream::in);
	convnet = ccv_convnet_read(0, "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012-vgg-d.sqlite3");

// 	load_tables_info();
        

//         processDataFromDir("/home/marcog/robocomp/components/prp/experimentFiles/images/");
//         
//         //show map after processing
//         for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
//         {
//                 cout << "Key: " << iter->first << endl << "Value: " << iter->second<< endl;
//         }
//         
//         save_tables_info();
//         load_tables_info();
//         
//         //show map after processing
//         for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
//         {
//                 cout << "Key: " << iter->first << endl << "Value: " << iter->second<< endl;
//         }

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(mutex);
	
	try
	{
		RoboCompAGMWorldModel::World w = agmexecutive_proxy->getModel();
		structuralChange(w);
	}
	catch(...)
	{
		printf("The executive is probably not running, waiting for first AGM model publication...");
	}
	
	
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	printf("ACTION: %s\n", action.c_str());
	
	boost::algorithm::to_lower(action);
	
	if (action == "imaginemostlikelymuginposition")
	{
		action_imagineMostLikelyMugInPosition();
	}

	if (first)
	{
		first=false;
		load_tables_info();
  		//processDataFromDir("/home/marcog/robocomp/components/prp/experimentFiles/capturas/");
		//show map after processing
		
        for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
        {
                cout << "1. Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
        for (MapIterator iter = table2.begin(); iter != table2.end(); iter++)
        {
                cout << "2.Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
        for (MapIterator iter = table3.begin(); iter != table3.end(); iter++)
        {
                cout << "3.Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
        for (MapIterator iter = table4.begin(); iter != table4.end(); iter++)
        {
                cout << "4.Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }       
		for (MapIterator iter = table5.begin(); iter != table5.end(); iter++)
        {
                cout << "5.Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
        //save_tables_info();
	}
}


bool SpecificWorker::reloadConfigAgent()
{
	return true;
}


bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	QMutexLocker locker(mutex);
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	QMutexLocker locker(mutex);
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}



int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	QMutexLocker locker(mutex);
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::save_tables_info()
{
    std::ofstream ofs("/home/robocomp/robocomp/components/prp/experimentFiles/tables_info.data");
    
    boost::archive::text_oarchive oa(ofs);
    
    oa << table1;
    oa << table2;
    oa << table3;
    oa << table4;
	oa << table5;
    
}

void SpecificWorker::load_tables_info()
{
    std::ifstream ofs("/home/robocomp/robocomp/components/prp/experimentFiles/tables_info.data");
    
    boost::archive::text_iarchive oa(ofs);
    oa >> table1;
    oa >> table2;
    oa >> table3;
    oa >> table4;
  	oa >> table5;
}

static unsigned int get_current_time(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

std::fstream& GotoLine(std::fstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(uint i=0; i < num - 1; ++i)
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

ColorSeq SpecificWorker::convertMat2ColorSeq(cv::Mat rgb)
{
	ColorSeq rgbMatrix;
	rgbMatrix.resize(rgb.cols*rgb.rows);
	
	for (int row = 0; row<rgb.rows; row++)
	{
		for (int column = 0; column<rgb.cols; column ++)
		{     
			
			rgbMatrix[(rgb.cols*row)+column].blue = rgb.at<cv::Vec3b>(row,column)[0];
			rgbMatrix[(rgb.cols*row)+column].green = rgb.at<cv::Vec3b>(row,column)[1];
			rgbMatrix[(rgb.cols*row)+column].red = rgb.at<cv::Vec3b>(row,column)[2];
		}
	}
	
	return rgbMatrix;
}

void SpecificWorker::processDataFromDir(const boost::filesystem::path &base_dir)
{
	//Recursively read all files and compute VFH
	for(boost::filesystem::directory_iterator it (base_dir); it!=boost::filesystem::directory_iterator (); ++it)
	{
		std::stringstream ss;
		ss << it->path();
		//if its a directory just call back the function
		if (boost::filesystem::is_directory (it->status()))
		{
			printf ("Entering directory %s.\n", ss.str().c_str());
			//call rescursively our function
			processDataFromDir(it->path());
		}
		//if not, go ahead and read and process the file
        if (boost::filesystem::is_regular_file (it->status()) && boost::filesystem::extension (it->path()) == ".png" ) 
		{
			std::string path2file = it->path().string();
			std::string location = path2file.substr(0, path2file.find_last_of("/\\"));
			location = location.substr(location.find_last_of("/\\")+1);
			
			if(location == "table5")
			{
				//first processing of entire image
				cv::Mat rgb = cv::imread( path2file );
				
				ColorSeq rgbMatrix = convertMat2ColorSeq (rgb);
				
				std::cout<<"Processing image: "<<path2file<<" from table: "<<location<<endl;
				std::cout<<rgbMatrix.size()<<std::endl;
				processImage(rgbMatrix, location);
				
				//3D based segment and process all segmented objects
				std::string pcd_path2file = path2file.substr(0, path2file.find_last_of(".")) + ".pcd";

				pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
				if (pcl::io::loadPCDFile<PointT> (pcd_path2file, *cloud) == -1) //* load the file
				{
					PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
				}
				std::vector<cv::Mat> segmented_objects;
				segmentObjects3D(cloud, rgb, segmented_objects);
				
				cout<<"El vector tiene>>>>> "<<segmented_objects.size()<<endl;
				
	// 			for(std::vector<cv::Mat>::iterator it = sgemented_objects.begin(); it != sgemented_objects.end(); ++it)
				for(uint i = 0 ; i < segmented_objects.size() ; i++)
				{	
					rgbMatrix = convertMat2ColorSeq (segmented_objects[i]);
					processImage(rgbMatrix, location);
				}
			}
			
		}
	}                        
}

void SpecificWorker::processImage(const ColorSeq &image, std::string location)
{
    ResultList result;
    std::string label;
    
    getLabelsFromImage(image, result);
    
    for(uint i=0; i<result.size(); i++)
    {
        std::stringstream names(result[i].name);
        if(location.compare("table1") == 0)
        {
            while(std::getline(names, label, ','))
            {
				
                std::map<std::string,double>::iterator it = table1.find(label);
                if (it == table1.end())
                    table1.insert ( std::pair<std::string, double>(label,result[i].believe) );
                else
				{
// 					std::cout<<" ------------ Max: "<<table1[label]<<" "<<(double)result[i].believe;
                    table1[label] = std::max(table1[label], (double)result[i].believe);
// 					std::cout<<"Result: "<<table1[label]<<std::endl;
				}
            }
        }
        else
        {
            if(location.compare("table2") == 0)
            {
                
                while(std::getline(names, label, ','))
                {
                    std::map<std::string,double>::iterator it = table2.find(label);
                    if (it == table2.end())
                        table2.insert ( std::pair<std::string, double>(label,result[i].believe) );
                    else
					{
// 						std::cout<<" ------------ Max: "<<table2[label]<<" "<<(double)result[i].believe;
                        table2[label] = std::max(table2[label], (double)result[i].believe);
// 						std::cout<<"Result: "<<table2[label]<<std::endl;
					}
                }
            }
            else
            {
                if(location.compare("table3") == 0)
                {
                    while(std::getline(names, label, ','))
                    {
                        std::map<std::string,double>::iterator it = table3.find(label);
                        if (it == table3.end())
                            table3.insert ( std::pair<std::string, double>(label,result[i].believe) );
                        else
						{
// 							std::cout<<" ------------ Max: "<<table3[label]<<" "<<(double)result[i].believe;
                            table3[label] = std::max(table3[label], (double)result[i].believe);
// 							std::cout<<"Result: "<<table3[label]<<std::endl;
						}
                    } 
                }
                else
                {
                    if(location.compare("table4") == 0)
                    {
                        while(std::getline(names, label, ','))
                        {
                            std::map<std::string,double>::iterator it = table4.find(label);
                            if (it == table4.end())
                                table4.insert ( std::pair<std::string, double>(label,result[i].believe) );
                            else
							{
// 								std::cout<<" ------------ Max: "<<table4[label]<<" "<<(double)result[i].believe;
                                table4[label] = std::max(table4[label], (double)result[i].believe);
// 								std::cout<<"Result: "<<table4[label]<<std::endl;
							}
                        }
                    }
                    else
                    {
						if(location.compare("table5") == 0)
						{
							std::map<std::string,double>::iterator it = table5.find(label);
                            if (it == table5.end())
                                table5.insert ( std::pair<std::string, double>(label,result[i].believe) );
                            else
							{
// 								std::cout<<" ------------ Max: "<<table5[label]<<" "<<(double)result[i].believe;
                                table5[label] = std::max(table5[label], (double)result[i].believe);
// 								std::cout<<"Result: "<<table5[label]<<std::endl;
							}
						}
						else
						{
							std::cout<<"Processing Image: Location not properly specified"<<std::endl;
							return;
						}
                    }
                }
            }
        }
    }
    
}

void SpecificWorker::getLabelsFromImage(const ColorSeq &image, ResultList &result)
{

#ifdef DEBUG
    cout<<"SpecificWorker::getLabelsFromImage"<<endl;
#endif
    
    ccv_dense_matrix_t* ccv_image = 0;
    
    convnet = ccv_convnet_read(0, "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012-vgg-d.sqlite3");
    
    ccv_read(image.data(), &ccv_image, CCV_IO_RGB_RAW, 480, 640, 1920);
    assert(ccv_image != 0);
//     ccv_matrix_t *a = 0;
    
    string label;
         
    ccv_dense_matrix_t* input = 0;
    ccv_convnet_input_formation(convnet->input, ccv_image, &input);
    ccv_matrix_free(ccv_image);
    unsigned int elapsed_time = get_current_time();
    ccv_array_t* rank = 0;
    ccv_convnet_classify(convnet, &input, 1, &rank, 5, 1);
    elapsed_time = get_current_time() - elapsed_time;
    int i;
    for (i = 0; i < rank->rnum - 1; i++)
    {
            //Obtain result
            ccv_classification_t* classification = (ccv_classification_t*)ccv_array_get(rank, i);
            GotoLine(file, classification->id + 1);
            std::getline(file,label);
#ifdef DEBUG
            printf("%d %f ", classification->id + 1, classification->confidence);
            cout<<label<<endl;
#endif
            //Save result to return
            Label l;
            l.name = label;
            l.believe = classification->confidence;
            result.push_back(l);
            
            //reset labels file pointer
            file.clear() ;
            file.seekg(0, ios::beg) ;
            
    }
    ccv_classification_t* classification = (ccv_classification_t*)ccv_array_get(rank, rank->rnum - 1);
    GotoLine(file, classification->id + 1);
    std::getline(file,label);            
#ifdef DEBUG
    printf("%d %f ", classification->id + 1, classification->confidence);
    cout<<label<<endl;
    printf("elapsed time %dms\n", elapsed_time);
#endif
    
    //Save result to return
    Label l;
    l.name = label;
    l.believe = classification->confidence;
    result.push_back(l);
    
    ccv_array_free(rank);
    ccv_matrix_free(input);
    ccv_convnet_free(convnet);
    
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &modification)
{
	QMutexLocker locker(mutex);
 	AGMModelConverter::fromIceToInternal(modification, worldModel);
	worldModelTime = QTime::currentTime();
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker locker(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(mutex);
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(mutex);
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker locker(mutex);
	for (auto modification : modifications)
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
}

void SpecificWorker::segmentObjects3D(pcl::PointCloud<PointT>::Ptr cloud, cv::Mat image, std::vector<cv::Mat> &result)
{
	
	//downsample
	pcl::VoxelGrid<PointT> voxel_grid;
	voxel_grid.setInputCloud (cloud);
	voxel_grid.setLeafSize (10, 10, 10);
	voxel_grid.filter(*cloud);
	
	//Use Ransac to find a plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (15);
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	std::cout<<"Ransac inliers size: "<<inliers->indices.size()<<std::endl;
	//save inlisers to coud
// 	pcl::copyPointCloud<PointT>(*cloud, *inliers, *cloud);
	
	//project points to plane
	pcl::PointCloud<PointT>::Ptr plane_projected (new pcl::PointCloud<PointT>);
	pcl::ProjectInliers<PointT> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setIndices (inliers);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*plane_projected);
	std::cout<<"Plane plane_projected size: "<<plane_projected->points.size()<<std::endl;
	
// 	*cloud = *plane_projected;
	
	//cloud_hull
	pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
 	pcl::ConvexHull<PointT> chull;
 	chull.setInputCloud(plane_projected);
 	chull.reconstruct(*cloud_hull);
	std::cout<<"Cloud hull size: "<<cloud_hull->points.size()<<std::endl;
	
// 	*cloud = *cloud_hull;
	
	pcl::ExtractPolygonalPrismData<PointT> prism_extract;
 	pcl::PointCloud<PointT>::Ptr polygon_cloud(new pcl::PointCloud<PointT>);
	pcl::PointIndices::Ptr prism_indices (new pcl::PointIndices);
	
 	prism_extract.setHeightLimits(25, 1500);
	prism_extract.setViewPoint(0, 0, 1);
 	prism_extract.setInputCloud(cloud);
 	prism_extract.setInputPlanarHull(cloud_hull);
 	prism_extract.segment(*prism_indices);
 	
	//let's extract the result
 	pcl::ExtractIndices<PointT> extract_prism_indices;
 	extract_prism_indices.setInputCloud(cloud);
 	extract_prism_indices.setIndices(prism_indices);
 	extract_prism_indices.filter(*(polygon_cloud));
	std::cout<<"Polygon cloud size: "<<polygon_cloud->points.size()<<std::endl;
	
	*cloud = *polygon_cloud;
	
	//euclidean extraction
	
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::EuclideanClusterExtraction<PointT> ec;
	std::vector<pcl::PointIndices> cluster_indices;
		
	tree->setInputCloud (polygon_cloud);
	cluster_indices.clear();
	
	ec.setClusterTolerance (70); // 2cm
	ec.setMinClusterSize (40);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (polygon_cloud);
	ec.extract (cluster_indices);
	std::cout<<"Cluster Indices: "<< cluster_indices.size()<<std::endl;
	
	
	//cut the image
// 	cv::Mat rgbd_image(480,640, CV_8UC3, cv::Scalar::all(0));
 	image_segmented_counter++;
	int j = 0;
	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
// 		
		cloud_cluster->clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
// 		
// 		//save the cloud at 
// // 		cluster_clouds.push_back(cloud_cluster);
// 		
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss <<"capture_"<<image_segmented_counter<< "_object_" << j;

		cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
		
// 		float maxAngleWidth = (float) (57.0f * (M_PI / 180.0f));
// 		float maxAngleHeight = (float) (43.0f * (M_PI / 180.0f));
// 		float angularResolution = (float)(57.0f / 640.0f * (M_PI/180.0f));
// 		Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
// 		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
// 		float noiseLevel=0.00;
// 		float minRange = 0.0f;
// 		int borderSize = 1;
		std::vector<cv::Point> points;
		
// 		for (unsigned int i = 0; i<cloud_cluster->points.size(); i++)
		for(pcl::PointCloud<PointT>::const_iterator cloud_it = cloud_cluster->begin(); cloud_it != cloud_cluster->end(); cloud_it++ )
		{
			int f = 525;
			int x = f*-cloud_it->x/cloud_it->z + 320;
			int y = f*cloud_it->y/cloud_it->z + 240;
			points.push_back(cv::Point(x,y));

// 			M.at<uchar> (y, x) = 255;

 		}
 		
 		cv::RotatedRect rect = cv::minAreaRect(cv::Mat(points));
// 		cv::Point2f vertices[4];
// 		box.points(vertices);
		cv::Mat M_, rotated, cropped;
		// get angle and size from the bounding box
        float angle = rect.angle;
        cv::Size rect_size = rect.size;
		rect_size.width += 30;
		rect_size.height += 30;
        // thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
        if (rect.angle < -45.) {
            angle += 90.0;
            swap(rect_size.width, rect_size.height);
        }
        // get the rotation matrix
        M_ = getRotationMatrix2D(rect.center, angle, 1.0);
        // perform the affine transformation
        warpAffine(image, rotated, M_, image.size(), cv::INTER_CUBIC);
        // crop the resulting image
        getRectSubPix(rotated, rect_size, rect.center, cropped);
		
// 		cv::imshow("Result", cropped);
// 		cv::waitKey(0);
		
 		cv::imwrite( ss.str() + ".png", cropped );
// 		
// 		pcl::io::savePCDFileASCII (ss.str () + ".pcd", *cloud_cluster);
 		j++;
		result.push_back(cropped);
	}
	
// 	return result;
}

std::string SpecificWorker::lookForObject(std::string label)
{
    std::string table = "none";
    double current_believe = -1;
    
    //Obtaining object posibility from the different tables
    std::map<std::string,double>::iterator it = table1.find(label);
    if (it != table1.end() && it->second > current_believe)
    {
        table =  "table1";
        current_believe = it->second;
    }
    
    if (it != table2.end() && it->second > current_believe)
    {
        table =  "table2";
        current_believe = it->second;
    }
    
    if (it != table3.end() && it->second > current_believe)
    {
        table =  "table3";    
        current_believe = it->second;
    }
    
    if (it != table4.end() && it->second > current_believe)
    {
        table =  "table4";       
        current_believe = it->second;
    }
	if (it != table5.end() && it->second > current_believe)
    {
        table =  "table5";       
        current_believe = it->second;
    }
    //if no object try to use semantics
    
    return table;
}

bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	QMutexLocker locker(mutex);
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		actionTime = QTime::currentTime();
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}

void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmexecutive_proxy, "objectoracleAgent");
	}
	catch(...)
	{
		exit(1);
	}
}


void SpecificWorker::imagineMostLikelyOBJECTPosition(string objectType)
{
	printf("inside imagineMostLikelyOBJECTPosition-%s  %d %d\n", objectType.c_str(), modifiedWorld, worldModel->version);
	if (modifiedWorld > worldModel->version or actionTime < worldModelTime)
		return;

	AGMModel::SPtr newModel(new AGMModel(worldModel));

	// Create new symbols and the edges which are independent from the container
	AGMModelSymbol::SPtr objSSt = newModel->newSymbol("objectSt");
	AGMModelSymbol::SPtr objS   = newModel->newSymbol("protoObject");
	newModel->addEdge(objS, objSSt, objectType);
	newModel->addEdge(objS, objSSt, "reachable");
	newModel->addEdge(objS, objSSt, "noReach");
	newModel->addEdge(objS, objSSt, "hasStatus");
	auto symbols = newModel->getSymbolsMap(params, "robot", "status", "table", "room");
	newModel->addEdge(symbols["robot"], objS, "know");
	newModel->addEdge(symbols["robot"], symbols["status"], "usedOracle");
	
	//Locate objS
	std::string table = lookForObject(objectType);
	int id = -1;
	int room_id = -1;
	if( table == "table1" )
	{
		id = 28;
		room_id = 5;
	}
	if( table == "table2" )
	{
		id = 26;
		room_id = 5;
	}
	if( table == "table3" )
	{
		id = 24;
		room_id = 3;
	}
	if( table == "table4" )
	{
		id = 22;
		room_id = 3;
	}
	if( table == "table5" )
	{
		id = 20;
		room_id = 3;
	}

	
	// Create the edges that indicate in which table the object will be located
	AGMModelSymbol::SPtr tableID = newModel->getSymbol(id);
	AGMModelSymbol::SPtr roomID  = newModel->getSymbol(room_id);
	newModel->addEdge(objS, tableID, "in");
	newModel->addEdge(objS, roomID, "in");

	
	// Send modification proposal
	modifiedWorld = worldModel->version + 1;
	sendModificationProposal(worldModel, newModel);
}



void SpecificWorker::action_imagineMostLikelyMugInPosition()
{
	QMutexLocker locker(mutex);
	imagineMostLikelyOBJECTPosition("mug");
}
void SpecificWorker::action_imagineMostLikelyCoffeePotInPosition()
{
	QMutexLocker locker(mutex);
	imagineMostLikelyOBJECTPosition("coffeepot");
}
void SpecificWorker::action_imagineMostLikelyMilkInPosition()
{
	QMutexLocker locker(mutex);
	imagineMostLikelyOBJECTPosition("milk");
}



void SpecificWorker::semanticDistance(const string &word1, const string &word2, float &result)
{

}


