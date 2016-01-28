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

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();

	file.open("/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012.words", std::ifstream::in);
	convnet = ccv_convnet_read(0, "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012-vgg-d.sqlite3");
        
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
	
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	
	if (pcl::io::loadPCDFile<PointT> ("/home/marcog/robocomp/components/prp/experimentFiles/capturas/00032.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}
	cv::Mat rgb = cv::imread("/home/marcog/robocomp/components/prp/experimentFiles/capturas/00032.png");
	
	std::cout << "Number of points before: " << cloud->points.size() << std::endl;

	segmentObjects3D( cloud, rgb);

	std::cout << "Number of points after: " << cloud->points.size() << std::endl;
	
	pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
// 	printf("ACTION: %s\n", action.c_str());
	
	if (action == "imagineMostLikelyMugInPosition")
	{
		action_imagineMostLikelyMugInPosition();
	}

}


bool SpecificWorker::reloadConfigAgent()
{
	return true;
}


bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
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
    std::ofstream ofs("tables_info.data");
    
    boost::archive::text_oarchive oa(ofs);
    
    oa << table1;
    oa << table2;
    oa << table3;
    oa << table4;
    
}

void SpecificWorker::load_tables_info()
{
    std::ifstream ofs("tables_info.data");
    
    boost::archive::text_iarchive oa(ofs);
    oa >> table1;
    oa >> table2;
    oa >> table3;
    oa >> table4;
    
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
                if (boost::filesystem::is_regular_file (it->status()))
		{
                        std::string path2file = it->path().string();
                        std::string location = path2file.substr(0, path2file.find_last_of("/\\"));
                        location = location.substr(location.find_last_of("/\\")+1);
                        
                        cv::Mat rgb = cv::imread( path2file );
                        ColorSeq rgbMatrix;
                        rgbMatrix.resize(640*480);
                        
                        for (int row = 0; row<480; row++)
                        {
                            for (int column = 0; column<640; column ++)
                            {     
                                
                                rgbMatrix[(640*row)+column].blue = rgb.at<cv::Vec3b>(row,column)[0];
                                rgbMatrix[(640*row)+column].green = rgb.at<cv::Vec3b>(row,column)[1];
                                rgbMatrix[(640*row)+column].red = rgb.at<cv::Vec3b>(row,column)[2];
                            }
                        }
                        
                        std::cout<<"Processing image: "<<path2file<<" from table: "<<location<<endl;
                        std::cout<<rgbMatrix.size()<<std::endl;
                        processImage(rgbMatrix, location);
		}
	}                        
}

void SpecificWorker::processImage(const ColorSeq &image, std::string location)
{
    ResultList result;
    std:string label;
    
    getLabelsFromImage(image, result);
    
    for(int i=0; i<result.size(); i++)
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
                    table1[label] = (table1[label] + result[i].believe)/2; 
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
                        table2[label] = (table2[label] + result[i].believe)/2; 
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
                            table3[label] = (table3[label] + result[i].believe)/2; 
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
                                table4[label] = (table4[label] + result[i].believe)/2; 
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

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event &modification)
{
	mutex->lock();
 	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{

}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 
	delete innerModel;
	innerModel = agmInner.extractInnerModel(worldModel);
	mutex->unlock();
}

void SpecificWorker::segmentObjects3D(pcl::PointCloud<PointT>::Ptr cloud, cv::Mat image)
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
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (polygon_cloud);
	ec.extract (cluster_indices);
	std::cout<<"Cluster Indices: "<< cluster_indices.size()<<std::endl;
	
	//cut the image
// 	cv::Mat rgbd_image(480,640, CV_8UC3, cv::Scalar::all(0));
// 	
// 	int j = 0;
// 	pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
// 	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
// 	{
// 		
// 		cloud_cluster->clear();
// 		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
// 		{
// 			cloud_cluster->points.push_back (this->cloud->points[*pit]); //*
// 		}
// 		cloud_cluster->width = cloud_cluster->points.size ();
// 		cloud_cluster->height = 1;
// 		cloud_cluster->is_dense = true;
// 		
// 		//save the cloud at 
// // 		cluster_clouds.push_back(cloud_cluster);
// 		
// 		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
// 		std::stringstream ss;
// 		ss <<"capture_"<<saved_counter<< "_object_" << j;
// 		
// 		/////save rgbd 
// 		
// 		cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
// 		for (unsigned int i = 0; i<cloud_cluster->points.size(); i++)
// 		{
// 			QVec xy = innermodel->project("robot", QVec::vec3(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z), "rgbd"); 
// 
// 			if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
// 			{
// 				M.at<uchar> ((int)xy(1), (int)xy(0)) = 255;
// 			}
// 			else if (not (isinf(xy(1)) or isinf(xy(0))))
// 			{
// 				std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
// 			}
//  		}
//  		
//  		//dilate
//  		cv::Mat dilated_M, z;
//  		cv::dilate( M, dilated_M, cv::Mat(), cv::Point(-1, -1), 2, 1, 1 );
// 		
//  		//find contour
// 		vector<vector<cv::Point> > contours;
// 		vector<cv::Vec4i> hierarchy;
// 		
// 		cv::findContours( dilated_M, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
// 		
// 		  /// Draw contours
// 		cv::Mat mask = cv::Mat::zeros( dilated_M.size(), CV_8UC3 );
// // 		int contour_index = 1;
// 
// // 		cv::Scalar color = cv::Scalar( 0, 255, 0 );
// // 		cv::drawContours( drawing, contours, contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
// 		
// 		cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
// 		
// 		
//     // let's create a new image now
//     cv::Mat crop(rgbd_image.rows, rgbd_image.cols, CV_8UC3);
// 
//     // set background to green
//     crop.setTo(cv::Scalar(255,255,255));
// 		
// 		rgbd_image.copyTo(crop, mask);
// 		
// 		normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
// 		
// 		cout<<"about to display"<<endl;
// 		
// 		cv::namedWindow( "Display window2", cv::WINDOW_AUTOSIZE );// Create a window for display.
//     cv::imshow( "Display window2", rgbd_image );
// 		cv::imwrite( "scene.png", rgbd_image );
// 		
// 		cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//     cv::imshow( "Display window", crop );
// 
// 		cv::imwrite( ss.str() + ".png", crop );
// 
// 		/////save rgbd end
// 		
// 		
//     writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false); //*
//     j++;
//   }
	
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
    
    //if no object try to use semantics
    
    return table;
}

bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
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
		AGMModelPrinter::printWorld(newModel);
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel,"objectoracleAgent");
	}
	catch(...)
	{
		exit(1);
	}
}





void SpecificWorker::action_imagineMostLikelyMugInPosition()
{
	AGMModel::SPtr newModel(new AGMModel(worldModel));

	// Create new symbols and the edges which are independent from the container
	AGMModelSymbol::SPtr mugSt = newModel->newSymbol("objectSt");
	AGMModelSymbol::SPtr mug   = newModel->newSymbol("protoObject");
	newModel->addEdge(mug, mugSt, "mug");
	newModel->addEdge(mug, mugSt, "reachable");
	newModel->addEdge(mug, mugSt, "noReach");
	newModel->addEdge(mug, mugSt, "hasStatus");
	auto symbols = newModel->getSymbolsMap(params, "robot", "status", "table", "room");
	newModel->addEdge(symbols["robot"], mug, "know");
	newModel->addEdge(symbols["robot"], symbols["status"], "usedOracle");
	
	// Create the edges that indicate in which table the object will be located
	AGMModelSymbol::SPtr tableID = newModel->getSymbol(42); // ERROR WARNING TODO  This lines should be changed to the corresponding identifiers 
	AGMModelSymbol::SPtr roomID  = newModel->getSymbol(42); // ERROR WARNING TODO  depending on the table containing the object to be found.
	newModel->addEdge(mug, tableID, "in");
	newModel->addEdge(mug, roomID, "in");

	
	// Send modification proposal
	sendModificationProposal(worldModel, newModel);

	
}



