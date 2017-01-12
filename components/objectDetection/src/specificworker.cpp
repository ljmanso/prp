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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
,cloud(new pcl::PointCloud<PointT>)
,ransac_inliers (new pcl::PointIndices)
,projected_plane(new pcl::PointCloud<PointT>)
,cloud_hull(new pcl::PointCloud<PointT>)
,prism_indices (new pcl::PointIndices)
,rgb_image(480,640, CV_8UC3, cv::Scalar::all(0))
,color_segmented(480,640, CV_8UC3, cv::Scalar::all(0))
,table(new Table())
,vfh_matcher(new VFH())

{
	//let's set the sizes
	table->set_board_size(500,30,500);
        
	
        
	marca_tx = marca_ty = marca_tz = marca_rx = marca_ry = marca_rz = 0;
	
	num_object_found = 0;
	num_scene = 15;
	graphic->setScene(&scene);
	graphic->show();
	item_pixmap=new QGraphicsPixmapItem();
	scene.addItem(item_pixmap);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::readThePointCloud(const string &image, const string &pcd)
{
    rgb_image = cv::imread(image);
    
    if(! rgb_image.data )                              // Check for invalid inpute
    {
        cout <<  "Could not open or find the image rgb.png" << std::endl ;
    }
    
    if (pcl::io::loadPCDFile<PointT> (pcd, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd.pcd \n");
    }
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	string name = PROGRAM_NAME;
	innermodel = new InnerModel(params[name+".innermodel"].value);
    id_robot=QString::fromStdString(params[name+".id_robot"].value);
	id_camera=QString::fromStdString(params[name+".id_camera"].value);
	QString id_camera_transform=QString::fromStdString(params[name+".id_camera_transform"].value);
	viewpoint_transform = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
	vfh_matcher->set_type_feature(params[name+".type_features"].value);
	if(params[name+".type_features"].value=="VFH")
		descriptors_extension="vfh";
	else if(params[name+".type_features"].value=="CVFH")
		descriptors_extension="cvfh";
	else if(params[name+".type_features"].value=="OUR-CVFH")
		descriptors_extension="ourcvfh";
	std::cout<<params[name+".type_features"].value<<" " <<descriptors_extension<<std::endl;
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	RoboCompRGBD::ColorSeq rgbMatrix;	
// 	RoboCompRGBD::depthType distanceMatrix;
// 	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	cv::Mat rgb_image(480,640, CV_8UC3, cv::Scalar::all(0));
	try
	{
		rgbd_proxy->getRGB(rgbMatrix,h,b);
// 		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
	{
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
		return;
	}
	for(unsigned int i=0; i<rgbMatrix.size(); i++)
	{
		int row = (i/640), column = i-(row*640);
		rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	
	cv::Mat dest;
    cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
    QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	item_pixmap->setPixmap(QPixmap::fromImage(image));
	scene.update();
}


void SpecificWorker::grabTheAR()
{

}

void SpecificWorker::aprilFitModel(const string &model)
{

}

void SpecificWorker::segmentImage()
{
#if DEBUG
	cv::imwrite("nosegmentada.png",rgb_image);
	std::cout<<"setting image"<<std::endl;
#endif
	
	segmentator.set_image(&rgb_image);
	
#if DEBUG
	std::cout<<"Segmenting image"<<std::endl;
#endif
	
	
	segmentator.set_tresholds(100, 150);
	color_segmented = segmentator.segment();
	
#if DEBUG
	std::cout<<"Segmented"<<std::endl;
	cv::imwrite("Segmentada.png",color_segmented);
	cv::Mat yellow, pink, green;
	cv::inRange(color_segmented, cv::Scalar(0, 150, 150), cv::Scalar(80, 255, 255), yellow);
	cv::imwrite("yellow.png",yellow);
	cv::inRange(color_segmented, cv::Scalar(65, 15, 125), cv::Scalar(150, 100, 255), pink);
	cv::imwrite("pink.png",pink);
	cv::inRange(color_segmented, cv::Scalar(25, 75, 50), cv::Scalar(106, 255, 150), green);
	cv::imwrite("green.png",green);
#endif
}

void SpecificWorker::mindTheGapPC()
{

}

void SpecificWorker::getPose(float &x, float &y, float &z, float &rx, float &ry, float &rz)
{
    
    //publish imageen
//     cv_bridge::CvImage out_msg;
//     out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3; // Or whatever
//     out_msg.image = rgb_image; // Your cv::Matencoding
//     image_pub.publish(out_msg.toImageMsg());
//     
//     //publish point cloud
//     sensor_msgs::PointCloud2 cloud2;
//     pcl::toROSMsg(*cloud, cloud2);
//     pcd_pub.publish(cloud2);

    x = 1.5;
    y = 2.5;
    z = 3.5;
	rx = 1.5;
    ry = 2.5;
    rz = 3.5;
    
    
    
}

void SpecificWorker::centroidBasedPose(float &x, float &y, float &theta)
{

}

void SpecificWorker::reloadVFH(const string &pathToSet)
{
	string s="./bin/createDescriptors "+pathToSet +" "+ descriptors_extension;
	char *cstr = &s[0u];
	if (system(cstr)==0)
	{
		vfh_matcher->reloadVFH(pathToSet);
		vfh_matcher->loadTrainingData();
	}
}

void SpecificWorker::ransac(const string &model)
{
	table->fit_board_with_RANSAC( cloud, ransac_inliers, 15);
	cout<<"RANSAC INLIERS: "<<ransac_inliers->indices.size()<<endl;
}

void SpecificWorker::euclideanClustering(int &numCluseters)
{
/*	
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
	
	//downsample
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (this->cloud);
	sor.setLeafSize (10, 10, 10);
	sor.filter (*cloud_filtered);
*/
  
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (this->cloud);
	cluster_indices.clear();
	cluster_clouds.clear();
	pcl::EuclideanClusterExtraction<PointT> ec;
	
	ec.setClusterTolerance (40); 
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (50000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (this->cloud);
	ec.extract (cluster_indices);
	 
/*	
	cv::Mat rgb_image(480,640, CV_8UC3, cv::Scalar::all(0));
	
	lets transform the image to opencv
	cout<<rgbMatrix.size()<<endl;
	for(int i=0; i<rgbMatrix.size(); i++)
	{
		std::cout<<"the first one: " <<i<<std::endl;
		int row = i/640;
		int column = i-(row*640);
		
		rgb_image.at<cv::Vec3b>(row,column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	*/ 
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			cloud_cluster->points.push_back (this->cloud->points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
			
		//save the cloud at 
		cluster_clouds.push_back(cloud_cluster);
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		#if SAVE_DATA	
			std::stringstream ss;
			ss <<"/home/robocomp/robocomp/components/prp/scene/"<<num_scene<<"_capture_object_" << j;
		
                /////save /*rgbd*/ 

			cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
			for (int i = 0; i<cloud_cluster->points.size(); i++)
			{
				InnerModelCamera *camera = innermodel->getCamera(id_camera);
				
				QVec xy = camera->project(id_robot, QVec::vec3(cloud_cluster->points[i].x, cloud_cluster->points[i].y, cloud_cluster->points[i].z)); 
				
				if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
				{
					M.at<uchar> ((int)xy(1), (int)xy(0)) = 255;
				}
				else if (not (isinf(xy(1)) or isinf(xy(0))))
				{
					std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
				}
			}
				
			//dilate
			cv::Mat dilated_M, z;
			cv::dilate( M, dilated_M, cv::Mat(), cv::Point(-1, -1), 2, 1, 1 );
			
			//find contour
			vector<vector<cv::Point> > contours;
			vector<cv::Vec4i> hierarchy;
			
			cv::findContours( dilated_M, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			
			/// Draw contours
			cv::Mat mask = cv::Mat::zeros( dilated_M.size(), CV_8UC3 );
	// 		int contour_index = 1;

	// 		cv::Scalar color = cv::Scalar( 0, 255, 0 );
	// 		cv::drawContours( drawing, contours, contour_index, color, 2, 8, hierarchy, 0, cv::Point() );
			
			cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
				
				
			// let's create a new image now
			cv::Mat crop(rgb_image.rows, rgb_image.cols, CV_8UC3);

			// set background to green
			crop.setTo(cv::Scalar(255,255,255));
				
			rgb_image.copyTo(crop, mask);
				
			normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);
				
	// 		cv::namedWindow( "Display window2", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// 		cv::imshow( "Display window2", rgb_image );
			std::string scenename = "/home/robocomp/robocomp/components/prp/scene/" + std::to_string(num_scene) + "_scene.png";
			cv::imwrite( scenename, rgb_image );
				
	// 		cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
	// 		cv::imshow( "Display window", crop );

			cv::imwrite( ss.str() + ".png", crop );

				/////save rgbd end
				
		
		
		#endif
/*
//		InnerModelNode *parent = innermodel->getNode(QString::fromStdString("rgbd"));
// 		innermodel->newTransform(QString::fromStdString("marca"), QString::fromStdString("static") ,parent, 356.152, -289.623, 756.853, 0.32, 0, 0);
// 
// 		QMat PP = innermodel->getTransformationMatrix("marca", "robot");
// 		
// 		for (unsigned int i=0; i<cloud_cluster->points.size(); i++)
// 		{
// 			QVec p1 = QVec::vec4(cloud_cluster->points[i].x,cloud_cluster->points[i].y, cloud_cluster->points[i].z, 1);
// 			QVec p2 = PP * p1;
// 			QVec p22 = p2.fromHomogeneousCoordinates();
// 
// 			cloud_cluster->points[i].x=p22(0);
// 			cloud_cluster->points[i].y=p22(1);
// 			cloud_cluster->points[i].z=p22(2);
// 			cloud_cluster->points[i].r=cloud_cluster->points[i].r;
// 			cloud_cluster->points[i].g=cloud_cluster->points[i].g;
// 			cloud_cluster->points[i].b=cloud_cluster->points[i].b;
// 		}	
// */
		#if SAVE_DATA	
			writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false); 
		#endif
		j++;
	}
	num_scene++;

}

bool SpecificWorker::aprilSeen(pose6D &offset, const pose6D &tag1, const pose6D &tag2, const pose6D &tag3, const pose6D &tag4, const pose6D &tag5, const pose6D &tag6, const pose6D &tag7, const pose6D &tag8, const pose6D &tag9)
{
	QMutexLocker locker(&april_mutex);
	
	for (auto ap : tags)
	{
		auto ap2 = ap;
		if(ap2.id==1)
		{
			offset.tx = ap2.tx + tag1.tx;
			offset.ty = ap2.ty + tag1.ty;
			offset.tz = ap2.tz + tag1.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==2)
		{
			offset.tx = ap2.tx + tag2.tx;
			offset.ty = ap2.ty + tag2.ty;
			offset.tz = ap2.tz + tag2.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==3)
		{
			offset.tx = ap2.tx + tag3.tx;
			offset.ty = ap2.ty + tag3.ty;
			offset.tz = ap2.tz + tag3.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==4)
		{
			offset.tx = ap2.tx + tag4.tx;
			offset.ty = ap2.ty + tag4.ty;
			offset.tz = ap2.tz + tag4.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==5)
		{
			offset.tx = ap2.tx + tag5.tx;
			offset.ty = ap2.ty + tag5.ty;
			offset.tz = ap2.tz + tag5.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==6)
		{
			offset.tx = ap2.tx + tag6.tx;
			offset.ty = ap2.ty + tag6.ty;
			offset.tz = ap2.tz + tag6.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==7)
		{
			offset.tx = ap2.tx + tag7.tx;
			offset.ty = ap2.ty + tag7.ty;
			offset.tz = ap2.tz + tag7.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==8)
		{
			offset.tx = ap2.tx + tag8.tx;
			offset.ty = ap2.ty + tag8.ty;
			offset.tz = ap2.tz + tag8.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		if(ap2.id==9)
		{
			offset.tx = ap2.tx + tag9.tx;
			offset.ty = ap2.ty + tag9.ty;
			offset.tz = ap2.tz + tag9.tz;
			offset.rx = ap2.rx; 
			offset.ry = ap2.ry;
			offset.rz = ap2.rz;
			return true;
		}
		
	}
	return false;
}

void SpecificWorker::saveCanonPose(const string &label, const int numPoseToSave, const pose6D &tag1, const pose6D &tag2, const pose6D &tag3, const pose6D &tag4, const pose6D &tag5, const pose6D &tag6, const pose6D &tag7, const pose6D &tag8, const pose6D &tag9)
{
	qDebug()<<__FUNCTION__;
	string path="/home/robocomp/robocomp/components/prp/objects/"+label+"/";
	poses_inner = new InnerModel();
	 
	int j = 0;
	num_pose = 0;
	for(std::vector<pcl::PointCloud<PointT>::Ptr>::const_iterator it = cluster_clouds.begin(); it != cluster_clouds.end(); ++it)
	{
		if(j==numPoseToSave)
		{
			//check if appril seen
			pose6D offset;
			if(aprilSeen(offset, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9))
			{
				//add the pose to innermodel
				InnerModelNode *root_node = poses_inner->getRoot();
				InnerModelTransform *node = poses_inner->newTransform("canon_pose", "static", root_node, offset.tx, offset.ty, offset.tz, offset.rx, offset.ry, offset.rz);
				root_node->addChild(node);
				
				//save the cloud 
				std::cout << "PointCloud representing the Cluster: " << (*it)->points.size() << " data points." << std::endl;
		
				std::stringstream ss;
				ss <<path<<"canon_pose_" << label;
		
				writer.write<PointT> (ss.str () + ".pcd", **it, false);
// 				visualize(*it);
			}
			else
			{
				qFatal("CAN'T SEE ANY APRIL!");
			}
		}
		j++;
	}
	std::string inner_name = path+label + ".xml";
	poses_inner->save(QString(inner_name.c_str()));
	delete (poses_inner);
	qDebug()<<"End "<<__FUNCTION__;
} 

void SpecificWorker::saveRegPose(const string &label, const int numPoseToSave, const pose6D &tag1, const pose6D &tag2, const pose6D &tag3, const pose6D &tag4, const pose6D &tag5, const pose6D &tag6, const pose6D &tag7, const pose6D &tag8, const pose6D &tag9)
{
	qDebug()<<__FUNCTION__;
	poses_inner = new InnerModel();
	string path="/home/robocomp/robocomp/components/prp/objects/"+label+"/";

	std::string inner_name = path+label + ".xml";
	poses_inner->open(inner_name);
	int j = 0;
	
	for(std::vector<pcl::PointCloud<PointT>::Ptr>::const_iterator it = cluster_clouds.begin(); it != cluster_clouds.end(); ++it)
	{
		if(j==numPoseToSave)
		{
			//check if appril seen
			pose6D offset;
			if(aprilSeen(offset, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9))
			{
				//add the pose to innermodel
				InnerModelNode *parent_node = poses_inner->getTransform("canon_pose");
				std::stringstream ss;
				ss <<"pose_"<<num_pose<<"_"<< label;
				
				InnerModelTransform *node = poses_inner->newTransform(ss.str().c_str(), "static", parent_node, offset.tx, offset.ty, offset.tz, offset.rx, offset.ry, offset.rz);
				parent_node->addChild(node);
				
				//save the cloud 
				std::cout << "PointCloud representing the Cluster: " << (*it)->points.size() << " data points." << std::endl;
				
				std::stringstream ss1;
				ss1 <<path<<"pose_"<<num_pose<<"_"<< label;
				std::cout <<ss1.str()<<endl;
				writer.write<PointT> (ss1.str () + ".pcd", **it, false);
// 				visualize(*it);
			}
			else
			{
				qFatal("CAN'T SEE ANY APRIL!");
			}
		}
		j++;
	}
	string imagename = path +"pose_" + QString::number(num_pose).toStdString() + "_" + label + ".png";
	cv::imwrite( imagename ,rgb_image);
	num_pose++;
	poses_inner->save(QString(inner_name.c_str()));
	delete (poses_inner);	
	qDebug()<<"End "<<__FUNCTION__;
}

void SpecificWorker::guessPose(const string &label, pose6D &guess)
{
	qDebug()<<__FUNCTION__;
	string path="/home/robocomp/robocomp/components/prp/objects/"+label+"/";
	//Load mathing view and find transform to real point cloud
	pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
	//change vfh extension to pcd
	std::string view_to_load = file_view_mathing;
	
	if (pcl::io::loadPCDFile<PointT> (view_to_load, *object) == -1) //* load the file
	{
		printf ("Couldn't read file test_pcd.pcd \n");
	}
	
	pcl::PointCloud<PointT>::Ptr scene = cluster_clouds[num_object_found];
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
	const float leaf = 5;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);
	
	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP< PointT ,pcl::Normal> nest;
	nest.setRadiusSearch (10);
	nest.setInputCloud (object);
	nest.compute (*object_normals);
	nest.setInputCloud (scene);
	nest.compute (*scene_normals);
	
	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	pcl::FPFHEstimationOMP<PointT,pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch (25);
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
	align.setMaximumIterations (50000); // Number of RANSAC iterations
	align.setNumberOfSamples (2); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{	
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
	
	Eigen::Matrix4f transformation = align.getFinalTransformation ();
	Eigen::Vector3f ea = transformation.block<3,3>(1,1).eulerAngles(0, 1, 2);
	
	float tx, ty, tz, rx, ry, rz;
	tx = transformation(3,0);
	ty = transformation(3,1);
	tz = transformation(3,2);
	rx = ea(0);
	ry = ea(1);
	rz = ea(2);
	
#if DEBUG
	std::cout<<"Ransac Translation: "<<tx<<" "<<ty<<" "<<tz<<std::endl;
	std::cout<<"Ransac Rotation: "<<rx<<" "<<ry<<" "<<rz<<std::endl;
#endif
	
	
	//load innermodel and calculate translation respect root
	poses_inner = new InnerModel();
	std::string inner_name = path + label + ".xml";
	poses_inner->open(inner_name);
	
	//get transform name (same as pcd file name)
	std::string node_name = file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	node_name = node_name.substr(file_view_mathing.find_last_of("/")+1);
	std::cout<<node_name<<endl;
	//parent node will be the matching view
	InnerModelNode *parent_node = poses_inner->getTransform(node_name.c_str());
	InnerModelTransform *node = poses_inner->newTransform("current_live_view", "static", parent_node, tx, ty, tz, rx, ry, rz);
	parent_node->addChild(node);
	
	//calculate transform to canon pose
	RTMat transform_to_canon = poses_inner->getTransformationMatrix("canon_pose", "current_live_vew");
	
	guess.tx = transform_to_canon.getTr()[0];
	guess.ty = transform_to_canon.getTr()[1];
	guess.tz = transform_to_canon.getTr()[2];
	guess.rx = transform_to_canon.getRxValue();
	guess.ry = transform_to_canon.getRyValue();
	guess.rz = transform_to_canon.getRzValue();
	
}

void SpecificWorker::passThrough()
{
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (this->cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (marca_tz, marca_tz + 1000);
  //pass.setFilterLimitsNegative (true);
        
        pass.filter (*this->cloud);
	pass.setInputCloud (this->cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (marca_tx, marca_tx + 1000);
        pass.setInputCloud (this->cloud);
	pass.filter (*this->cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (marca_ty-200, marca_ty+1000);
	pass.setInputCloud (this->cloud);
	pass.filter (*this->cloud);
#if DEBUG
        timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_passThrough.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
#endif
        
        
}

void SpecificWorker::surfHomography(listType &guesses)
{

}

void SpecificWorker::fitTheViewVFH()
{

}

void SpecificWorker::showObject(const int numObject)
{

}

void SpecificWorker::convexHull(const string &model)
{
	
#if DEBUG
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "laositafira.pcd";
		printf("<%s>\n", pcdname.c_str());
		writer.write<PointT> ( pcdname, *projected_plane, false);
#endif
		
	table->board_convex_hull(projected_plane, cloud_hull);
// 		  // Load in the point cloud
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ> ());
// 	if (pcl::io::loadPCDFile (pcdname, *cloud_in) != 0)
// 	{
// 		cout<<"putaaaaaaaaa"<<endl;
// 	}
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
// 	pcl::ConcaveHull<pcl::PointXYZ> chull;
// // 	cout<<"Mecagoentodolavirgen!!: "<<cloud_in->points.size()<<endl;
// // 	pcl::PolygonMesh mesh_out;
// //     chull.setInputCloud (cloud_in);
// //     chull.reconstruct (mesh_out);
// 
// 	printf ("This is line %d of file \"%s\".\n",
//             __LINE__, __FILE__);
// // 	pcl::ConcaveHull<PointT> chull;
// 	printf ("This is line %d of file \"%s\".\n",
//             __LINE__, __FILE__);
// 	chull.setInputCloud (cloud_in);
// 	printf ("This is line %d of file \"%s\".\n",
//             __LINE__, __FILE__);
// 	chull.setAlpha (0.1);
// 	printf ("This is line %d of file \"%s\".\n",
//             __LINE__, __FILE__);
// 	chull.reconstruct (*cloud_out);
// 	printf ("This is line %d of file \"%s\".\n",
//             __LINE__, __FILE__);
// //     table->board_convex_hull(projected_plane, cloud_hull);
// //     #if DEBUG
    std::cout<<"Cloud hull size joder: "<<cloud_hull->points.size()<<std::endl;
// //     #endif
}

void SpecificWorker::mirrorPC()
{

}

void SpecificWorker::statisticalOutliersRemoval()
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (this->cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*(this->cloud));
}

void SpecificWorker::loadTrainedVFH()
{
	vfh_matcher->loadTrainingData();
	std::cout<<"Training data loaded"<<std::endl;
}

void SpecificWorker::reset()
{

}

void SpecificWorker::normalSegmentation(const string &model)
{

}

void SpecificWorker::getInliers(const string &model)
{

}

void SpecificWorker::vfh(listType &guesses)
{
    int object__to_show = 0;
    vfh_matcher->doTheGuess(cluster_clouds[object__to_show], vfh_guesses);
	for(auto a:vfh_guesses)
		guesses.push_back(a.file);
// 	guesses = vfh_guesses;
}

void SpecificWorker::grabThePointCloud(const string &image, const string &pcd)
{
	cout<<__FUNCTION__<<endl;
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
	}
	catch(Ice::Exception e)
	{
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
		return;
	}

#if DEBUG
		cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
#endif
	
	for(unsigned int i=0; i<rgbMatrix.size(); i++)
	{
		int row = (i/640), column = i-(row*640);
		rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
	}
	QMat PP = viewpoint_transform;
	
	cloud->points.resize(points_kinect.size());
	
	
	for (unsigned int i=0; i<points_kinect.size(); i++)
	{
// 		memcpy(&cloud->points[i], &points_kinect[i],3*sizeof(float));
		
		QVec p1 = QVec::vec4(points_kinect[i].x, points_kinect[i].y, points_kinect[i].z, 1);
//  	QVec p2 = PP * p1;
 		QVec p22 = (PP * p1).fromHomogeneousCoordinates();
// 		QVec p22 = p1.fromHomogeneousCoordinates();
		
		memcpy(&cloud->points[i],p22.data(),3*sizeof(float));
		
		cloud->points[i].r=rgbMatrix[i].red;
		cloud->points[i].g=rgbMatrix[i].green;
		cloud->points[i].b=rgbMatrix[i].blue;
	}
	cloud->width = 1;
	cloud->height = points_kinect.size();

	
	std::vector< int > index;
	removeNaNFromPointCloud (*cloud, *cloud, index);
	
#if DEBUG
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		string pcdname =  "/home/robocomp/robocomp/components/prp/objects/" + QString::number(ts.tv_sec).toStdString() + ".pcd";
		printf("<%s>\n", pcdname.c_str());
		writer.write<PointT> ( pcdname, *cloud, false);
		pcdname = "/home/robocomp/robocomp/components/prp/scene/" + std::to_string(num_scene) + "_scene.pcd";
		writer.write<PointT> ( pcdname , *cloud, false);
			
		string imagename = "/home/robocomp/robocomp/components/prp/objects/" + QString::number(ts.tv_sec).toStdString() + ".png";
		cv::imwrite( imagename ,rgb_image);
#endif

}

void SpecificWorker::fitModel(const string &model, const string &method)
{

}

void SpecificWorker::setContinousMode(const bool &mode)
{

}

void SpecificWorker::projectInliers(const string &model)
{
	table->project_board_inliers(this->cloud, ransac_inliers, projected_plane);
#if DEBUG
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_projectInliers.pcd";
		printf("<%s>\n", pcdname.c_str());
		writer.write<PointT> ( pcdname, *projected_plane, false);
#endif
}

void SpecificWorker::extractPolygon(const string &model)
{
	cout<<"CloudHull size: "<<cloud_hull->points.size()<<endl;
	cout<<"Cloud size: "<<cloud->points.size()<<endl;

	table->extract_table_polygon(this->cloud, cloud_hull, QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)) , 20, 1500, prism_indices, this->cloud);
	QVec::vec3(viewpoint_transform(0,3), viewpoint_transform(1,3), viewpoint_transform(2,3)).print("Viewpoint: ");
//  table->extract_table_polygon(this->cloud, cloud_hull, QVec::vec3(0,0,1320) , 20, 1500, prism_indices, this->cloud);        
		
	cout<<"Prism size: "<<prism_indices->indices.size()<<endl;
	cout<<"Point Cloud size: "<<this->cloud->points.size()<<endl;
#if DEBUG
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		string pcdname =  "/home/robocomp/robocomp/components/perception/" + QString::number(ts.tv_sec).toStdString() + "_extractPolygon.pcd";
		printf("<%s>\n", pcdname.c_str());
		writer.write<PointT> ( pcdname, *cloud, false);
#endif
}

bool SpecificWorker::findTheObject(const string &objectTofind)
{
	std::string guessgan="";
	while(!V_text_item.empty())
	{
		scene.removeItem(V_text_item.back());
		V_text_item.pop_back();
	}
	float dist=3.40e38;
	for(unsigned int i=0; i<cluster_clouds.size();i++)
	{
		vfh_matcher->doTheGuess(cluster_clouds[i], vfh_guesses);
		
		VFH::file_dist_t second;
		for(auto dato:vfh_guesses)if(dato.label!=vfh_guesses[0].label){ second=dato; break;}
// 		std::cout<<vfh_guesses[0].label<<"   ----   "<< second.label<< "   ----   "<< vfh_guesses[0].dist/second.dist<<std::endl;

		if(vfh_guesses[0].dist/second.dist<THRESHOLD)
		{
			if(dist>vfh_guesses[0].dist && (vfh_guesses[0].label==objectTofind||objectTofind==""))
			{
				guessgan=vfh_guesses[0].label;
				dist=vfh_guesses[0].dist;
				num_object_found = i;
				file_view_mathing = vfh_guesses[0].file;
			}
			if(objectTofind=="")
			{
				settexttocloud(guessgan,cluster_clouds[i]);
				dist=3.40e38;
				guessgan="";
			}
		}
	}
	if((guessgan!=""&&guessgan==objectTofind))
	{
		paintcloud(cluster_clouds[num_object_found]);
		settexttocloud(guessgan,cluster_clouds[num_object_found]);
		std::cout<<file_view_mathing<<guessgan<<" "<<dist<<" "<<num_object_found<<endl;
		return true;
	}
	return false;
}

//todo again
void SpecificWorker::getPose(float &x, float &y, float &z)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cluster_clouds[num_object_found], centroid); 
	x = centroid[0]*1000.;
	y = centroid[1]*1000.;
	z = centroid[2]*1000.;
	
}
//todo again
void SpecificWorker::getRotation(float &rx, float &ry, float &rz)
{
	// Point clouds
	string guessgan=file_view_mathing.substr(0, file_view_mathing.find_last_of("/"));
	guessgan = guessgan.substr(guessgan.find_last_of("/")+1);
	string pathxml="/home/robocomp/robocomp/components/prp/objects/"+guessgan+"/"+guessgan+".xml";
	pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
	//change vfh extension to pcd
	std::string view_to_load = file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	view_to_load = view_to_load + ".pcd"; 
	if (pcl::io::loadPCDFile<PointT> (view_to_load, *object) == -1) //* load the file
	{
		printf ("Couldn't read file test_pcd.pcd \n");
	}
	
	pcl::PointCloud<PointT>::Ptr scene = cluster_clouds[num_object_found];
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
	const float leaf = 5;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);
	
	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP< PointT ,pcl::Normal> nest;
	nest.setRadiusSearch (10);
	nest.setInputCloud (object);
	nest.compute (*object_normals);
	nest.setInputCloud (scene);
	nest.compute (*scene_normals);
	
	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	pcl::FPFHEstimationOMP<PointT,pcl::Normal, pcl::FPFHSignature33> fest;
	fest.setRadiusSearch (25);
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
	align.setMaximumIterations (50000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (5); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
	{	
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}
	
	Eigen::Matrix4f transformation = align.getFinalTransformation ();
	Eigen::Vector3f ea = transformation.block<3,3>(1,1).eulerAngles(0, 1, 2);
	qDebug()<<"Rotation"<<ea(0)<<", "<<ea(1)<<", "<<ea(2);
	rx = ea(0);
	ry = ea(1);
	rz = ea(2);
// 	vector<pcl::PointCloud< PointT >::Ptr>clouds;
// 	clouds.push_back(scene);
// 	clouds.push_back(object_aligned);
// 	visualize(clouds);
	string node_name=file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	node_name=node_name.substr(node_name.find_last_of("/")+1);
	InnerModel inner(pathxml);
	QVec rot = (inner.getRotationMatrixTo("canon_pose",QString::fromStdString(node_name))).extractAnglesR_min();
	qDebug()<<rot;
	rx+=rot(0);
	ry+=rot(1);
	rz+=rot(2);
	std::cout<<"Alignmet end results: rx = "<<rx<<"; ry = "<<ry<<", rz = "<<rz<<std::endl;

#if DEBUG
	std::cout<<"Rotation: "<<rx<<" "<<ry<<" "<<rz<<std::endl;
#endif
	
}

void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
	
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	QMutexLocker locker(&april_mutex);
	this->tags = tags;
	april_mutex.unlock();
}

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "r" && event.keyDown ())
	{
		//     std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf (str, "text#%03d", i);
			viewer->removeShape (str);
		}
		text_id = 0;
	}
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
		event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		//     std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

		char str[512];
		sprintf (str, "text#%03d", text_id ++);
		//     viewer->addText ("clicked here", event.getX (), event.getY (), str);
	}
}


void SpecificWorker::visualize(vector<pcl::PointCloud< PointT >::Ptr> clouds)
{
	unsigned int id=0;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	for(auto cloud:clouds)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, QString::number(id).toStdString());
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, QString::number(id).toStdString());
		id++;
	}
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
	viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());
	
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
// 	HWND hWnd = (HWND)viewer->getRenderWindow()->GetGenericWindowId(); 
// 	viewer->getRenderWindow()
}

void SpecificWorker::settexttocloud(string name, pcl::PointCloud< PointT >::Ptr cloud)
{
	QGraphicsTextItem *text=new QGraphicsTextItem(QString::fromStdString(name));
	QFont serifFont("Times", 25, QFont::Bold);
	text->setFont(serifFont);
	InnerModelCamera *camera = innermodel->getCamera(id_camera);
	int end=cloud->size()-1;
	QVec xy = camera->project(id_robot, QVec::vec3(cloud->points[end].x, cloud->points[end].y, cloud->points[end].z)); 
	QVec xyfrist = camera->project(id_robot, QVec::vec3(cloud->points[0].x, cloud->points[0].y, cloud->points[0].z)); 
	text->setPos(xyfrist(0)-30,(int)(xy(1)+xyfrist(1))/2);
	scene.addItem(text);
	V_text_item.push_back(text);
}

void SpecificWorker::paintcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
	while(!V_pixmap_item.empty())
	{
		scene.removeItem(V_pixmap_item.back());
		V_pixmap_item.pop_back();
	}
	InnerModelCamera *camera = innermodel->getCamera(id_camera);
	QImage image(640,480,QImage::Format_ARGB32_Premultiplied);
	image.fill(Qt::transparent);
	int max=cloud->points[0].z,min=cloud->points[0].z;
	for(unsigned int i=1;i<cloud->points.size();i++)
	{
		if(cloud->points[i].z>max)
			max=cloud->points[i].z;
		if(cloud->points[i].z<min)
			min=cloud->points[i].z;
	}
	qDebug()<<max<<" "<<min;
	for(unsigned int i=0;i<cloud->points.size();i++)
	{
		QVec xy = camera->project(id_robot, QVec::vec3(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
		
		int x = xy(0), y = xy(1);
		if (xy(0)>=0 and xy(0) < 640 and xy(1)>=0 and xy(1) < 480 )
		{
// 		M.at<uchar> ((int)xy(1), (int)xy(0)) = 255;
			unsigned int color=(cloud->points[i].z - min) * -254 / (max - min)-254;
			image.setPixel(x-15,y+10,qRgb(color, 0, 0));
		}
		else if (not (isinf(xy(1)) or isinf(xy(0))))
		{
			std::cout<<"Accediendo a -noinf: "<<xy(1)<<" "<<xy(0)<<std::endl;
		}
		
// 		QGraphicsEllipseItem* e=new QGraphicsEllipseItem(x,y,1,1);
// 		e->setBrush(QBrush(Qt::blue));
// 		scene.addItem(e);
// 		rgb_image.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
	}

// 	cv::Mat dest;
//     cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
//     QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	V_pixmap_item.push_back(new QGraphicsPixmapItem(QPixmap::fromImage(image)));
	scene.addItem(V_pixmap_item.back());
}




