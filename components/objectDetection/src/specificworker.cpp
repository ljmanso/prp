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
#ifdef USE_QTGUI
,viewer(new Viewer(MEDIDA))
#endif
{
	//let's set the sizes
	table->set_board_size(500/MEDIDA,30/MEDIDA,500/MEDIDA);
	marca_tx = marca_ty = marca_tz = marca_rx = marca_ry = marca_rz = 0;
	
	num_object_found = 0;
	num_scene = 15;
#ifdef USE_QTGUI
	graphic->setScene(&scene);
	graphic->show();
	item_pixmap=new QGraphicsPixmapItem();
	scene.addItem(item_pixmap);
// 	viewer=NULL;
	viewer->addPointCloud(cloud,"scene",1,0,0,0);
#endif
	boost::filesystem::remove("training_data.h5");
	boost::filesystem::remove("training_data.list");	
}

/**
* \brief Default destructor
*/

SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	string name = PROGRAM_NAME;
	innermodel = new InnerModel(params[name+".innermodel"].value);
	id_robot=QString::fromStdString(params[name+".id_robot"].value);
	id_camera=QString::fromStdString(params[name+".id_camera"].value);
	id_camera_transform=QString::fromStdString(params[name+".id_camera_transform"].value);
	pathLoadDescriptors = params[name+".pathLoadDescriptors"].value;
	viewpoint_transform = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
	vfh_matcher->set_type_feature(params[name+".type_features"].value);
	if(params[name+".type_features"].value=="VFH")
		descriptors_extension="vfh";
	else if(params[name+".type_features"].value=="CVFH")
		descriptors_extension="cvfh";
	else if(params[name+".type_features"].value=="OUR-CVFH")
		descriptors_extension="ourcvfh";
	std::cout<<params[name+".type_features"].value<<" " <<descriptors_extension<<std::endl;
	if(params[name+".test"].value=="1")
	{
		std::cout<<"Modo test activo"<<std::endl;
		test=true;
	}
	reloadVFH();
	timer.start(10);
	return true;
}

void SpecificWorker::updateinner()
{
	if(test)
			innermodel->updateJointValue(QString::fromStdString("head_pitch_joint"),0.8);
	else
	{
		MotorList motors;
		motors.push_back("head_yaw_joint");
		motors.push_back("head_pitch_joint");
		MotorStateMap motors_states = jointmotor_proxy->getMotorStateMap(motors);
		for(auto motor:motors)
		{
			innermodel->updateJointValue(QString::fromStdString(motor),motors_states[motor].pos);
		}
	}
}

void SpecificWorker::updatergbd()
{
#ifdef USE_QTGUI
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	cv::Mat rgb_image(480,640, CV_8UC3, cv::Scalar::all(0));
	if(test)
	{
		rgb_image = cv::imread("/home/robocomp/robocomp/components/prp/scene/Scene.png");

		if(! rgb_image.data )
		{
			cout <<  "Could not open or find the image rgb.png" << std::endl ;
		}	
	}
	else
	{
		rgbd_proxy->getRGB(rgbMatrix,h,b);
		for(unsigned int i=0; i<rgbMatrix.size(); i++)
		{
			int row = (i/640), column = i-(row*640);
			rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
		}
// 		pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
// 		RoboCompRGBD::ColorSeq rgbMatrix;	
// 		RoboCompRGBD::depthType distanceMatrix;
// 		RoboCompRGBD::PointSeq points_kinect;
// 		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
// 		for(unsigned int i=0; i<rgbMatrix.size(); i++)
// 		{
// 			int row = (i/640), column = i-(row*640);
// 			rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
// 		}
// 		QMat PP = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
// 		
// 		scene->points.resize(points_kinect.size());
// 		
// 		for (unsigned int i=0; i<points_kinect.size(); i++)
// 		{
// 			QVec p1 = (PP * QVec::vec4(points_kinect[i].x, points_kinect[i].y, points_kinect[i].z, 1)).fromHomogeneousCoordinates();
// 			
// 			memcpy(&scene->points[i],p1.data(),3*sizeof(float));
// 			
// 			scene->points[i].r=rgbMatrix[i].red;
// 			scene->points[i].g=rgbMatrix[i].green;
// 			scene->points[i].b=rgbMatrix[i].blue;
// 		}
// 		scene->width = 1;
// 		scene->height = points_kinect.size();
// 		std::vector< int > index;
// 		removeNaNFromPointCloud (*scene, *scene, index);
// 		updatePointCloud(scene,"scene");
	}
	
	cv::Mat dest;
	cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
	QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	item_pixmap->setPixmap(QPixmap::fromImage(image));
	scene.update();
#endif
}

void SpecificWorker::compute()
{
// 	qDebug()<<"entro";
	updateinner();
#ifdef USE_QTGUI
	updatergbd();
	viewer->update();
#endif
}

/*
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
*/

void SpecificWorker::reloadVFH()
{
	string s="./bin/createDescriptors "+pathLoadDescriptors +" "+ descriptors_extension;
	char *cstr = &s[0u];
	if (system(cstr)==0)
	{
		vfh_matcher->reloadVFH(pathLoadDescriptors);
		vfh_matcher->loadTrainingData();
	}
}

QVec extraerposefromTM(QMat M)
{
	QVec initVec = QVec::vec6(0,0,0,0,0,0);
	const QVec a = (M * initVec.subVector(0,2).toHomogeneousCoordinates()).fromHomogeneousCoordinates();
	const Rot3D R(initVec(3), initVec(4), initVec(5));
	const QVec b = (M.getSubmatrix(0,2,0,2)*R).extractAnglesR_min();
	QVec ret(6);
	ret(0) = a(0);
	ret(1) = a(1);
	ret(2) = a(2);
	ret(3) = b(0);
	ret(4) = b(1);
	ret(5) = b(2);
	return ret;
}

bool SpecificWorker::aprilSeen(pose6D &offset)
{
	QMutexLocker locker(&april_mutex);
	static InnerModel tabletags("/home/robocomp/robocomp/components/robocomp-shelly/files/tabletags.xml");
	QMat transformM,cameratoapril5M, cameratorobot, cameratoaprilseeM;
	for (auto ap : tags)
	{
		if(ap.id < 10 and ap.id > 0)
		{
			cameratoaprilseeM = RTMat(ap.rx, ap.ry, ap.rz, ap.tx, ap.ty, ap.tz);
			transformM =tabletags.getTransformationMatrix(QString("tag")+QString::number(ap.id),"tag5");
			cameratorobot = innermodel->getTransformationMatrix(id_robot,id_camera);
			cameratoapril5M = cameratorobot * cameratoaprilseeM * transformM;
			QVec ret = extraerposefromTM(cameratoapril5M);
			ret.print("tag5 from robot");
			offset.tx = ret(0);
			offset.ty = ret(1);
			offset.tz = ret(2);
			offset.rx = ret(3); 
			offset.ry = ret(4);
			offset.rz = ret(5);
			return true;
		}
	}
	return false;
}

void SpecificWorker::initSaveObject(const string &label, const int numPoseToSave)
{
	caputurePointCloudObjects();
	qDebug()<<__FUNCTION__;
	boost::filesystem::path path=boost::filesystem::path("/home/robocomp/robocomp/components/prp/objects/"+label+"/");
	if(!boost::filesystem::exists(path))
		boost::filesystem::create_directories(path);
	poses_inner = new InnerModel();
	 
// 	int j = 0;
	num_pose = 0;
// 	for(std::vector<pcl::PointCloud<PointT>::Ptr>::const_iterator it = cluster_clouds.begin(); it != cluster_clouds.end(); ++it)
// 	{
// 		if(j==numPoseToSave)
// 		{
// 			//check if appril seen
// 			pose6D offset;
// 			if(aprilSeen(offset))
// 			{
// 				QVec poseoffset =QVec::vec6(offset.tx, offset.ty, offset.tz, offset.rx, offset.ry, offset.rz);
// 				poseoffset = innermodel->transform6D(id_robot, poseoffset, id_camera_transform);
// 				
// 				InnerModelNode *root_node = poses_inner->getRoot();
// 				InnerModelTransform *node = poses_inner->newTransform("canon_pose", "static", root_node, poseoffset.x(), poseoffset.y(), poseoffset.z(), poseoffset.rx(), poseoffset.ry(), poseoffset.rz());
// 				root_node->addChild(node);
// 				
// 				//save the cloud 
// 				std::cout << "PointCloud representing the Cluster: " << (*it)->points.size() << " data points." << std::endl;
// 		
// 				std::stringstream ss;
// 				ss <<path<<"canon_pose_" << label;
// 		
// 				writer.write<PointT> (ss.str () + ".pcd", **it, false);
// // 				visualize(*it);
// 			}
// 			else
// 			{
// 				qFatal("CAN'T SEE ANY APRIL!");
// 			}
// 		}
// 		j++;
// 	}
	std::string inner_name = path.string()+label + ".xml";
	poses_inner->save(QString(inner_name.c_str()));
	delete (poses_inner);
	qDebug()<<"End "<<__FUNCTION__;
} 

void SpecificWorker::saveRegPose(const string &label, const int numPoseToSave)
{
	caputurePointCloudObjects();
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
			if(aprilSeen(offset))
			{
				//add the pose to innermodel
				QVec poseoffset =QVec::vec6(offset.tx, offset.ty, offset.tz, offset.rx, offset.ry, offset.rz);
// 				poseoffset = innermodel->transform6D(id_robot, poseoffset, id_camera_transform);
				
				
				InnerModelNode *parent_node = poses_inner->getTransform("root");
				std::stringstream ss;
				ss <<"pose_"<<num_pose<<"_"<< label;
				
				InnerModelTransform *node = poses_inner->newTransform(ss.str().c_str(), "static", parent_node, poseoffset.x(), poseoffset.y(), poseoffset.z(), poseoffset.rx(), poseoffset.ry(), poseoffset.rz());
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

/*
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

void SpecificWorker::statisticalOutliersRemoval()
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (this->cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*(this->cloud));
}*/

void SpecificWorker::loadTrainedVFH()
{
	vfh_matcher->loadTrainingData();
	std::cout<<"Training data loaded"<<std::endl;
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
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
#if DEBUG
		cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
#endif
		for(unsigned int i=0; i<rgbMatrix.size(); i++)
		{
			int row = (i/640), column = i-(row*640);
			rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
		}
		viewpoint_transform = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
		QMat PP = viewpoint_transform;
		
		cloud->points.resize(points_kinect.size());
		
		for (unsigned int i=0; i<points_kinect.size(); i++)
		{
			QVec p1 = (PP * QVec::vec4(points_kinect[i].x, points_kinect[i].y, points_kinect[i].z, 1)).fromHomogeneousCoordinates();
			
			memcpy(&cloud->points[i],p1.data(),3*sizeof(float));
			
			cloud->points[i].r=rgbMatrix[i].red;
			cloud->points[i].g=rgbMatrix[i].green;
			cloud->points[i].b=rgbMatrix[i].blue;
		}
		cloud->width = 1;
		cloud->height = points_kinect.size();
// 		Convert cloud from mm to m
		if(MEDIDA==1000.)
			cloud = PointCloudfrom_mm_to_Meters(cloud);

		std::vector< int > index;
		removeNaNFromPointCloud (*cloud, *cloud, index);
		cloud->is_dense = false;
	
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
	catch(Ice::Exception e)
	{
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
		return;
	}
}

void SpecificWorker::readThePointCloud(const string &image, const string &pcd)
{
    rgb_image = cv::imread(image);
    
    if(! rgb_image.data )                              // Check for invalid inpute
    {
        cout <<  "Could not open or find the image " << image << std::endl ;
    }
    
    if (pcl::io::loadPCDFile<PointT> (pcd, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file pcd.pcd \n");
    }
    if (MEDIDA==1000.)
		cloud = PointCloudfrom_mm_to_Meters(cloud);
}

void SpecificWorker::ransac(const string &model)
{
	//remove the floor before the ransac
	// Create the filtering object
	string pcdname =  "/home/robocomp/robocomp/components/perception/beforePASSTHROUGH.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-100, 500.0);
	pass.setFilterLimitsNegative (true);
	pass.filter (*cloud);
	pcdname =  "/home/robocomp/robocomp/components/perception/beforeRANSAC.pcd";
	printf("<%s>\n", pcdname.c_str());
	writer.write<PointT> ( pcdname, *cloud, false);
	
	
	table->fit_board_with_RANSAC( cloud, ransac_inliers, 15/MEDIDA);
	cout<<"RANSAC INLIERS: "<<ransac_inliers->indices.size()<<endl;
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
#if DEBUG
    std::cout<<"Cloud hull size joder: "<<cloud_hull->points.size()<<std::endl;
#endif
}

void SpecificWorker::extractPolygon(const string &model)
{
	cout<<"CloudHull size: "<<cloud_hull->points.size()<<endl;
	cout<<"Cloud size: "<<cloud->points.size()<<endl;
	QVec viewpoint = QVec::vec3(viewpoint_transform(0,3)/MEDIDA, viewpoint_transform(1,3)/MEDIDA, viewpoint_transform(2,3)/MEDIDA);
	table->extract_table_polygon(this->cloud, cloud_hull, viewpoint , 20/MEDIDA, 1500/MEDIDA, prism_indices, this->cloud);
	viewpoint.print("Viewpoint: ");		
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

void SpecificWorker::euclideanClustering(int &numCluseters)
{
// 	pcl::cuda::OrganizedRadiusSearch<PointT>  ors(this->cloud);
// 	ors->
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (this->cloud);
	cluster_indices.clear();
	cluster_clouds.clear();
	pcl::EuclideanClusterExtraction<PointT> ec;
	
	ec.setClusterTolerance (40/MEDIDA); 
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (50000);
	ec.setSearchMethod (tree);
	
	ec.setInputCloud (this->cloud);
	ec.extract (cluster_indices);
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (this->cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//save the cloud at 
		cluster_clouds.push_back(cloud_cluster);
#if DEBUG
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
#endif

#if SAVE_DATA	
			std::stringstream ss;
			ss <<"/home/robocomp/robocomp/components/prp/scene/"<<num_scene<<"_capture_object_" << j;
		
                /////save /*rgbd*/ 

			cv::Mat M(480,640,CV_8UC1, cv::Scalar::all(0));
			for (unsigned int i = 0; i<cloud_cluster->points.size(); i++)
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
			writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false); 
#endif
			j++;
	}
	qDebug()<<"Clouds of points captured: "<< cluster_clouds.size();
	num_scene++;

}

bool SpecificWorker::findTheObject(const string &objectTofind, pose6D &pose)
{
	caputurePointCloudObjects();

	std::string guessgan="";
#ifdef USE_QTGUI
	while(!V_text_item.empty())
	{
		scene.removeItem(V_text_item.back());
		V_text_item.pop_back();
	}
#endif
	struct timespec Inicio, Fin, resta;
	clock_gettime(CLOCK_REALTIME, &Inicio);
	float dist=3.40e38;
	for(unsigned int i=0; i<cluster_clouds.size();i++)
	{
		vfh_matcher->doTheGuess(cluster_clouds[i], vfh_guesses);
		
		VFH::file_dist_t second;
		for(auto dato:vfh_guesses)if(dato.label!=vfh_guesses[0].label){ second=dato; break;}
		std::cout<<vfh_guesses[0].label<<"   ----   "<< second.label<< "   ----   "<< vfh_guesses[0].dist/second.dist<<std::endl;

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
		std::cout<<file_view_mathing<<guessgan<<" "<<dist<<" "<<num_object_found<<endl;
		clock_gettime(CLOCK_REALTIME, &Fin);
		SUB(&resta, &Fin, &Inicio);
		qDebug()<<"Recognition VFH: "<<resta.tv_sec<<"s "<<resta.tv_nsec<<"ns";
		clock_gettime(CLOCK_REALTIME, &Inicio);
		pose=getPose();
		clock_gettime(CLOCK_REALTIME, &Fin);
		SUB(&resta, &Fin, &Inicio);
		qDebug()<<"Fitting PCD: "<<resta.tv_sec<<"s "<<resta.tv_nsec<<"ns";
		return true;
	}
	return false;
}

bool SpecificWorker::findObjects(listObject& lObjects)
{
	caputurePointCloudObjects();
	std::string guessgan="";
	for(unsigned int i=0; i<cluster_clouds.size();i++)
	{
		vfh_matcher->doTheGuess(cluster_clouds[i], vfh_guesses);

		VFH::file_dist_t second;
		for(auto dato:vfh_guesses)if(dato.label!=vfh_guesses[0].label){ second=dato; break;}
		std::cout<<vfh_guesses[0].label<<"   ----   "<< second.label<< "   ----   "<< vfh_guesses[0].dist/second.dist<<std::endl;

		pose6D p;
		if(vfh_guesses[0].dist/second.dist<THRESHOLD)
		{
			guessgan=vfh_guesses[0].label;
			num_object_found = i;
			file_view_mathing = vfh_guesses[0].file;
			p.label=guessgan;
			p=getPose();
		}
		else
		{
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid (*cluster_clouds[i], centroid);
			p.label="unknown";
			if(MEDIDA==1000.)
			{
				p.tx = centroid[0]*MEDIDA;
				p.ty = centroid[1]*MEDIDA;
				p.tz = centroid[2]*MEDIDA;
			}
			else
			{
				p.tx = centroid[0];
				p.ty = centroid[1];
				p.tz = centroid[2];
			}
		}
		lObjects.push_back(p);
		guessgan="";
	}
	num_object_found=0;
	file_view_mathing="";
	if(cluster_clouds.size()==0)
		return false;
	return true;
}

pose6D  SpecificWorker::getPose()
{
	removeAllpixmap();
	static QMat april = RTMat(-M_PI_2, 0, 0,0,0,0);
	
	//Print centroide
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cluster_clouds[num_object_found], centroid); 
	QVec centroidpose = QVec::vec3(centroid[0]/MEDIDA,centroid[1]/MEDIDA,centroid[2]/MEDIDA);
	centroidpose.print("centroid vista atual");
#ifdef USE_QTGUI
	InnerModelCamera *camera = innermodel->getCamera(id_camera);
	centroidpose= camera->project(id_robot,QVec::vec3(centroidpose.x(),centroidpose.y(),centroidpose.z()));
	QImage image(640,480,QImage::Format_ARGB32_Premultiplied);
	image.fill(Qt::transparent);
	for (int x =centroidpose.x()-4;x<centroidpose.x()+4;x++)
		for (int y =centroidpose.y()-4;y<centroidpose.y()+4;y++)
			image.setPixel(x,y,qRgb(0, 255, 0));
	V_pixmap_item.push_back(new QGraphicsPixmapItem(QPixmap::fromImage(image)));
	SpecificWorker::scene.addItem(V_pixmap_item.back());
#endif
	// Point clouds
	string guessgan=file_view_mathing.substr(0, file_view_mathing.find_last_of("/"));
	guessgan = guessgan.substr(guessgan.find_last_of("/")+1);
	string pathxml="/home/robocomp/robocomp/components/prp/objects/"+guessgan+"/"+guessgan+".xml";
	pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
	//change vfh extension to pcd
	std::string view_to_load = file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	view_to_load = view_to_load + ".pcd"; 
	pcl::PointCloud<PointT>::Ptr object(cluster_clouds[num_object_found]);
	if (pcl::io::loadPCDFile<PointT> (view_to_load, *scene) == -1) //* load the file
	{
		printf ("Couldn't read file test_pcd.pcd \n");
	}
	//convert pointcloud to mm
	scene=PointCloudfrom_mm_to_Meters(scene);
	object=PointCloudfrom_mm_to_Meters(object);
	
	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/seen.pcd", *scene, false);
	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/saved.pcd", *object, false);

// 	-------------------------------------------------------------------

	pcl::PointCloud<PointT>::Ptr object_aligned(new pcl::PointCloud<PointT>);
	QMat saveToViewR = fitingSCP(object,scene,object_aligned);
// 	QMat saveToViewR2 = fitingICP(object_aligned,object,object_aligned);

	//convert pointcloud to meters
// 	object_aligned = PointCloudfrom_Meter_to_mm(object_aligned);
// 	scene = PointCloudfrom_Meter_to_mm(scene);

 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/scene.pcd", *scene, false);
 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/objectaling.pcd", *object_aligned, false);

	string node_name=file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	node_name=node_name.substr(node_name.find_last_of("/")+1);
	InnerModel inner(pathxml);
	QMat PoseSavetoRootR = inner.getTransformationMatrix("root",QString::fromStdString(node_name));

// 	convert m to mm
	QVec saveToView = extraerposefromTM(saveToViewR);
	saveToView(0)=saveToView(0)*1000;
	saveToView(1)=saveToView(1)*1000;
	saveToView(2)=saveToView(2)*1000;
	saveToViewR= (RTMat(saveToView(3), saveToView(4), saveToView(5), saveToView(0), saveToView(1), saveToView(2))).invert();

// 	QVec saveToView2 = extraerposefromTM(saveToViewR2);
// 	saveToView2(0)=saveToView2(0)*1000;
// 	saveToView2(1)=saveToView2(1)*1000;
// 	saveToView2(2)=saveToView2(2)*1000;
// 	saveToViewR2= (RTMat(saveToView2(3), saveToView2(4), saveToView2(5), saveToView2(0), saveToView2(1), saveToView2(2)))/*.invert()*/;

	QMat poseObjR =/*saveToViewR2**/saveToViewR*PoseSavetoRootR*april;
	QVec pose=extraerposefromTM(poseObjR);
	pose.print("pose");
	pose6D poseObj;
	poseObj.tx=pose.x();
	poseObj.ty=pose.y()+125;
	poseObj.tz=pose.z();
	poseObj.rx=pose.rx();
	poseObj.ry=pose.ry();
	poseObj.rz=pose.rz();
// 	Print tag5
#ifdef USE_QTGUI
	pose= camera->project(id_robot,QVec::vec3(pose.x(),pose.y(),pose.z()));
	image.fill(Qt::transparent);
	for (int x =pose.x()-4;x<pose.x()+4;x++)
		for (int y =pose.y()-4;y<pose.y()+4;y++)
			image.setPixel(x,y,qRgb(255, 0, 0));
	V_pixmap_item.push_back(new QGraphicsPixmapItem(QPixmap::fromImage(image)));
	SpecificWorker::scene.addItem(V_pixmap_item.back());
// 	removeCoordinateSystem("poseobject");
	viewer->removeCoordinateSystem("poseobjectR");
// 	addCoordinateSystem(poseObj.tx/1000,poseObj.ty/1000,poseObj.tz/1000,"poseobject");
	poseObjR=RTMat(poseObj.rx,poseObj.ry,poseObj.rz,poseObj.tx/1000,poseObj.ty/1000,poseObj.tz/1000);
	viewer->addCoordinateSystem(poseObjR,"poseobjectR");
	
#endif
	return poseObj;
}

void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
	
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	QMutexLocker locker(&april_mutex);
	if(tags.size()==0)
		return;
	qDebug()<<"Found tags :"<<tags.size();
	this->tags = tags;
}

void SpecificWorker::caputurePointCloudObjects()
{
	static vector<string> id_objects;
#ifdef USE_QTGUI
	while(!id_objects.empty())
	{
		viewer->removePointCloud(id_objects.back());
		id_objects.pop_back();
	}
#endif
	if(test)
		readThePointCloud("/home/robocomp/robocomp/components/prp/scene/Scene.png","/home/robocomp/robocomp/components/prp/scene/Scene.pcd");
	else
		grabThePointCloud("image.png", "rgbd.pcd");
#ifdef USE_QTGUI
	viewer->updatePointCloud(cloud,"scene");
#endif
	struct timespec Inicio, Fin, resta;
	clock_gettime(CLOCK_REALTIME, &Inicio);
	copy_scene=copy_pointcloud(cloud);
	cloud = VoxelGrid_filter(cloud, 3/MEDIDA, 3/MEDIDA, 3/MEDIDA);
	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/VoxelGrid_filter.pcd", *cloud, false);
	ransac("plane");
	projectInliers("plane");
	convexHull("plane");
	extractPolygon("plane");
	int n;
	euclideanClustering(n);
	clock_gettime(CLOCK_REALTIME, &Fin);
	SUB(&resta, &Fin, &Inicio);
	qDebug()<<"Captured: "<<resta.tv_sec<<"s "<<resta.tv_nsec<<"ns";
#ifdef USE_QTGUI
	for(unsigned int i=0;i<cluster_clouds.size();i++)
	{
		viewer->addPointCloud(cluster_clouds[i],QString::number(i).toStdString(),1,255,0,0);
		id_objects.push_back(QString::number(i).toStdString());
	}
#endif
	
	
}

void SpecificWorker::settexttocloud(string name, pcl::PointCloud< PointT >::Ptr cloud)
{
#ifdef USE_QTGUI
	QGraphicsTextItem *text=new QGraphicsTextItem(QString::fromStdString(name));
	QFont serifFont("Times", 25, QFont::Bold);
	text->setFont(serifFont);
	InnerModelCamera *camera = innermodel->getCamera(id_camera);
	int end=cloud->size()-1;
	QVec xy = camera->project(id_robot, QVec::vec3(cloud->points[end].x*MEDIDA, cloud->points[end].y*MEDIDA, cloud->points[end].z*MEDIDA)); 
	QVec xyfrist = camera->project(id_robot, QVec::vec3(cloud->points[0].x*MEDIDA, cloud->points[0].y*MEDIDA, cloud->points[0].z*MEDIDA)); 
	text->setPos(xyfrist(0)-30,(int)(xy(1)+xyfrist(1))/2);
	scene.addItem(text);
	V_text_item.push_back(text);
#endif
}

void SpecificWorker::removeAllpixmap()
{
#ifdef USE_QTGUI
	while(!V_pixmap_item.empty())
	{
		scene.removeItem(V_pixmap_item.back());
		V_pixmap_item.pop_back();
	}
#endif
}

void SpecificWorker::paintcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
#ifdef USE_QTGUI
	removeAllpixmap();
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
			image.setPixel(x,y,qRgb(color, 0, 0));
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
#endif
}
