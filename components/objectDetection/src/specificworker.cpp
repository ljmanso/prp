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
{
	cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
	ransac_inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
	projected_plane = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
	cloud_hull = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>());
	prism_indices = pcl::PointIndices::Ptr(new pcl::PointIndices());
	rgb_image = cv::Mat(480,640, CV_8UC3, cv::Scalar::all(0));
	color_segmented = cv::Mat(480,640, CV_8UC3, cv::Scalar::all(0));
	table = boost::shared_ptr<Table>(new Table());
	descriptor_matcher = boost::shared_ptr<DESCRIPTORS>(new DESCRIPTORS());
#ifdef USE_QTGUI
	viewer = boost::shared_ptr<Viewer>(new Viewer(MEDIDA));
#endif
	tx = 0;
	ty = 0;
	tz = 0;
	rx = 0;
	ry = 0;
	rz = 0;

	test=false;
	//let's set the sizes
	table->set_board_size(500/MEDIDA,30/MEDIDA,500/MEDIDA);

	num_scene = 15;
printf("%s %d\n", __FILE__, __LINE__);
#ifdef USE_QTGUI
printf("%s %d\n", __FILE__, __LINE__);
	graphic->setScene(&scene);
	graphic->show();
	item_pixmap=new QGraphicsPixmapItem();
	scene.addItem(item_pixmap);
	viewer->addPointCloud(cloud,"scene",1,0,0,0);
	connect(reloadButton, SIGNAL(clicked()), this, SLOT(reloadDESCRIPTORS_Button()));
	connect(goButton, SIGNAL(clicked()), this, SLOT(fullRun_Button()));
	connect(findObjectButton, SIGNAL(clicked()), this, SLOT(findTheObject_Button()));
	connect(saveViewButton, SIGNAL(clicked()), this, SLOT(saveView()));
	connect(ResetPoseButton, SIGNAL(clicked()), this, SLOT(ResetPose_Button()));
	guess = QVec::zeros(6);
#endif
	boost::filesystem::remove("training_data.h5");
	boost::filesystem::remove("training_data.list");
	image_path = "/home/robocomp/robocomp/components/prp/scene/Scene.png";
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
	descriptor_matcher->set_type_feature(params[name+".type_features"].value);
	type_fitting = params[name+".type_fitting"].value;
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
	reloadDESCRIPTORS();
	if (test)
	{
		innermodel->updateJointValue(QString::fromStdString("head_pitch_joint"),0.710744);
		innermodel->updateJointValue(QString::fromStdString("head_yaw_joint"),0.0102265);
	}
	timer.start(10);
	return true;
}



void SpecificWorker::compute()
{
	if (!test)
		updateinner();
#ifdef USE_QTGUI
	if(guess != QVec::zeros(6))
	{
		tx = guess.x() + tx_Slider->value()/1.;
		ty = guess.y() + ty_Slider->value()/1.;
		tz = guess.z()/1000. + tz_Slider->value()/1000.;
		rx = guess.rx();
		ry = guess.ry();
		rz = guess.rz();
		QMat poseObjR=RTMat(rx/1000., ry/1000., rz/1000., tx, ty, tz);
		viewer->updateCoordinateSystemPose(poseObjR,"poseobjectR");
	}
	try
	{
			updatergbd();
			viewer->update();
	}
	catch(...){}
#endif
}

/*
 * Method of interface ObjectDetection.ice
 */
void SpecificWorker::computeObjectScene(pcl::PointCloud<PointT>::Ptr obj_scene, ObjectType *Obj, SpecificWorker *s)
{
	std::vector<DESCRIPTORS::file_dist_t> descriptor_guesses;
	// s->matcher_mutex.lock();
	s->descriptor_matcher->doTheGuess(obj_scene, descriptor_guesses);
	// s->matcher_mutex.unlock();
	DESCRIPTORS::file_dist_t second;
	second.label="unknown";
	for(auto dato:descriptor_guesses)if(dato.label!=descriptor_guesses[0].label){ second=dato; break;}

	std::cout<<descriptor_guesses[0].label<<"   ----   "<< second.label<< "   ----   "<< descriptor_guesses[0].dist/second.dist<<std::endl;

	Obj->tx = Obj->ty = Obj->tz = Obj->rx = Obj->ry = Obj->rz = 0;
	#pragma omp parallel sections // divides the team into sections
 	{
		#pragma omp section
		{
			getBoundingBox(obj_scene, Obj->minx, Obj->maxx, Obj->miny, Obj->maxy, Obj->minz, Obj->maxz);
		}
		#pragma omp section
		{
			if( second.label=="unknown" || descriptor_guesses[0].dist/second.dist<THRESHOLD)
			{
				Obj->label = descriptor_guesses[0].label;
				if(s->lObjectsTofind.size()==0 || std::find(s->lObjectsTofind.begin(), s->lObjectsTofind.end(), descriptor_guesses[0].label) != s->lObjectsTofind.end() )
				{
					ObjectType objaux;
					s->getPose(objaux, descriptor_guesses[0].file, obj_scene);
					Obj->tx = objaux.tx;
					Obj->ty = objaux.ty;
					Obj->tz = objaux.tz;
					Obj->rx = objaux.rx;
					Obj->ry = objaux.ry;
					Obj->rz = objaux.rz;
				}
			}
			else
			{
				Obj->label = "unknown";
			}
		}
	}
}


 bool SpecificWorker::findObjects(const StringVector &objectsTofind, ObjectVector &objects)
 {
	 	capturePointCloudObjects();
		lObjectsTofind=objectsTofind;
		struct timespec Inicio_, Fin_, resta_;
		clock_gettime(CLOCK_REALTIME, &Inicio_);
		boost::thread_group threads;
		std::vector<ObjectType*> objectspointer;
	 	for(auto obj_scene:cluster_clouds)
	 	{
			ObjectType *Obj = new ObjectType;
			objectspointer.push_back(Obj);
			threads.add_thread(new boost::thread(SpecificWorker::computeObjectScene, copy_pointcloud(obj_scene), Obj, this));
			// computeObjectScene(copy_pointcloud(obj_scene), Obj, this);
	 	}
		threads.join_all();
	  // std::cout << "Threads Done" << std::endl;
		for(auto obj_aux:objectspointer)
			objects.push_back(*obj_aux);
		clock_gettime(CLOCK_REALTIME, &Fin_);
		SUB(&resta_, &Fin_, &Inicio_);
		qDebug()<<"Tiempo Ejecucion findObjects:  "<<resta_.tv_sec<<"s "<<resta_.tv_nsec<<"ns";
	 	if(cluster_clouds.size()==0)
	 		return false;
	 	return true;

 }

/*
 * Method of interface AprilTags
 */

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

/**
 * Method Private of This Class
 */

QVec SpecificWorker::extraerposefromTM(QMat M)
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

void SpecificWorker::grabThePointCloud()
{
	try
	{
		rgbd_proxy->getImage(rgbMatrix, distanceMatrix, points_kinect,  h, b);
#if DEBUG
		cout<<"SpecificWorker::grabThePointcloud rgbMatrix.size(): "<<rgbMatrix.size()<<endl;
#endif
		#pragma omp parallel for
		for(unsigned int i=0; i<rgbMatrix.size(); i++)
		{
			int row = (i/640), column = i-(row*640);
			rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
		}
		viewpoint_transform = innermodel->getTransformationMatrix(id_robot,id_camera_transform);
		QMat PP = viewpoint_transform;
		cloud->points.resize(points_kinect.size());
		#pragma omp parallel for
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

// #if DEBUG
		timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		string pcdname =  "/home/robocomp/robocomp/components/prp/scene_prueba/" + QString::number(ts.tv_sec).toStdString() + ".pcd";
		printf("<%s>\n", pcdname.c_str());
		writer.write<PointT> ( pcdname, *cloud, false);
		// pcdname = "/home/robocomp/robocomp/components/prp/scene_prueba/" + std::to_string(num_scene) + "_scene.pcd";
		// writer.write<PointT> ( pcdname , *cloud, false);

		string imagename = "/home/robocomp/robocomp/components/prp/scene_prueba/" + QString::number(ts.tv_sec).toStdString() + ".png";
		cv::imwrite( imagename, rgb_image);
// #endif
	}
	catch(Ice::Exception e)
	{
		qDebug()<<"Error talking to rgbd_proxy: "<<e.what();
		return;
	}
}

void SpecificWorker::readThePointCloud(const string &image, const string &pcd)
{
		std::cout<<image<<std::endl;
		std::cout<<pcd<<std::endl;

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

void SpecificWorker::ransac()
{
	table->fit_board_with_RANSAC( cloud, ransac_inliers, 15/MEDIDA);
	cout<<"RANSAC INLIERS: "<<ransac_inliers->indices.size()<<endl;
}

void SpecificWorker::projectInliers()
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

void SpecificWorker::convexHull()
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

void SpecificWorker::extractPolygon()
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
	qDebug()<<"Comienza el for";
	int j = 0;
	#pragma omp parallel for
	for (unsigned int i = 0; i<cluster_indices.size();i++)
	// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ()+i;
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

			cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);


			// let's create a new image now
			cv::Mat crop(rgb_image.rows, rgb_image.cols, CV_8UC3);

			// set background to green
			crop.setTo(cv::Scalar(255,255,255));

			rgb_image.copyTo(crop, mask);

			normalize(mask.clone(), mask, 0.0, 255.0, CV_MINMAX, CV_8UC1);

			std::string scenename = "/home/robocomp/robocomp/components/prp/scene/" + std::to_string(num_scene) + "_scene.png";
			cv::imwrite( scenename, rgb_image );

			cv::imwrite( ss.str() + ".png", crop );

				/////save rgbd end
			writer.write<PointT> (ss.str () + ".pcd", *cloud_cluster, false);
#endif
			j++;
	}
	qDebug()<<"Clouds of points captured: "<< cluster_clouds.size();
	num_scene++;
}

void SpecificWorker::capturePointCloudObjects()
{
	// static boost::filesystem::directory_iterator it (boost::filesystem::path("/home/ivan/robocomp/components/prp/scene_prueba/brick"));
	// while(boost::filesystem::extension (it->path ()) != ".pcd")
	// 	it++;
	// std::cout<<it->path().string()<<std::endl;
	// boost::filesystem::path filename=*it;
	// std::stringstream outpath;
	// outpath << filename.branch_path().string() << "/" << boost::filesystem::basename(filename) <<".";
	// image_path = outpath.str()+"png";
	if(test)
		// readThePointCloud(outpath.str()+"png",outpath.str()+"pcd");
		readThePointCloud("/home/robocomp/robocomp/components/prp/scene/Scene.png","/home/robocomp/robocomp/components/prp/scene/Scene.pcd");
	else
		grabThePointCloud();
	cloud = Filter_in_axis(cloud, "y", -100/MEDIDA, 700/MEDIDA, true);
#ifdef USE_QTGUI
	viewer->updatePointCloud(cloud,"scene");
#endif
	struct timespec Inicio, Fin, resta;
	clock_gettime(CLOCK_REALTIME, &Inicio);
	cloud = VoxelGrid_filter(cloud, 3/MEDIDA, 3/MEDIDA, 3/MEDIDA);
	// writer.write<PointT> ("/home/robocomp/robocomp/components/perception/VoxelGrid_filter.pcd", *cloud, false);
	ransac();					//Calculo del plano de la mesa
	projectInliers();
	convexHull();
	extractPolygon();
	int n;
	euclideanClustering(n);
	clock_gettime(CLOCK_REALTIME, &Fin);
	SUB(&resta, &Fin, &Inicio);
	qDebug()<<"Captured: "<<resta.tv_sec<<"s "<<resta.tv_nsec<<"ns";
	// if (it != boost::filesystem::directory_iterator ())
	// 	++it;
}

void SpecificWorker::updateinner()
{
	try
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
	catch(...)
	{
		if (test)
		{
			innermodel->updateJointValue(QString::fromStdString("head_pitch_joint"),0.710744);
			innermodel->updateJointValue(QString::fromStdString("head_yaw_joint"),0.0102265);
		}
		else
			qFatal("Can't access motors");
	}
}

void SpecificWorker::loadTrainedDESCRIPTORS()
{
	descriptor_matcher->loadTrainingData();
	std::cout<<"Training data loaded"<<std::endl;
}

void SpecificWorker::descriptors(StringVector &guesses)
{
    int object__to_show = 0;
    descriptor_matcher->doTheGuess(cluster_clouds[object__to_show], descriptor_guesses);
	for(auto a:descriptor_guesses)
		guesses.push_back(a.file);
// 	guesses = descriptor_guesses;
}

bool SpecificWorker::aprilSeen(QVec &offset)
{
	QMutexLocker locker(&april_mutex);
	static InnerModel tabletags("/home/robocomp/robocomp/components/robocomp-shelly/files/tabletags.xml");
	QMat transformM,cameratoapril5M, cameratorobot, cameratoaprilseeM;
	std::vector<QVec> poses;
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
// 			offset=ret;
			poses.push_back(ret);
// 			return true;
		}
	}
	if(!poses.empty())
	{
		int count = 0;
		for (unsigned int i=0;i<poses.size();i++)
		{
			for (unsigned int j=0; j<poses.size(); j++)
			{
				if(30>(poses[i]-poses[j]).norm2())
					count++;
			}
			if(count>3)
			{
				offset = poses[i];
				return true;
			}
		}
	}
	return false;
}

void SpecificWorker::reloadDESCRIPTORS()
{
	if(descriptor_matcher->readFilesAndComputeDESCRIPTORS(pathLoadDescriptors,pathLoadDescriptors))
	{
		descriptor_matcher->reloadDESCRIPTORS(pathLoadDescriptors);
		descriptor_matcher->loadTrainingData();
	}
	qDebug()<<__LINE__;
	// string s="./bin/createDescriptors "+pathLoadDescriptors +" "+ descriptors_extension;
	// char *cstr = &s[0u];
	// if (system(cstr)==0)
	// {

	// }
}

void  SpecificWorker::getPose(ObjectType &Obj, string file_view_mathing, 	pcl::PointCloud<PointT>::Ptr obj_scene)
{
	static QMat april = RTMat(-M_PI_2, 0, 0,0,0,0);

	//Print centroide
	// Eigen::Vector4f centroid;
	// pcl::compute3DCentroid (*obj_scene, centroid);
	// QVec centroidpose = QVec::vec3(centroid[0]/MEDIDA,centroid[1]/MEDIDA,centroid[2]/MEDIDA);
	// centroidpose.print("centroid vista atual");
	// Point clouds
	string guessgan = file_view_mathing.substr(0, file_view_mathing.find_last_of("/"));
	guessgan = guessgan.substr(guessgan.find_last_of("/")+1);
	string pathxml = pathLoadDescriptors+"/"+guessgan+"/"+guessgan+".xml";
	pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
	//change vfh extension to pcd
	std::string view_to_load = file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	view_to_load = view_to_load + ".pcd";
	pcl::PointCloud<PointT>::Ptr object(obj_scene);
	if (pcl::io::loadPCDFile<PointT> (view_to_load, *scene) == -1) //* load the file
	{
		printf ("Couldn't read file test_pcd.pcd \n");
	}
	//convert pointcloud to mm
	scene  = PointCloudfrom_mm_to_Meters(scene);
	object = PointCloudfrom_mm_to_Meters(object);
#if DEBUG
	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/seen.pcd", *scene, false);
	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/saved.pcd", *object, false);
#endif
// 	-------------------------------------------------------------------

	pcl::PointCloud<PointT>::Ptr object_aligned(new pcl::PointCloud<PointT>);
	QMat saveToViewR;
	if(type_fitting == "RSCP")
		saveToViewR = fittingRSCP(object,scene,object_aligned);
	else if(type_fitting == "ICP")
		saveToViewR = fittingICP(object,scene,object_aligned);
#if DEBUG
 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/scene.pcd", *scene, false);
 	writer.write<PointT> ("/home/robocomp/robocomp/components/prp/objects/objectaling.pcd", *object_aligned, false);
#endif
	string node_name = file_view_mathing.substr(0, file_view_mathing.find_last_of("."));
	node_name        = node_name.substr(node_name.find_last_of("/")+1);
	InnerModel inner(pathxml);
	QMat PoseSavetoRootR = inner.getTransformationMatrix("root",QString::fromStdString(node_name));

// 	convert m to mm
	QVec saveToView = extraerposefromTM(saveToViewR);
	saveToView(0)=saveToView(0)*1000;
	saveToView(1)=saveToView(1)*1000;
	saveToView(2)=saveToView(2)*1000;
	saveToViewR= (RTMat(saveToView(3), saveToView(4), saveToView(5), saveToView(0), saveToView(1), saveToView(2))).invert();

	QMat poseObjR = saveToViewR*PoseSavetoRootR*april;
	QVec pose = extraerposefromTM(poseObjR);
	pose.print("pose");
	Obj.tx=pose.x();
	Obj.ty=pose.y() + offset_object;
	// poseObj.ty=890.;
	Obj.tz=pose.z();
	Obj.rx=pose.rx();
	Obj.ry=pose.ry();
	Obj.rz=pose.rz();
}

#ifdef USE_QTGUI
void SpecificWorker::saveView()
{

	string label=label_le->text().toStdString();
	//Open the object XML
	poses_inner = new InnerModel();
	string path=pathLoadDescriptors+"/"+label+"/";
	std::string inner_name = path+label + ".xml";
	poses_inner->open(inner_name);

	//Create the new node
	InnerModelNode *parent_node = poses_inner->getTransform("root");
	std::stringstream ss;
	ss <<"pose_"<<num_pose<<"_"<< label;
	InnerModelTransform *node = poses_inner->newTransform(ss.str().c_str(), "static", parent_node, tx, ty, tz, rx, ry, rz);
	parent_node->addChild(node);

	//Save the object cloud
	std::stringstream ss1;
	ss1 <<path<<"pose_"<<num_pose<<"_"<< label;
	std::cout <<ss1.str()<<endl;
	writer.write<PointT> (ss1.str () + ".pcd", *cluster_clouds[ob_to_save->value()], false);

	//Save the object image
	string imagename = path +"pose_" + QString::number(num_pose).toStdString() + "_" + label + ".png";
	cv::imwrite( imagename ,rgb_image);

	//Save the object XML
	num_pose++;
	poses_inner->save(QString(inner_name.c_str()));

	delete (poses_inner);
}

void SpecificWorker::initSaveObject(const string &label, const int numPoseToSave)
{
	capturePointCloudObjects();

	//Create the directory that contains the object info
	boost::filesystem::path path=boost::filesystem::path(pathLoadDescriptors+"/"+label+"/");
	if(!boost::filesystem::exists(path))
		boost::filesystem::create_directories(path);

	//Creates the XML of the object to save
	poses_inner = new InnerModel();
	num_pose = 0;
	std::string inner_name = path.string()+label + ".xml";
	poses_inner->save(QString(inner_name.c_str()));

	delete (poses_inner);
}

QVec SpecificWorker::saveRegPose(const string &label, const int numPoseToSave)
{
	capturePointCloudObjects();
	//check if appril seen
	QVec poseoffset = QVec::zeros(6);
	if(!aprilSeen(poseoffset))
	{
		qFatal("CAN'T SEE ANY APRIL!");
	}
	return poseoffset;
}

void SpecificWorker::updatergbd()
{
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	cv::Mat rgb_image(480,640, CV_8UC3, cv::Scalar::all(0));
	if(test)
	{
		rgb_image = cv::imread(image_path);

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
	}

	cv::Mat dest;
	cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
	QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	item_pixmap->setPixmap(QPixmap::fromImage(image));
	scene.update();
}

void SpecificWorker::settexttocloud(string name, float minx, float maxx, float miny, float maxy, float minz, float maxz)
{
	QGraphicsTextItem *text=new QGraphicsTextItem(QString::fromStdString(name));
	QFont serifFont("Times", 15, QFont::Bold);
	text->setFont(serifFont);
	InnerModelCamera *camera = innermodel->getCamera(id_camera);
	float m_x = minx + (maxx-minx)/2;
	float m_y = miny + (maxy-miny)/2;
	float m_z = minz + (maxz-minz)/2;
	QVec xy = camera->project(id_robot, QVec::vec3(m_x*MEDIDA, m_y*MEDIDA, m_z*MEDIDA));
	text->setPos(xy(0)-30,xy(1));
	scene.addItem(text);
	V_text_item.push_back(text);
}

void SpecificWorker::paintcloud(pcl::PointCloud< PointT >::Ptr cloud)
{
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
	}
	V_pixmap_item.push_back(new QGraphicsPixmapItem(QPixmap::fromImage(image)));
	scene.addItem(V_pixmap_item.back());
}

void SpecificWorker::removeAllpixmap()
{
	while(!V_pixmap_item.empty())
	{
		scene.removeItem(V_pixmap_item.back());
		V_pixmap_item.pop_back();
	}
}

void SpecificWorker::reloadDESCRIPTORS_Button()
{
	qDebug()<<__FUNCTION__;
	try
	{
		reloadDESCRIPTORS();
	}
	catch(...)
	{
		QMessageBox::warning(this, "something went wrong", "something went wrong");
	}
}

void SpecificWorker::findTheObject_Button()
{
	printf("%s %d\n", __FILE__, __LINE__);
	static vector<string> id_objects;
	while(!id_objects.empty())
	{
		viewer->removeCube(id_objects.back());
		// viewer->removePointCloud(id_objects.back());
		viewer->removeText(id_objects.back());
		viewer->removeCoordinateSystem(id_objects.back());
		id_objects.pop_back();
	}
	removeAllpixmap();
	qDebug()<<__FUNCTION__;
	std::string object = text_object->toPlainText().toStdString();
	ObjectVector lObjects;
	StringVector lNameObjects;
	if(object!="")
		lNameObjects.push_back(object);
	try
	{
		struct timespec Inicio_, Fin_, resta_;
		clock_gettime(CLOCK_REALTIME, &Inicio_);
		findObjects(lNameObjects, lObjects);
		clock_gettime(CLOCK_REALTIME, &Fin_);
		SUB(&resta_, &Fin_, &Inicio_);
		qDebug()<<"-----"<<resta_.tv_sec<<"s "<<resta_.tv_nsec<<"ns";
		int i=0;
		for(auto obj:lObjects)
		{
			std::cout<<"Object: "<<obj.label<<std::endl;
			std::cout<<"	Pose: "<<obj.tx<<", "<<obj.ty<<", "<<obj.tz<<", "<<obj.rx<<", "<<obj.ry<<", "<<obj.rz<<std::endl;
			std::cout<<"	  BB: "<<obj.minx<<", "<<obj.miny<<", "<<obj.minz<<", "<<obj.maxx<<", "<<obj.maxy<<", "<<obj.maxz<<std::endl;
			settexttocloud(obj.label, obj.minx, obj.maxx, obj.miny, obj.maxy, obj.minz, obj.maxz);
			viewer->addText3D(obj.label,obj.minx, obj.maxx, obj.miny, obj.maxy, obj.minz, obj.maxz, QString::number(i).toStdString());
			viewer->addCube(obj.minx, obj.maxx, obj.miny, obj.maxy, obj.minz, obj.maxz, QString::number(i).toStdString());
			if(obj.rx != 0 && obj.ry != 0 && obj.rz)
				viewer->addCoordinateSystem(RTMat(obj.rx,obj.ry,obj.rz,obj.tx/1000,obj.ty/1000,obj.tz/1000), QString::number(i).toStdString());
			id_objects.push_back(QString::number(i).toStdString());
			i++;
		}
	}
	catch(...)
	{
		QMessageBox::warning(this, "something went wrong", "something went wrong");
	}
}

void SpecificWorker::fullRun_Button()
{
	string label=label_le->text().toStdString();
	char *c;
	string s="mkdir " + pathLoadDescriptors + "/" + label;
	c= &s[0u];
	try
	{
		system(c);
		if(InitPose->isChecked())
			initSaveObject(label,ob_to_save->value());
		if(regularPose->isChecked())
		{
			guess = saveRegPose(label,ob_to_save->value());
			if(guess !=QVec::zeros(6))
				isObject->setText("Objeto Guardado.");
			else
				isObject->setText("Objeto NO Guardado.");
			x_object->setText(QString::number(guess.x()));
			y_object->setText(QString::number(guess.y()));
			z_object->setText(QString::number(guess.z()));
			rx_object->setText(QString::number(guess.rx()));
			ry_object->setText(QString::number(guess.ry()));
			rz_object->setText(QString::number(guess.rz()));
			viewer->removeCoordinateSystem("poseobject");
			QMat poseObjR=RTMat(guess.rx(),guess.ry(),guess.rz(),guess.x()/1000.,guess.y()/1000.,guess.z()/1000.);
			viewer->addCoordinateSystem(poseObjR,"poseobject");
		}
	}
	catch(...)
	{
		QMessageBox::warning(this, "something went wrong", "something went wrong");
	}
}

void SpecificWorker::ResetPose_Button()
{
	viewer->removeCoordinateSystem("poseobject");
	guess = QVec::zeros(6);
}
#endif
