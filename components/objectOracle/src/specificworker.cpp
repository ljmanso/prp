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

static unsigned int get_current_time(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx),
first(true)
{
	inner_mutex = new QMutex(QMutex::Recursive);
	world_mutex = new QMutex(QMutex::Recursive);
	agent_mutex = new QMutex(QMutex::Recursive);
	
	save_full_data = save_table_data = labeling = false;
	modifiedWorld = -1;
	image_segmented_counter = 0;
	image_save_counter = 0;
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";
	innerModel = new InnerModel();
	worldModelTime = actionTime = QTime::currentTime();
	
	connect(saveButton, SIGNAL(clicked()), this, SLOT(save_tables_info()));

#ifdef CONVNET
	file.open("/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012.words", std::ifstream::in);
	convnet = ccv_convnet_read(0, "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/ccv/image-net-2012-vgg-d.sqlite3");
#endif

	oracleImage.resize(IMAGE_WIDTH*IMAGE_HEIGHT);
	rgbImage.resize(IMAGE_WIDTH*IMAGE_HEIGHT);
	points.resize(IMAGE_WIDTH*IMAGE_HEIGHT);
	
	cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	fullCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
	
	std::string model_file   = "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/caffe/deploy.prototxt";
	std::string trained_file = "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/caffe/bvlc_reference_caffenet.caffemodel";
	std::string mean_file    = "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/caffe/imagenet_mean.binaryproto";
	std::string label_file   = "/home/robocomp/robocomp/components/prp/experimentFiles/dpModels/caffe/synset_words.txt";
	
	caffe_classifier = new CaffeClassifier(model_file, trained_file, mean_file, label_file);
        
	labeler = std::make_shared<Labeler>(model_file, trained_file, mean_file, label_file);
        
        
	#ifdef INNER_VIEWER
		//AGM Model Viewer
		osgView = new OsgView();
		manipulator = new osgGA::TrackballManipulator;
		osgView->setCameraManipulator(manipulator, true);
		osgView->show();
 	#endif
	
 	load_tables_info();
        

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
	
	std::cout<<"The cup is at: "<<lookForObject("cup")<<std::endl;

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

void SpecificWorker::loadTablesFromModel()
{
    auto symbols = worldModel->getSymbolsMap(params, "table");
    
    for (const auto& elem : symbols)
    {
        
        std::map<std::string, double> table;
        
        //lets save the table map and its id
        
        tables.push_back( std::pair <std::map<std::string, double>, int> (table, elem.second->identifier) );
    }
        
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	QMutexLocker locker(agent_mutex);
	
	if(params["labeling"].value == "on")
	{
		printf("Labeling is ON\n");
		labeling =true;
	}
	else
		printf("Labeling is OFF\n");
	
	if(params["save_full_data"].value == "on")
	{
		printf("Saving Full Data is ON\n");
		save_full_data = true; 
	}
	else
		printf("Saving Full Data is OFF\n");
	
	
	if(params["save_table_data"].value == "on")
	{
		printf("Saving Table Data is ON\n");
		save_table_data = true;
	}
	else
		printf("Saving Table Data is OFF\n");
	
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
	static bool first=true;
	printf("ACTION: %s\n", action.c_str());
	
	boost::algorithm::to_lower(action);
	
	if (action == "imaginemostlikelymuginposition")
	{
		action_imagineMostLikelyMugInPosition();
	}

	if (first)
	{
		qLog::getInstance()->setProxy("both", logger_proxy);
		rDebug2(("oracleAgent started\n"));
		first=false;
		load_tables_info();
  		//processDataFromDir("/home/marcog/robocomp/components/prp/experimentFiles/capturas/");
		//show map after processing
		
/*        for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
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
        }*/
        //save_tables_info();
	}
	//read image from kinect
	try
	{
		qDebug() << "read frame";
		rgbd_proxy->getRGB(rgbImage, hState, bState);
		rgbd_proxy->getXYZ(points, hState, bState);
		//Convert image to RoboCompObjectOracle
//		memcpy(&oracleImage[0], &rgbImage[0], IMAGE_WIDTH*IMAGE_HEIGHT*3);

		std::string location = checkTable(rgbImage);
		//std::string location = "table1";
		//if robot is close to any table

		if (location != "invalid" )
		{
			printf("**************************************\nLOCATION %s\n********************************\n",location.c_str());
			
			std::string file_name = std::to_string(image_save_counter) + "_" + location;
			int pos = 0, k=0;
			
			if(save_table_data || labeling)
			{
				//crop image 
				matImage = cv::Mat(up-down,right-left,CV_8UC3, cv::Scalar::all(0));
				cloud->clear();
				cloud->resize((up-down)*(right-left));
				
				for(int j=down; j<up; j++)
				{
					for(int i=left; i<right; i++)
					{
						pos = j * IMAGE_WIDTH + i;
						matImage.at<cv::Vec3b>(j-down, i-left) = cv::Vec3b(rgbImage[pos].blue, rgbImage[pos].green, rgbImage[pos].red);
						QVec p1 = QVec::vec4(points[pos].x, points[pos].y, points[pos].z, 1);
						memcpy(&cloud->points[k],p1.data(),3*sizeof(float));
						cloud->points[k].r = rgbImage[pos].red;
						cloud->points[k].g = rgbImage[pos].green;
						cloud->points[k].b = rgbImage[pos].blue;
						k++;
					}
				}
					
	
			//	cv::imwrite( file_name + ".jpg", matImage );
			//	pcl::io::savePCDFileASCII (file_name + ".pcd", *cloud);
			//	cv::imshow("3D viewer",matImage);
			}
			
			if(save_full_data)
			{
				fullImage = cv::Mat(480,640,CV_8UC3, cv::Scalar::all(0));
				pos = 0;
				k=0;
				fullCloud->clear();
				fullCloud->resize(480*640);
				for(int j=0; j<480; j++)
				{
					for(int i=0; i<640; i++)
					{
						pos = j * IMAGE_WIDTH + i;
						fullImage.at<cv::Vec3b>(j, i) = cv::Vec3b(rgbImage[pos].blue, rgbImage[pos].green, rgbImage[pos].red);
						QVec p1 = QVec::vec4(points[pos].x, points[pos].y, points[pos].z, 1);
						memcpy(&fullCloud->points[k],p1.data(),3*sizeof(float));
						fullCloud->points[k].r = rgbImage[pos].red;
						fullCloud->points[k].g = rgbImage[pos].green;
						fullCloud->points[k].b = rgbImage[pos].blue;
						k++;
					}
				}
				cv::imwrite( file_name + "_full.jpg", fullImage );
				pcl::io::savePCDFileASCII (file_name + "_full.pcd", *fullCloud);
			}
			
			++image_save_counter;
		
//			unsigned int elapsed_time = get_current_time();
			//processDataFromKinect(matImage, points, location);
			if(labeling)
			{
				labelImage(matImage, location);
				cv::imshow("Labeled table",matImage);
			}
					

//			elapsed_time = get_current_time() - elapsed_time;
//			printf("elapsed time %d ms\n",elapsed_time);
			
		}

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
	#ifdef INNER_VIEWER
		//AGM Model viewer
		updateViewer();
	#endif
}
std::string SpecificWorker::checkTable(RoboCompRGBD::ColorSeq image)
{
	world_mutex->lock();
	AGMModel::SPtr copyModel(new AGMModel(worldModel));
	world_mutex->unlock();

	std::string table = "invalid";
	for (AGMModel::iterator symbol_it=copyModel->begin(); symbol_it!=copyModel->end(); symbol_it++)
	{
		const AGMModelSymbol::SPtr &symbol = *symbol_it;
		if (symbol->symbolType == "object")
		{
			for (auto edge = symbol->edgesBegin(copyModel); edge != symbol->edgesEnd(copyModel); edge++)
			{
				QString edgeLabel = QString::fromStdString(edge->getLabel());
				const std::pair<int32_t, int32_t> symbolPair = edge->getSymbolPair();
				if (copyModel->getSymbol(symbolPair.second)->symbolType == "objectSt" and edgeLabel == "table")
				{
					try
					{
						std::string tableIMName = symbol->getAttribute("imName");
						const float tableWidth  = str2float(symbol->getAttribute("width"));
						const float tableHeight = str2float(symbol->getAttribute("height"));
						const float tableDepth  = str2float(symbol->getAttribute("depth"));
						printf("Checking table %s\n", tableIMName.c_str());
						if (isTableVisible(image, tableIMName, tableWidth, tableHeight, tableDepth))
						{
							return tableIMName;
						}
					}
					catch(...)
					{
						printf("EXCEPTION: Imposible to read properties from symbol %s\n", symbol->toString().c_str());
					}
				}
			}
		}
	}
	return table;
}

bool SpecificWorker::isTableVisible(RoboCompRGBD::ColorSeq image, const std::string tableIMName, const float tableWidth, const float tableHeight, const float tableDepth)
{
	inner_mutex->lock();
	float distance = innerModel->transform("rgbd", tableIMName.c_str()).norm2();
	inner_mutex->unlock();
//	std::cout<<"Distance to TABLE "<<tableIMName<<":"<<distance<<std::endl;
	
	if( distance < TABLE_DISTANCE )
	{
//		std::cout<<"table"<<tableIMName<<"dimensions"<< tableWidth << tableHeight << tableDepth<<std::endl;
		
//		innerModel->transform6D("rgbd", QVec::vec6(0,0,0,0,0,0), tableIMName.c_str()).print("relative table pose rbd");
//		innerModel->transform6D("robot", QVec::vec6(0,0,0,0,0,0), tableIMName.c_str()).print("relative table pose robot");
		inner_mutex->lock();
		QVec a1 = innerModel->transform("rgbd", QVec::vec3(-tableWidth/2, 0, -tableDepth/2), tableIMName.c_str());
		QVec b1 = innerModel->transform("rgbd", QVec::vec3(-tableWidth/2, 0, +tableDepth/2), tableIMName.c_str());
		QVec c1 = innerModel->transform("rgbd", QVec::vec3(+tableWidth/2, 0, -tableDepth/2), tableIMName.c_str());
		QVec d1 = innerModel->transform("rgbd", QVec::vec3(+tableWidth/2, 0, +tableDepth/2), tableIMName.c_str());

/*		a1.print("leftdown 1");
		b1.print("leftup 1");
		c1.print("rightdown 1");
		d1.print("rightup 1");
*/		
		QVec a2 = camera->project("rgbd", a1);
		QVec b2 = camera->project("rgbd", b1);
		QVec c2 = camera->project("rgbd", c1);
		QVec d2 = camera->project("rgbd", d1);
		inner_mutex->unlock();
		
		QList<QVec> points_on_screen;
	
		if( a2(0) > 0 && a2(0) < IMAGE_WIDTH && a2(1) > 0 && a2(1) < IMAGE_HEIGHT )
		{
//			printf("leftdown seen\n");
			points_on_screen.append(a2);
		}
		if( b2(0) > 0 && b2(0) < IMAGE_WIDTH && b2(1) > 0 && b2(1) < IMAGE_HEIGHT )
		{
//			printf("leftup seen\n");
			points_on_screen.append(b2);
		}
		if( c2(0) > 0 && c2(0) < IMAGE_WIDTH && c2(1) > 0 && c2(1) < IMAGE_HEIGHT )
		{
//			printf("righdown seen\n");
			points_on_screen.append(c2);
		}
		if( d2(0) > 0 && d2(0) < IMAGE_WIDTH && d2(1) > 0 && d2(1) < IMAGE_HEIGHT )
		{
//			printf("rightup seen\n");
			points_on_screen.append(d2);
		}

		// show some info
		a2.print("leftdown 2");
		b2.print("leftup 2");
		c2.print("rightdown 2");
		d2.print("rightup 2");
		

                //draw points on screen
                matImage = cv::Mat(480,640,CV_8UC3, cv::Scalar::all(0));
                    

                for(unsigned int i=0; i<image.size(); i++)
                {
                        int row = i/640;
                        int column = i-(row*640);
                        
                        matImage.at<cv::Vec3b>(row,column) = cv::Vec3b(image[i].blue, image[i].green, image[i].red);
                }
                    
                for (QVec v : points_on_screen)
                {
                     cv::circle(matImage, cv::Point((int)v(0),(int)v(1)), 5, (0,0,255), -1);
                }   

                cv::imshow("Table Detection" ,matImage);


		std::cout<<"Table: "<<tableIMName<<" ==> Num corners on screen: "<< points_on_screen.size() <<std::endl;
		if (points_on_screen.size() > 2)
		{
			left = IMAGE_WIDTH;
			down = IMAGE_HEIGHT;
			right = 0;
			up = 0;
			for (QList<QVec>::iterator  iterator=points_on_screen.begin();iterator !=points_on_screen.end();++iterator)
			{
				QVec point = (*iterator);
				if (point(0) > 0 && point(0) < left)
					left = point(0);
				if (point(0) <= IMAGE_WIDTH && point(0) > right)
					right = point(0);
				if (point(1) > 0 && point(1) < down)
					down = point(1);
				if (point(1) <= IMAGE_HEIGHT && point(1) > up)
					up = point(1);
			}
			
			
			left -= OFFSET;
			right += OFFSET;
			down -= OFFSET_TOP;
			up += OFFSET;
			
			if (left < 0)
				left = 0;
			if (right > IMAGE_WIDTH)
				right = IMAGE_WIDTH;
			if (down < 0)
				down = 0;
			if (up > IMAGE_HEIGHT)
				up = IMAGE_HEIGHT;
		
			return true;
		}
	}
	return false;
}

bool SpecificWorker::reloadConfigAgent()
{
	return true;
}


bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	QMutexLocker locker(agent_mutex);
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
	QMutexLocker locker(agent_mutex);
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
	QMutexLocker locker(agent_mutex);
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
    
	if (ofs) 
	{	
		std::cout<<"Loading previously labeled tables...."<<std::endl;
		boost::archive::text_iarchive oa(ofs);
		oa >> table1;
		oa >> table2;
		oa >> table3;
		oa >> table4;
		oa >> table5;
		
		//display results
		showTablesOnInterface();
		
	}
	else
		std::cout<<"No tables initialization found, proceding with empty tables.";

}

void SpecificWorker::showTablesOnInterface()
{
	
	table1_qmat = QMap<std::string, double>(table1);
	mapmodel_1.setMap(&table1_qmat);
	filtermodel_1.setSourceModel( &mapmodel_1 );
	tableView_1->setModel(&filtermodel_1);
	tableView_1->setSortingEnabled(true);
	
	table2_qmat = QMap<std::string, double>(table2);
	mapmodel_2.setMap(&table2_qmat);
	filtermodel_2.setSourceModel( &mapmodel_2 );
	tableView_2->setModel(&filtermodel_2);
	tableView_2->setSortingEnabled(true);
	
	table3_qmat = QMap<std::string, double>(table3);
	mapmodel_3.setMap(&table3_qmat);
	filtermodel_3.setSourceModel( &mapmodel_3 );
	tableView_3->setModel(&filtermodel_3);
	tableView_3->setSortingEnabled(true);
	
	table4_qmat = QMap<std::string, double>(table4);
	mapmodel_4.setMap(&table4_qmat);
	filtermodel_4.setSourceModel( &mapmodel_4 );
	tableView_4->setModel(&filtermodel_4);
	tableView_4->setSortingEnabled(true);
	
	table5_qmat = QMap<std::string, double>(table5);
	mapmodel_5.setMap(&table5_qmat );
	filtermodel_5.setSourceModel( &mapmodel_5 );
	tableView_5->setModel(&filtermodel_5);
	tableView_5->setSortingEnabled(true);
	
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

RoboCompRGBD::ColorSeq SpecificWorker::convertMat2ColorSeq(cv::Mat rgb)
{
	RoboCompRGBD::ColorSeq rgbMatrix;
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

void SpecificWorker::processDataFromKinect(cv::Mat matImage, const RoboCompRGBD::PointSeq &points_kinect, std::string location)
{
	//process whole image
	processImage(matImage, location);
	std::vector<cv::Mat> segmented_objects;
	
	//segment image and process again
	segmentObjects3D(cloud, matImage, segmented_objects);
	
	cout<<"El vector tiene>>>>> "<<segmented_objects.size()<<endl;
	
	for(uint i = 0 ; i < segmented_objects.size() ; i++)
	{	
//		RoboCompRGBD::ColorSeq auxMatrix = convertMat2ColorSeq (segmented_objects[i]);
		processImage(segmented_objects[i], location);
	}
	
}

void SpecificWorker::labelImage(cv::Mat matImage, std::string location)
{
	/// Global variables
	cv::Mat blt,tophat,gray,bw;
	ResultList result;
	
	int morph_elem = 0;
	int morph_size = 21;

	int const max_elem = 2;
	
	if( matImage.empty() )
	{ 
		std::cout<<"Image to label not valid!"<<std::endl;
		return -1; 
	}

	/// Bilateral Filtering: edge preserving color and spatial filter.

	bilateralFilter(matImage, blt, 8, 40, 5, 1 );

	int operation = 4 + 2;

	cv::Mat element = cv::getStructuringElement( morph_elem, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );

	/// Apply the specified morphology operation (tophat).
	morphologyEx( blt, tophat, operation, element);
	cvtColor(tophat, gray, CV_BGR2GRAY);
	///Thresholding to create binay image.
	int threshval=30;
	bw = threshval < 128 ? (gray > threshval) : (gray < threshval);
	cv::Mat labeledImage(gray.size(), CV_32S);
	///Labeling each segment.
	int nLabels = connectedComponents(bw, labeledImage, 8);
	std::vector<cv::Rect> rects(nLabels);
	///Fitting bounding-box to segments.
    
	for(int label = 0; label < nLabels; ++label)
	{
			//Note:storing in form (rmin,cmin,rmax,cmax) and not (xmin,ymin,width,height)
		rects[label]=cv::Rect(gray.rows,gray.cols, 0,0 ); 
	}
	
	cv::Mat dst(gray.size(), CV_8UC3);
	for(int r = 0; r < dst.rows; ++r)
	{
		for(int c = 0; c < dst.cols; ++c)
		{
		int label = labeledImage.at<int>(r, c);
		if(c<rects[label].x)rects[label].x=c;
		if(r<rects[label].y)rects[label].y=r;
			if(c>rects[label].width)rects[label].width=c;
		if(r>rects[label].height)rects[label].height=r;
		}
	}
	
	dst=matImage;
	cv::Scalar rect_color = cv::Scalar( 0, 0, 255 );
	///Extract each bounding-box, draw rectangle and classify using CNN
	for(int label = 0; label < nLabels; ++label)
	{
		//Note:converting back to (xmin,ymin,width,height) format.
		rects[label].width=rects[label].width-rects[label].x;
		rects[label].height=rects[label].height-rects[label].y;
		if(rects[label].width*rects[label].height<200)
			continue;            	 
		
		rectangle( dst, rects[label].tl(), rects[label].br(), rect_color, 2, 8, 0 );
		cv::Rect r1=rects[label];
		cv::Rect r2;
		int bord=20; 
		r2.x=r1.x-bord > 0 ? r1.x-bord : r1.x;
		r2.y=r1.y-bord > 0 ? r1.y-bord : r1.y;
		r2.width= r1.x+r1.width+bord < matImage.cols ? r1.width+bord:r1.width;
		r2.height=r1.y+r1.height+bord < matImage.rows ? r1.height+bord:r1.height;
		cv::Mat roi = matImage( r2 );
		float top_score=-1;
		std::string top_label="";

		std::vector<Prediction> predictions = labeler->Classify(roi);
			Prediction p = predictions[0];
		top_score=p.second;
		top_label=p.first;
		putText(dst, top_label.substr(0, top_label.find(",")), rects[label].tl(), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
// 		cout<<"Window ID:"<<label<<", detected category:"<<top_label<<", score:"<<top_score<<endl;
		
		//add label to results
		Label l;
		l.name = p.first;
		l.believe = p.second;
		result.push_back(l);
	}
	
	matImage = dst;
	
	//add labels to table
	addLabelsToTable(result, location);
	
	//display results
	showTablesOnInterface();
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
				
				//RoboCompRGBD::ColorSeq rgbMatrix = convertMat2ColorSeq (rgb);
				
				std::cout<<"Processing image: "<<path2file<<" from table: "<<location<<endl;
				std::cout<<rgb.size()<<std::endl;
				processImage(rgb, location);
				
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
//					rgbMatrix = convertMat2ColorSeq (segmented_objects[i]);
					processImage(segmented_objects[i], location);
				}
			}
			
		}
	}                        
}


void SpecificWorker::processImage(cv::Mat matImage, std::string location)
{
    ResultList result;
    
    getLabelsFromImageWithCaffe(matImage, result);
    addLabelsToTable(result, location);
    printf("proccesImage end\n");       
    
}

void SpecificWorker::addLabelsToTable(ResultList result, std::string location)
{
    std::string label;
    
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
 					std::cout<<" ------------ Max: "<<table1[label]<<" "<<(double)result[i].believe;
                    table1[label] = std::max(table1[label], (double)result[i].believe);
 					std::cout<<"Result: "<<table1[label]<<std::endl;
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

#ifdef CONVNET
void SpecificWorker::getLabelsFromImage(const RoboCompRGBD::ColorSeq &image, ResultList &result)
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
#else
void SpecificWorker::getLabelsFromImage(const RoboCompRGBD::ColorSeq &image, ResultList &result)
{
			Label l;
            l.name = "CCV_CONNET NOT ENABLED AT COMPILE TIME";
            l.believe = 1;
            result.push_back(l);
}
#endif



void SpecificWorker::getLabelsFromImageWithCaffe(cv::Mat matImage, ResultList &result)
{
	std::vector<Prediction> predictions = caffe_classifier->Classify(matImage);
	
	  /* Print the top N predictions. */
	for (size_t i = 0; i < predictions.size(); ++i) 
	{
		Prediction p = predictions[i];
		std::cout << std::fixed << std::setprecision(4) << p.second << " - \""<< p.first << "\"" << std::endl;
		
		Label l;
		l.name = p.first;
		l.believe = p.second;
		result.push_back(l);
	}
}


void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::World &modification)
{
	QMutexLocker locker(world_mutex);
	QMutexLocker lockerIN(inner_mutex);
 	AGMModelConverter::fromIceToInternal(modification, worldModel);
	worldModelTime = QTime::currentTime();

	delete innerModel;
	innerModel = AGMInner::extractInnerModel(worldModel);
	camera = innerModel->getCamera("rgbd");
	#ifdef INNER_VIEWER
		changeInner();
	#endif
}

void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker locker(world_mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
		AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
	}
	
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	QMutexLocker locker(world_mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	AGMInner::updateImNodeFromEdge(worldModel, modification, innerModel);
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	QMutexLocker locker(world_mutex);
	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
}

void SpecificWorker::symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications)
{
	QMutexLocker locker(world_mutex);
	for (auto modification : modifications)	
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
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
//		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//		std::stringstream ss;
//		ss <<"capture_"<<image_segmented_counter<< "_object_" << j;

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
		
// 		cv::imwrite( ss.str() + ".png", cropped );
// 		
// 		pcl::io::savePCDFileASCII (ss.str () + ".pcd", *cloud_cluster);
 		j++;
		result.push_back(cropped);
	}
	
// 	return result;
}

std::string SpecificWorker::lookForObjectNoW2V(std::string label)
{
    std::string table = "none";
    double current_believe = -1;
    
    //Obtaining object posibility from the different tables
    std::map<std::string,double>::iterator it1 = table1.find(label);
    if (it1 != table1.end() && it1->second > current_believe)
    {
        table =  "table1";
        current_believe = it1->second;
		
    }
    
    std::map<std::string,double>::iterator it2 = table2.find(label);
    if (it2 != table2.end() && it2->second > current_believe)
    {
        table =  "table2";
        current_believe = it2->second;
    }
    
    std::map<std::string,double>::iterator it3 = table3.find(label);
    if (it3 != table3.end() && it3->second > current_believe)
    {
        table =  "table3";    
        current_believe = it3->second;
    }
    
    std::map<std::string,double>::iterator it4 = table4.find(label);
    if (it4 != table4.end() && it4->second > current_believe)
    {
        table =  "table4";       
        current_believe = it4->second;
    }
    
    std::map<std::string,double>::iterator it5 = table5.find(label);
	if (it5 != table5.end() && it5->second > current_believe)
    {
        table =  "table5";       
        current_believe = it5->second;
    }
   
	//delete result from the table
	if( table == "table1" )
		table1.erase(it1);
	if( table == "table2" )
		table2.erase(it2);
	if( table == "table3" )
		table3.erase(it3);
	if( table == "table3" )
		table4.erase(it3);
	if( table == "table5" )
		table5.erase(it5);	
    
    return table;
}

std::string SpecificWorker::lookForObject(std::string label)
{
	float higher_similarity, calculated_similarity;
	std::string current_table;
	
	std::vector<float> label_representation;

	semanticsimilarity_proxy->getWordRepresentation(label, label_representation);
	
	//intialize to the first table values
	semanticsimilarity_proxy->w2vVectorsDistance(table1_w2v, label_representation, higher_similarity);
	current_table = "table1";
	
	semanticsimilarity_proxy->w2vVectorsDistance(table2_w2v, label_representation, calculated_similarity);
	if ( calculated_similarity > higher_similarity )
	{
		higher_similarity = calculated_similarity;
		current_table = "table2";
	}
	
	semanticsimilarity_proxy->w2vVectorsDistance(table3_w2v, label_representation, calculated_similarity);
	if ( calculated_similarity > higher_similarity )
	{
		higher_similarity = calculated_similarity;
		current_table = "table3";
	}
	
	semanticsimilarity_proxy->w2vVectorsDistance(table4_w2v, label_representation, calculated_similarity);
	if ( calculated_similarity > higher_similarity )
	{
		higher_similarity = calculated_similarity;
		current_table = "table4";
	}
	
	semanticsimilarity_proxy->w2vVectorsDistance(table5_w2v, label_representation, calculated_similarity);
	if ( calculated_similarity > higher_similarity )
	{
		higher_similarity = calculated_similarity;
		current_table = "table5";
	}
	
	return current_table;
	
}

bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	QMutexLocker locker(agent_mutex);
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		printf("param:%s   value:%s\n", it->first.c_str(), it->second.value.c_str());
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
	world_mutex->lock();
	printf("inside imagineMostLikelyOBJECTPosition-%s  %d %d\n", objectType.c_str(), modifiedWorld, worldModel->version);
	if (modifiedWorld > worldModel->version or actionTime < worldModelTime){
		world_mutex->unlock();
		return;
	}
	AGMModel::SPtr newModel(new AGMModel(worldModel));
	world_mutex->unlock();

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
		id = 38;
		room_id = 5;
	}
	if( table == "table2" )
	{
		id = 35;
		room_id = 5;
	}
	if( table == "table3" )
	{
		id = 26;
		room_id = 3;
	}
	if( table == "table4" )
	{
		id = 23;
		room_id = 3;
	}
	if( table == "table5" )
	{
		id = 20;
		room_id = 3;
	}

	if (id != -1 and room_id != -1)
	{
		// Create the edges that indicate in which table the object will be located
		AGMModelSymbol::SPtr tableID = newModel->getSymbol(id);
		AGMModelSymbol::SPtr roomID  = newModel->getSymbol(room_id);
		newModel->addEdge(objS, tableID, "in");
		newModel->addEdge(tableID, objS, "RT");
		newModel->addEdge(objS, roomID, "in");

		// Send modification proposal
// 		modifiedWorld = worldModel->version + 1;
		world_mutex->lock();
		sendModificationProposal(worldModel, newModel);
		world_mutex->unlock();
	}
	else
	{
		printf("Object: %s not found", objectType.c_str()); 
	}
}



void SpecificWorker::action_imagineMostLikelyMugInPosition()
{
	imagineMostLikelyOBJECTPosition("mug");
}


void SpecificWorker::action_imagineMostLikelyCoffeePotInPosition()
{
	imagineMostLikelyOBJECTPosition("coffeepot");
}


void SpecificWorker::action_imagineMostLikelyMilkInPosition()
{
	imagineMostLikelyOBJECTPosition("milk");
}



void SpecificWorker::semanticDistance(const string &word1, const string &word2, float &result)
{

}
#ifdef INNER_VIEWER
//AGM Model Viewer
void SpecificWorker::updateViewer()
{
	QTime cc;
	cc = QTime::currentTime();
	inner_mutex->lock();
	if (not innerModel) return;
	if (not innerModel->getNode("root")) return;
//	printf("root %p\n", innerModel->getNode("root"));

	if (not innerViewer)
	{
		innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
//		printf("innerViewer: %p\n", innerViewer);
		innerViewer->setMainCamera(manipulator, InnerModelViewer::TOP_POV);
	}
//	printf("innerViewer: %p\n", innerViewer);
	innerViewer->update();
	inner_mutex->unlock();
	osgView->autoResize();
	osgView->frame();
//	printf("updateViewer - %d\n", cc.elapsed());
}

void SpecificWorker::changeInner ()
{
	if (innerViewer)
	{
		osgView->getRootGroup()->removeChild(innerViewer);				
	}
	inner_mutex->lock();
	innerViewer = new InnerModelViewer(innerModel, "root", osgView->getRootGroup(), true);
	inner_mutex->unlock();
}
#endif 
