/*
 *    Copyright (C) 2016 by YOUR NAME HERE
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

using namespace cv;

///global variables 
Mat src;
RoboCompRGBD::ColorSeq rgbMatrix;
bool windowmutex;


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
  //callback declaration when button pressed.
  
  connect(pushButtonLeft, SIGNAL(clicked()), this, SLOT(moveleft()));
  connect(pushButtonRight, SIGNAL(clicked()), this, SLOT(moveright()));
  connect(pushButtonFront, SIGNAL(clicked()), this, SLOT(movefront()));
  connect(pushButtonBack, SIGNAL(clicked()), this, SLOT(moveback()));
  connect(pushButtonClock, SIGNAL(clicked()), this, SLOT(rotateclock()));
  connect(pushButtonAnticlock, SIGNAL(clicked()), this, SLOT(rotateanticlock()));
  connect(pushButtondetect, SIGNAL(clicked()), this, SLOT(detectobject()));
  windowmutex=false;
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
     namedWindow( "Display window", 1 );
     RoboCompDifferentialRobot::TBaseState bState;
     RoboCompJointMotor::MotorStateMap hState;
     
 	
 	try
 	{
 	sleep(0.2);
 	///grab RGB image from simulator camera
 	rgbd_proxy->getRGB(rgbMatrix,hState,bState);    
    
    int rows=480;
    int cols=640;
    
    ///opencv image initialize
    src.create(rows,cols,CV_8UC3);
	
	/// deep conversion to opencv Mat
	/// to do: shallow conversion
	  
	int k=0;
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)	
			{
				Vec3b intensity;
				intensity.val[0]=rgbMatrix[k].blue;
				intensity.val[1]=rgbMatrix[k].green;
				intensity.val[2]=rgbMatrix[k].red;
                src.at<Vec3b>(i,j)=intensity;
                k++;
             }
    }    
    
    ///share same window for camera output and detection output.
    ///show camera output till detection button clicked,
    ///After that show detection output till next button click.
    
    if(!windowmutex)
      {    
        imshow("Display window",src);
        waitKey(100);
	  }
 	
 	}
 	catch(const Ice::Exception &e)
 	{
 		std::cout << "Error reading from Camera" << e << std::endl;
 	}
}



void SpecificWorker::moveleft()
{
	///move robot to left and stop		
	
	windowmutex=false;
	cout<<"move left"<<endl;
	
	try
	{
		omnirobot_proxy->setSpeedBase(-100,00,0);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::moveright()
{
	windowmutex=false;
	cout<<"move right"<<endl;
	try
	{
		omnirobot_proxy->setSpeedBase(100,00,0);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::movefront()
{
	windowmutex=false;
	cout<<"move front"<<endl;
	try
	{
		omnirobot_proxy->setSpeedBase(0,100,0);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::moveback()
{
	windowmutex=false;
	cout<<"move back"<<endl;
	try
	{
		omnirobot_proxy->setSpeedBase(0,-100,0);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::rotateclock()
{
	windowmutex=false;
	cout<<"rotate clockwise"<<endl;
	try
	{
		omnirobot_proxy->setSpeedBase(0,0,0.2);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::rotateanticlock()
{
	
	cout<<"rotate anti-clockwise"<<endl;
	windowmutex=false;
	try
	{
		omnirobot_proxy->setSpeedBase(0,00,-0.2);
		sleep(1);	
	    omnirobot_proxy->setSpeedBase(0,0,0);
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}

void SpecificWorker::detectobject()
{
	
	cout<<"Detect Object"<<endl;
	try
	{
	
	/// convert RoboCompRGBD::ColorSeq to RoboCompobjectDetectionCNN::ColorSeq.
	///to do: use shallow copy instead of deep copy.
	
	RoboCompobjectDetectionCNN::ColorSeq rgbMatrix1; 	 
    rgbMatrix1.resize(src.rows*src.cols);
    
    int k=0;
	for(int i=0;i<src.rows;i++)
	{
		for(int j=0;j<src.cols;j++)	
			{
				rgbMatrix1[k].blue=rgbMatrix[k].blue;
				rgbMatrix1[k].green=rgbMatrix[k].green;
				rgbMatrix1[k].red=rgbMatrix[k].red;
                k++;
             }
    }    
	
	
	ResultList result;
	Mat dst = src.clone();
	putText(dst, "Computing object detections.", Point(20,20), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	putText(dst, "Will take a while!!!", Point(20,40), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	putText(dst, "kindly be patient.", Point(20,60), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	windowmutex=true;
	imshow("Display window",dst);
	waitKey(200);	
    
    ///calling CNN Object detection component    
    objectdetectioncnn_proxy->getLabelsFromImage(rgbMatrix1, src.rows, src.cols, result);
    dst = src.clone();
    ///Display detection results.
    
    for(unsigned int i=0; i < result.size();i++ )
            {
                //cout<<result[i].name<<" "<<result[i].believe<<endl;
                //cout<<result[i].bb.x<<" "<<result[i].bb.y<<" "<<result[i].bb.width<<" "<<result[i].bb.height<<endl;
                Rect r1;
                BoundingBox bb;
                bb=result[i].bb;
                r1.x=bb.x;
				r1.y=bb.y;
				r1.width=bb.width;
				r1.height=bb.height;
				Scalar rect_color = Scalar( 0, 0, 255 );
                rectangle( dst, r1.tl(), r1.br(), rect_color, 2, 8, 0 );
                string name=result[i].name;
                putText(dst, name.substr(0, name.find(",")), r1.tl(), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
            }
            
    
	imshow("Display window",dst);
    waitKey(200);	
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Error in  proxy (base). Waiting" << endl;
		cout << "[" << PROGRAM_NAME << "]: Motivo: " << endl << ex << endl;
	}
}
