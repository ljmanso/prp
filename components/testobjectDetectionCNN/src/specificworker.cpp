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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
  first = true;
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
///run once: read image file from argument and pass to objectdetectionCNN module. 	
if(first)
  {
    first =false;
	Mat src=imread(inputFile,1);
	
	Mat dst = src.clone();
	putText(dst, "Computing object detections.", Point(20,20), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	putText(dst, "Will take a while!!!", Point(20,40), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	putText(dst, "kindly be patient.", Point(20,60), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	imshow("Source image", dst );
    waitKey(200);
    ///convert Mat to ColorSeq for transfering
    ///todo shallow conversion or better serialization. 
    
	int k=0;
    ColorSeq rgbMatrix;
    rgbMatrix.resize(src.rows*src.cols);
    for(int i=0;i<src.rows;i++)
	{
		for(int j=0;j<src.cols;j++)	
			{
				Vec3b intensity = src.at<Vec3b>(i,j);
				
				rgbMatrix[k].blue = intensity.val[0];
				rgbMatrix[k].green = intensity.val[1];
				rgbMatrix[k].red = intensity.val[2];
                k++;
             }
    }
    ResultList result;
    ///call objectdetectioncnn module and get detections.
    objectdetectioncnn_proxy->getLabelsFromImage(rgbMatrix, src.rows, src.cols, result);
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
                rectangle( src, r1.tl(), r1.br(), rect_color, 2, 8, 0 );
                string name=result[i].name;
                putText(src, name.substr(0, name.find(",")), r1.tl(), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
            }
            
    
	imshow("Source image", src );
    waitKey(0);
  }

}
