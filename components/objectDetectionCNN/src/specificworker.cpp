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
#include "proposal_caffe.h"

using namespace cv;

Classifier* classifier;

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
///initialize caffe model only once
///read the model file and 	other parameters from "etc/caffe_config"

if(first)
{
	first=false;
	
	::google::InitGoogleLogging("objectDetectionCNN");
	//read_CNN_config("etc/caffe_config",model_file, trained_file, mean_file, label_file);
	read_CNN_config("etc/caffe_config");
	if(gpuid>-1)Caffe::set_mode(Caffe::GPU);
	else Caffe::set_mode(Caffe::CPU);
	classifier= new Classifier(model_file, weights_file, mean_file, label_file);
	
}

}


void SpecificWorker::getLabelsFromImage(const ColorSeq &image, const int rows, const int cols, ResultList &result)
{


Mat src,blt,tophat,gray,bw;

int morph_elem = 0;
int morph_size = 21;

///deep copy from ColorSeq to Mat
src.create(rows,cols,CV_8UC3);
int k=0;
for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)	
			{
				Vec3b intensity;
				intensity.val[0]=image[k].blue;
				intensity.val[1]=image[k].green;
				intensity.val[2]=image[k].red;
                src.at<Vec3b>(i,j)=intensity;
                k++;
             }
    }



/// Bilateral Filtering: edge preserving color and spatial filter.

  bilateralFilter(src, blt, 8, 40, 5, 1 );  


//imshow("Source image", blt); waitKey(0);      

  
  int operation = 4 + 2;

  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

  /// Apply the specified morphology operation (tophat).
  morphologyEx( blt, tophat, operation, element);
  cvtColor(tophat, gray, CV_BGR2GRAY);
  ///Thresholding to create binay image.
  int threshval=30;
  bw = threshval < 128 ? (gray > threshval) : (gray < threshval);
  Mat labelImage(gray.size(), CV_32S);
  ///Labeling each segment.
  int nLabels = connectedComponents(bw, labelImage, 8);
  std::vector<Rect> rects(nLabels);
  ///Fitting bounding-box to segments.
    for(int labelid = 0; labelid < nLabels; ++labelid){
		//Note:storing in form (rmin,cmin,rmax,cmax) and not (xmin,ymin,width,height)
        rects[labelid]=Rect(gray.rows,gray.cols, 0,0 ); 
    }
    
    Mat dst(gray.size(), CV_8UC3);
    for(int r = 0; r < dst.rows; ++r){
        for(int c = 0; c < dst.cols; ++c){
            int labelid = labelImage.at<int>(r, c);
            if(c<rects[labelid].x)rects[labelid].x=c;
            if(r<rects[labelid].y)rects[labelid].y=r;
	        if(c>rects[labelid].width)rects[labelid].width=c;
            if(r>rects[labelid].height)rects[labelid].height=r;
         }
     }
    dst=src;
    Scalar rect_color = Scalar( 0, 0, 255 );
    
    ///Extract each bounding-box, draw rectangle and classify using CNN
    for(int labelid = 0; labelid < nLabels; ++labelid){
         //Note:converting back to (xmin,ymin,width,height) format.
         rects[labelid].width=rects[labelid].width-rects[labelid].x;
	     rects[labelid].height=rects[labelid].height-rects[labelid].y;
         if(rects[labelid].width*rects[labelid].height<200)continue;            	 
         rectangle( dst, rects[labelid].tl(), rects[labelid].br(), rect_color, 2, 8, 0 );
         Rect r1=rects[labelid];
         Rect r2;
         int bord=2; 
         r2.x=r1.x-bord > 0 ? r1.x-bord : r1.x;
         r2.y=r1.y-bord > 0 ? r1.y-bord : r1.y;
         r2.width= r1.x+r1.width+bord < src.cols ? r1.width+bord:r1.width;
         r2.height=r1.y+r1.height+bord < src.rows ? r1.height+bord:r1.height;
         Mat roi = src( r2 );
         float top_score=-1;
         String top_labelid="";
         
         std::vector<Prediction> predictions = classifier->Classify(roi);
		 Prediction p = predictions[0];
	     top_score=p.second;
		 top_labelid=p.first;
		 putText(dst, top_labelid.substr(0, top_labelid.find(",")), rects[labelid].tl(), CV_FONT_HERSHEY_DUPLEX, 0.5, cvScalar(0,0,250), 1, CV_AA);
	     if(top_score<0.9)continue;
	     BoundingBox bb;
	     bb.x=r1.x;
	     bb.y=r1.y;
	     bb.width=r1.width;
	     bb.height=r1.height;
	     
	     ///return bounding boxes, category labels and prediction score.
	     Label l;
		 l.name = p.first;
		 l.believe = p.second;
	     l.bb=bb;
	     result.push_back(l);
	     
	     cout<<"Window ID:"<<labelid<<", detected category:"<<p.first<<":"<<top_labelid<<", score:"<<top_score<<endl;
             
    }
  //imshow( window_name, dst);
  //waitKey(0);

                				
}

//read_caffe_config("etc/caffe_config",model_file, trained_file, mean_file, label_file);
void SpecificWorker::read_CNN_config(const string config_file)
{
std::ifstream f(config_file.c_str());
if(!f.good())
	{
		cout<<"CNN config file not found at: "<<config_file<<"\n exiting...!"<<endl;
		exit(1);
	}
	
std::string line;

 while (getline(f, line))
    {
        istringstream ss(line);
        string name, value;
        
        ss >> name >> value;
        //std::cout<<"name="<<name<<std::endl;
        //std::cout<<"value="<<value<<std::endl;
        //cout<<"find(weights)"<<(int)name.find("caffemodel")<<endl;
        
        if((int)name.find("model_file")>=0) model_file=value;
        else if((int)name.find("weights_file")>=0) weights_file=value;
        else if((int)name.find("label_file")>=0) label_file=value;
        else if((int)name.find("mean_file")>=0) mean_file=value;
        else if((int)name.find("gpuid")>=0) gpuid=std::stoi(value);
    }    
        if(model_file.length()<3)
        {
           
           cout<<"CNN model file not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	    
	    if(weights_file.length()<3)
        {
           cout<<"CNN weights file not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	    
	    
	    if(label_file.length()<3)
        {
           cout<<"CNN label file not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	f.close();    
        //cout<<model_file<<"OK.....4"<<endl;
    
}




