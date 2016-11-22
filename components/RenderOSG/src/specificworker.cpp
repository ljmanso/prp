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
if(first)
{
	first=false;
	double distance_ratio = 1;
	double field_of_view = 25;
	double azimuth = 0;
	double elevation = 0;
	double yaw = 0;
	
	strvec fileNamesVector;
	std::string modelname;
    std::ifstream modelfile;
    
    read_renderer_config("etc/renderer_config");
    
    
    modelfile.open(modelfile_name);
    
    
    boost::filesystem::path img_dir(output_image_directory_name);
    boost::filesystem::path depth_dir(output_depth_directory_name);
    boost::filesystem::create_directory(img_dir);
    boost::filesystem::create_directory(depth_dir);
    
    int modelid=1;
    if (modelfile.is_open())
    {
      while ( getline (modelfile,modelname) )
		{
			std::cout << modelname << '\n';
			fileNamesVector.push_back(modelname);
	const bool offScreen    = true;
    int screenWidth   = viewport_size_x;
    int screenHeight  = viewport_size_y;
    
    std::stringstream dirStream;
    dirStream <<"M"<< std::setw(6) << std::setfill('0') << modelid;
    //std::cout<<argv[2]+std::string("/")+dirStream.str()<<std::endl;
    //std::cout<<argv[3]+std::string("/")+dirStream.str()<<std::endl;
    
    boost::filesystem::path img_dir_model(output_image_directory_name+std::string("/")+dirStream.str());
    boost::filesystem::path depth_dir_model(output_depth_directory_name+std::string("/")+dirStream.str());
    boost::filesystem::create_directory(img_dir_model);
    boost::filesystem::create_directory(depth_dir_model);
    
        
    unsigned int imgid=0;
    
    
    for(int azi_bin=0;azi_bin<Nbin_azimuth;azi_bin++)
    {
		std::cout<<"modelid="<<modelid<<", imgid="<<imgid<<", azimuth="<<azimuth<<std::endl;
		for(int ele_bin=0;ele_bin<Nbin_elevation;ele_bin++)
		{
			//std::cout<<"modelid="<<modelid<<", imgid="<<imgid<<", azi="<<azimuth<<", ele="<<elevation<<std::endl;
			for(int yaw_bin=0;yaw_bin<Nbin_yaw;yaw_bin++)
			{
				std::stringstream imgStream, depthStream;
				imgStream << output_image_directory_name << "/"<<dirStream.str() << "/" << "I"<<std::setw(8) << std::setfill('0') << imgid << ".jpg";
				depthStream << output_depth_directory_name << "/"<<dirStream.str() << "/" << "D"<<std::setw(8) << std::setfill('0') << imgid << ".tiff";
				
				azimuth=start_azimuth+azi_bin*360/Nbin_azimuth;
				elevation=start_elevation+ele_bin*360/Nbin_elevation;
				yaw=start_yaw+yaw_bin*360/Nbin_yaw;
				
				CLR::Renderer renderer;
				renderer.initialize(fileNamesVector, offScreen, screenWidth, \
				screenHeight, azimuth, elevation, yaw, distance_ratio, field_of_view, imgStream.str(), depthStream.str());

				int depth_size      = screenWidth * screenHeight;
				int rendering_size  = depth_size * 3;
    
				/// Initialize rendering
				std::vector<unsigned char> rendering(rendering_size);
				std::vector<double> depth(depth_size);
				
				unsigned char * ptr_rendering = rendering_size ? const_cast<unsigned char *>(&rendering[0])
                                                   : static_cast<unsigned char *>(NULL);
				double * ptr_depth = depth_size ? const_cast<double *>(&depth[0])
                                    : static_cast<double *>(NULL);
				renderer.render(ptr_rendering, ptr_depth);
				imgid++;
				}
			}
		}		
	
    std::cout<<"Done....!"<<std::endl;
    fileNamesVector.pop_back();
    modelid++;
    }
			modelfile.close();
	}
	else std::cout << "Unable to open file"; 
	}
	exit(0);
}

void SpecificWorker::read_renderer_config(const string config_file)
{
std::ifstream f(config_file.c_str());
if(!f.good())
	{
		cout<<"Renderer config file not found at: "<<config_file<<"\n exiting...!"<<endl;
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
    
             if((int)name.find("modelfile_name")>=0) modelfile_name=value;
        else if((int)name.find("output_image_directory_name")>=0) output_image_directory_name=value;
        else if((int)name.find("output_depth_directory_name")>=0) output_depth_directory_name=value;
        else if((int)name.find("viewport_size_x")>=0) viewport_size_x=std::stoi(value);
        else if((int)name.find("viewport_size_y")>=0) viewport_size_y=std::stoi(value);
        else if((int)name.find("Nbin_azimuth")>=0) Nbin_azimuth=std::stoi(value);
        else if((int)name.find("start_azimuth")>=0) start_azimuth=std::stoi(value);
        else if((int)name.find("Nbin_elevation")>=0) Nbin_elevation=std::stoi(value);
        else if((int)name.find("start_elevation")>=0) start_elevation=std::stoi(value);
        else if((int)name.find("Nbin_yaw")>=0) Nbin_yaw=std::stoi(value);
        else if((int)name.find("start_yaw")>=0) start_yaw=std::stoi(value);
        
    }    
        if(modelfile_name.length()<3)
        {
           
           cout<<" model file not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	    
	    if(output_image_directory_name.length()<3)
        {
           cout<<"output image directory not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	    
	    
	    if(output_image_directory_name.length()<3)
        {
           cout<<"output image directory not found in config file. \nexiting...!"<<endl;
		   exit(1);
	    }
	f.close();    
        //cout<<model_file<<"OK.....4"<<endl;
    
}






