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
#include "caffe_helper.h" //caffe tools source
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	first=true;
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
	read_CNN_config("etc/caffe_config");
	boost::filesystem::path img_dir(img_dir_file);
	std::ofstream dataset;
	dataset.open(train_list_file);
	int num_models=0;
	if ( !boost::filesystem::exists( img_dir ) )
  {
    std::cout << "\nNot found: " << img_dir.filename() << std::endl;
    exit(1);
  }
  if ( boost::filesystem::is_directory( img_dir ) )
  {
    boost::filesystem::directory_iterator end_iter;
    for ( boost::filesystem::directory_iterator dir_itr( img_dir );
          dir_itr != end_iter;++dir_itr )
   	{
		//std::cout << dir_itr->path().filename()<<std::endl;
		std::string model_name=dir_itr->path().filename().string();
		//model_names.push_back(model_name);
		boost::filesystem::path img_dir_model(img_dir_file+std::string("/")+model_name);
		for ( boost::filesystem::directory_iterator inner_dir_itr( img_dir_model );
			inner_dir_itr != end_iter;++inner_dir_itr )
		{
			//std::string img_path=img_dir_file+std::string("/")+model_name+std::string("/")+inner_dir_itr->path().filename().string();
			std::string img_path=img_dir_file+std::string("/")+model_name+std::string("/")+inner_dir_itr->path().filename().string();
			dataset<<img_path<<" "<<std::stoi(model_name.substr(1,model_name.length()))<<std::endl;
			}
		std::cout<<model_name<<std::endl;
		num_models++;	
	}
  } 
  dataset.close();  		
  boost::filesystem::remove_all(train_list_lmdb);
  convert_imageset(train_list_file, train_list_lmdb, 256, 256, false);
  compute_image_mean(train_list_lmdb, mean_file);
  train(solver_file,model_file,weights_file,gpuids,snapshot_file);
  	
}
}

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
        else if((int)name.find("solver_file")>=0) solver_file=value;
        else if((int)name.find("mean_file")>=0) mean_file=value;
        else if((int)name.find("gpuids")>=0) gpuids=value;
        else if((int)name.find("img_dir_file")>=0) img_dir_file=value;
        else if((int)name.find("train_list_file")>=0) train_list_file=value;
        else if((int)name.find("train_list_lmdb")>=0) train_list_lmdb=value;
        
    }    
        
	f.close();    
    
}







