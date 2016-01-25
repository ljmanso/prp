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
        
        processDataFromDir("/home/marcog/robocomp/components/prp/experimentFiles/images/");
        
        //show map after processing
        for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
        {
                cout << "Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
        
        save_tables_info();
        load_tables_info();
        
        //show map after processing
        for (MapIterator iter = table1.begin(); iter != table1.end(); iter++)
        {
                cout << "Key: " << iter->first << endl << "Value: " << iter->second<< endl;
        }
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

std::string SpecificWorker::lookForObject(std::string label)
{
    std::string table;
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



