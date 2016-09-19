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

/**
       \brief
       @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#ifndef Q_MOC_RUN

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/range_image/range_image.h>
#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <innermodel/innermodelviewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/core/utility.hpp>
//#include <opencv2/saliency.hpp>
//#include "opencv2/imgcodecs.hpp"

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>
#include <sstream>

#include <boost/serialization/map.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "caffeClassifier.h"

#include "labeler.h"
#include "mapmodel.h"
#include "word2vec.h"

#endif

#ifdef CONVNET
#ifdef __cplusplus
extern "C"{
#endif 
    
#include "ccv/ccv.h"

#ifdef __cplusplus
}

#endif
#endif

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define TABLE_DISTANCE 2500
#define OFFSET 50
#define OFFSET_TOP 80
#define INNER_VIEWER

//#include "t.hpp"

typedef pcl::PointXYZRGB PointT;

class SpecificWorker : public GenericWorker
{
Q_OBJECT
private:
#ifdef CONVNET
	ccv_convnet_t* convnet;
#endif
	fstream file;
	bool first;
	
	Model w2v_model;

public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

        void loadTablesFromModel();
	bool reloadConfigAgent();
	bool activateAgent(const ParameterMap &prs);
	bool setAgentParameters(const ParameterMap &prs);
	ParameterMap getAgentParameters();
	void killAgent();
	int uptimeAgent();
	bool deactivateAgent();
	StateStruct getAgentState();
	void semanticDistance(const string &word1, const string &word2, float &result);
        
	void processDataFromDir(const boost::filesystem::path &base_dir);
	//given an image and its location it process its objects and save them to the corresponding location
	void processImage(cv::Mat image, std::string location);
	void addLabelsToTable(ResultList result, std::string location);
	void load_tables_info();
	   
	void segmentObjects3D(pcl::PointCloud<PointT>::Ptr cloud, cv::Mat image, std::vector<cv::Mat> &result);
	std::string lookForObject(std::string label);
	void getLabelsFromImage(const RoboCompObjectOracle::ColorSeq &image, ResultList &result);
	void getLabelsFromImageWithCaffe(cv::Mat matImage, ResultList &result);
	void structuralChange(const RoboCompAGMWorldModel::World &modification);
	void edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications);
	void edgeUpdated(const RoboCompAGMWorldModel::Edge &modification);
	void symbolUpdated(const RoboCompAGMWorldModel::Node &modification);
	void symbolsUpdated(const RoboCompAGMWorldModel::NodeSequence &modifications);
	RoboCompObjectOracle::ColorSeq convertMat2ColorSeq(cv::Mat rgb);

	std::string checkTable(RoboCompRGBD::ColorSeq image);
	std::string checkTableApril(RoboCompRGBD::ColorSeq image);
	bool isTableVisible(RoboCompRGBD::ColorSeq image, const std::string tableIMName, const float tableWidth, const float tableHeight, const float tableDepth);
	void processDataFromKinect(cv::Mat matImage, const RoboCompRGBD::PointSeq &points, std::string location);
	void labelImage(cv::Mat matImage, std::string location);
	void showTablesOnInterface();
	
public slots:
	void compute();
	void save_tables_info();

private:
	
	QMutex *inner_mutex, *world_mutex, *agent_mutex;
	
	//config params
	bool save_full_data, save_table_data, labeling;
	InnerModelCamera *camera;
	
	std::map<std::string, double>  table1;
	std::map<std::string, double>  table2;
	std::map<std::string, double>  table3;
	std::map<std::string, double>  table4;
	std::map<std::string, double>  table5;
	 
	MapModel mapmodel_1;
	MapModel mapmodel_2;
	MapModel mapmodel_3;
	MapModel mapmodel_4;
	MapModel mapmodel_5;
	
	QMap<std::string, double> table1_qmat;
	QMap<std::string, double> table2_qmat;
	QMap<std::string, double> table3_qmat;
	QMap<std::string, double> table4_qmat;
	QMap<std::string, double> table5_qmat;
	
    std::vector< std::pair< std::map<std::string, double>, int> > tables; 
	
	CaffeClassifier *caffe_classifier;
	std::shared_ptr<Labeler> labeler;
        
	int image_save_counter;
	int image_segmented_counter;
	std::string action;
	QTime actionTime;
	ParameterMap params;
	AGMModel::SPtr worldModel;
	QTime worldModelTime;
	InnerModel *innerModel;
	bool active;
	bool setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated);
	void sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel);

	int32_t modifiedWorld;
	void imagineMostLikelyOBJECTPosition(string objectType);
	void action_imagineMostLikelyMugInPosition();
	void action_imagineMostLikelyCoffeePotInPosition();
	void action_imagineMostLikelyMilkInPosition();
	
	RoboCompDifferentialRobot::TBaseState bState;
	RoboCompJointMotor::MotorStateMap hState;
	RoboCompRGBD::ColorSeq rgbImage;
	RoboCompRGBD::PointSeq points;
	RoboCompObjectOracle::ColorSeq oracleImage;
	pcl::PointCloud<PointT>::Ptr cloud;
	cv::Mat matImage;
	pcl::PointCloud<PointT>::Ptr fullCloud;
	cv::Mat fullImage;
	int left, right, down, up;
	
	#ifdef INNER_VIEWER
	//AGM Model viewer
	osgGA::TrackballManipulator *manipulator;
	OsgView *osgView;	
	InnerModelViewer *innerViewer; 
	
	void updateViewer();
	void changeInner ();
	#endif
};

#endif

