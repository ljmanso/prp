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
 #include <pcl/segmentation/extract_clusters.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/conversions.h>
 #include <pcl/point_types_conversion.h>
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <pcl/surface/convex_hull.h>
 #include <pcl/surface/concave_hull.h>
#endif

#include <genericworker.h>

#ifndef Q_MOC_RUN
	#include <innermodel/innermodel.h>
	#include <innermodel/innermodelviewer.h>
	#include "color_segmentation/Segmentator.h"
	#include "shapes/table.h"
	#include "vfh/vfh.h"
	#ifdef USE_QTGUI
		#include "viewer/viewer.h"
		#include <QGraphicsPixmapItem>
	#endif
	#include "pointcloud/pointcloud.h"
	#include "time.h"
#endif


#define DEBUG 0
#define SAVE_DATA 0
#define THRESHOLD 0.8
#define MEDIDA 1.
#define offset_object 125

#define SUB(dst, src1, src2) \
  { \
    if ((src2)->tv_nsec > (src1)->tv_nsec) { \
      (dst)->tv_sec = (src1)->tv_sec - (src2)->tv_sec - 1; \
      (dst)->tv_nsec = ((src1)->tv_nsec - (src2)->tv_nsec) + 1000000000; \
    } \
    else { \
      (dst)->tv_sec = (src1)->tv_sec - (src2)->tv_sec; \
      (dst)->tv_nsec = (src1)->tv_nsec - (src2)->tv_nsec; \
    } \
  }

typedef pcl::PointXYZRGB PointT;

using namespace computepointcloud;

class SpecificWorker : public GenericWorker
{
	bool test;
	QString id_robot, id_camera,id_camera_transform;
	string descriptors_extension, pathLoadDescriptors;
	InnerModel *innermodel;
	
	//for poses calculation respect to the canonical one
	InnerModel *poses_inner;
	tagsList tags;
	QMutex april_mutex;
	int num_pose;

	int num_object_found;
	std::string file_view_mathing;

	int num_scene;
	
	pcl::PCDWriter writer;

	float marca_tx, marca_ty, marca_tz, marca_rx, marca_ry, marca_rz;
        
	//Cloud of the current points for pcl
	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointIndices::Ptr ransac_inliers;
	pcl::PointCloud<PointT>::Ptr projected_plane;
	pcl::PointCloud<PointT>::Ptr cloud_hull;
	pcl::PointIndices::Ptr prism_indices;
	
	//Image of the current view for opencv
	cv::Mat rgb_image;
	cv::Mat color_segmented;
    RTMat viewpoint_transform;
	
	//Point cloud grabing
	RoboCompRGBD::ColorSeq rgbMatrix;	
	RoboCompRGBD::depthType distanceMatrix;
	RoboCompRGBD::PointSeq points_kinect;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	
	//color Segmentator
	Segmentator segmentator;
	
	//euclidean clustering
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<pcl::PointCloud<PointT>::Ptr> cluster_clouds;
    
	//VFH
	boost::shared_ptr<VFH> vfh_matcher;
	std::vector<VFH::file_dist_t> vfh_guesses;
	boost::shared_ptr<Table> table;
	
#ifdef USE_QTGUI
	QGraphicsPixmapItem* item_pixmap;
	vector<QGraphicsTextItem*> V_text_item;
	vector<QGraphicsPixmapItem*> V_pixmap_item;
	
	QGraphicsScene scene;

	boost::shared_ptr<Viewer> viewer;
	QVec poseoffset;
#endif
	pcl::PointCloud< PointT >::Ptr copy_scene;
	
  
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	
	bool setParams(RoboCompCommonBehavior::ParameterList params);
/*
 * Method of Interface ObjectDetection.ice
 */
	bool findTheObject(const string &objectTofind, pose6D &pose);
	bool findObjects(listObject &lObjects);

/*
 * Method of Interface AprilTags.ice
 */
	void newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);
	void newAprilTag(const tagsList &tags);

public slots:
	void compute();
#ifdef USE_QTGUI
	void saveView();
	void findTheObject_Button();
	void reloadVFH_Button();
	void fullRun_Button();
#endif
	
private:
	QVec extraerposefromTM(QMat M);
	void grabThePointCloud();
	void readThePointCloud(const string &image, const string &pcd);
	void ransac();
	void projectInliers();
	void convexHull();
	void extractPolygon();
	void euclideanClustering(int &numCluseters);

	void caputurePointCloudObjects();

	void updateinner();
	void loadTrainedVFH();
	void vfh(listType &guesses);
	bool aprilSeen(QVec &offset);
	
	void reloadVFH();
	pose6D getPose();
#ifdef USE_QTGUI
	void initSaveObject(const string &label, const int numPoseToSave);
	QVec saveRegPose(const string &label, const int numPoseToSave);
	
	void updatergbd();
	void settexttocloud(std::string name,pcl::PointCloud<PointT>::Ptr cloud);
	void paintcloud(pcl::PointCloud<PointT>::Ptr cloud);
	void removeAllpixmap();
#endif

// 	void segmentImage();
// 	void passThrough();
// 	void statisticalOutliersRemoval();

};

#endif

