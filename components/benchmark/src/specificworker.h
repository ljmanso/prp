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
 #include <pcl/io/pcd_io.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/segmentation/extract_clusters.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/io/pcd_io.h>
 #include <pcl/conversions.h>
 #include <pcl/point_types_conversion.h>
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <pcl/surface/convex_hull.h>
 #include <pcl/surface/concave_hull.h>
 #include <pcl/filters/voxel_grid.h>
 #include <pcl/registration/sample_consensus_prerejective.h>
 #include <pcl/features/fpfh_omp.h>
 #include <pcl/common/time.h>
#endif

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

typedef pcl::PointXYZRGB PointT;


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute(); 	

	void save(std::string base);
	
	void on_mugButton_clicked()       { save("mug"); }
	void on_laptopButton_clicked()    { save("laptop"); }
	void on_wrenchButton_clicked()    { save("wrench"); }
	void on_ballButton_clicked()      { save("ball"); }
	void on_glassesButton_clicked()   { save("glasses"); }
	void on_keysButton_clicked()      { save("keys"); }
	void on_bottleButton_clicked()    { save("bottle"); }
	void on_canButton_clicked()       { save("can"); }
	void on_bookButton_clicked()      { save("book"); }
	void on_cellphoneButton_clicked() { save("cellphone"); }
	void on_appleButton_clicked()     { save("apple"); }
	void on_walletButton_clicked()    { save("wallet"); }
	
private:
	RoboCompRGBD::PointSeq points;
	pcl::PCDWriter writer;
	pcl::PointCloud<PointT>::Ptr cloud;

	cv::Mat frame;
};

#endif

