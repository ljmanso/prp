/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

// Ice includes
#include <Ice/Ice.h>
#include <ObjectDetection.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompObjectDetection;

class ObjectDetectionI : public virtual RoboCompObjectDetection::ObjectDetection
{
public:
	ObjectDetectionI(GenericWorker *_worker);
	~ObjectDetectionI();
	
	void grabTheAR(const Ice::Current&);
	void aprilFitModel(const string  &model, const Ice::Current&);
	void segmentImage(const Ice::Current&);
	void mindTheGapPC(const Ice::Current&);
	void getRotation( float  &rx,  float  &ry,  float  &rz, const Ice::Current&);
	void centroidBasedPose( float  &x,  float  &y,  float  &theta, const Ice::Current&);
	void reloadVFH(const string  &pathToSet, const Ice::Current&);
	void ransac(const string  &model, const Ice::Current&);
	void euclideanClustering( int  &numCluseters, const Ice::Current&);
	void passThrough(const Ice::Current&);
	void surfHomography( listType  &guesses, const Ice::Current&);
	void fitTheViewVFH(const Ice::Current&);
	void saveRegPose(const string  &label, const int  numPoseToSave,  pose6D  &tag1,  pose6D  &tag2,  pose6D  &tag3,  pose6D  &tag4,  pose6D  &tag5,  pose6D  &tag6,  pose6D  &tag7,  pose6D  &tag8,  pose6D  &tag9, const Ice::Current&);
	void showObject(const int  numObject, const Ice::Current&);
	void convexHull(const string  &model, const Ice::Current&);
	void mirrorPC(const Ice::Current&);
	bool findTheObject(const string  &objectTofind, const Ice::Current&);
	void statisticalOutliersRemoval(const Ice::Current&);
	void loadTrainedVFH(const Ice::Current&);
	void reset(const Ice::Current&);
	void normalSegmentation(const string  &model, const Ice::Current&);
	void getInliers(const string  &model, const Ice::Current&);
	void getPose( float  &x,  float  &y,  float  &z, const Ice::Current&);
	void vfh( listType  &guesses, const Ice::Current&);
	bool findObjects( listObject  &lObjects, const Ice::Current&);
	void grabThePointCloud(const string  &image, const string  &pcd, const Ice::Current&);
	void fitModel(const string  &model, const string  &method, const Ice::Current&);
	void projectInliers(const string  &model, const Ice::Current&);
	void guessPose(const string  &label,  pose6D  &guess, const Ice::Current&);
	void extractPolygon(const string  &model, const Ice::Current&);
	void saveCanonPose(const string  &label, const int  numPoseToSave,  pose6D  &tag1,  pose6D  &tag2,  pose6D  &tag3,  pose6D  &tag4,  pose6D  &tag5,  pose6D  &tag6,  pose6D  &tag7,  pose6D  &tag8,  pose6D  &tag9, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
