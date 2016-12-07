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
#include "objectdetectionI.h"

ObjectDetectionI::ObjectDetectionI(GenericWorker *_worker)
{
	worker = _worker;
}


ObjectDetectionI::~ObjectDetectionI()
{
}

void ObjectDetectionI::grabTheAR(const Ice::Current&)
{
	worker->grabTheAR();
}

void ObjectDetectionI::aprilFitModel(const string  &model, const Ice::Current&)
{
	worker->aprilFitModel(model);
}

void ObjectDetectionI::segmentImage(const Ice::Current&)
{
	worker->segmentImage();
}

void ObjectDetectionI::mindTheGapPC(const Ice::Current&)
{
	worker->mindTheGapPC();
}

void ObjectDetectionI::getRotation( float  &rx,  float  &ry,  float  &rz, const Ice::Current&)
{
	worker->getRotation(rx, ry, rz);
}

void ObjectDetectionI::centroidBasedPose( float  &x,  float  &y,  float  &theta, const Ice::Current&)
{
	worker->centroidBasedPose(x, y, theta);
}

void ObjectDetectionI::reloadVFH(const string  &pathToSet, const Ice::Current&)
{
	worker->reloadVFH(pathToSet);
}

void ObjectDetectionI::ransac(const string  &model, const Ice::Current&)
{
	worker->ransac(model);
}

void ObjectDetectionI::euclideanClustering( int  &numCluseters, const Ice::Current&)
{
	worker->euclideanClustering(numCluseters);
}

void ObjectDetectionI::passThrough(const Ice::Current&)
{
	worker->passThrough();
}

void ObjectDetectionI::surfHomography( listType  &guesses, const Ice::Current&)
{
	worker->surfHomography(guesses);
}

void ObjectDetectionI::fitTheViewVFH(const Ice::Current&)
{
	worker->fitTheViewVFH();
}

void ObjectDetectionI::saveRegPose(const string  &label, const int  numPoseToSave, const pose6D  &tag1, const pose6D  &tag2, const pose6D  &tag3, const pose6D  &tag4, const pose6D  &tag5, const pose6D  &tag6, const pose6D  &tag7, const pose6D  &tag8, const pose6D  &tag9, const Ice::Current&)
{
	worker->saveRegPose(label, numPoseToSave, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9);
}

void ObjectDetectionI::showObject(const int  numObject, const Ice::Current&)
{
	worker->showObject(numObject);
}

void ObjectDetectionI::convexHull(const string  &model, const Ice::Current&)
{
	worker->convexHull(model);
}

void ObjectDetectionI::mirrorPC(const Ice::Current&)
{
	worker->mirrorPC();
}

bool ObjectDetectionI::findTheObject(const string  &objectTofind, const Ice::Current&)
{
	return worker->findTheObject(objectTofind);
}

void ObjectDetectionI::statisticalOutliersRemoval(const Ice::Current&)
{
	worker->statisticalOutliersRemoval();
}

void ObjectDetectionI::loadTrainedVFH(const Ice::Current&)
{
	worker->loadTrainedVFH();
}

void ObjectDetectionI::reset(const Ice::Current&)
{
	worker->reset();
}

void ObjectDetectionI::normalSegmentation(const string  &model, const Ice::Current&)
{
	worker->normalSegmentation(model);
}

void ObjectDetectionI::getInliers(const string  &model, const Ice::Current&)
{
	worker->getInliers(model);
}

void ObjectDetectionI::getPose( float  &x,  float  &y,  float  &z, const Ice::Current&)
{
	worker->getPose(x, y, z);
}

void ObjectDetectionI::vfh( listType  &guesses, const Ice::Current&)
{
	worker->vfh(guesses);
}

void ObjectDetectionI::grabThePointCloud(const string  &image, const string  &pcd, const Ice::Current&)
{
	worker->grabThePointCloud(image, pcd);
}

void ObjectDetectionI::fitModel(const string  &model, const string  &method, const Ice::Current&)
{
	worker->fitModel(model, method);
}

void ObjectDetectionI::projectInliers(const string  &model, const Ice::Current&)
{
	worker->projectInliers(model);
}

void ObjectDetectionI::guessPose(const string  &label,  pose6D  &guess, const Ice::Current&)
{
	worker->guessPose(label, guess);
}

void ObjectDetectionI::extractPolygon(const string  &model, const Ice::Current&)
{
	worker->extractPolygon(model);
}

void ObjectDetectionI::saveCanonPose(const string  &label, const int  numPoseToSave, const pose6D  &tag1, const pose6D  &tag2, const pose6D  &tag3, const pose6D  &tag4, const pose6D  &tag5, const pose6D  &tag6, const pose6D  &tag7, const pose6D  &tag8, const pose6D  &tag9, const Ice::Current&)
{
	worker->saveCanonPose(label, numPoseToSave, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9);
}






