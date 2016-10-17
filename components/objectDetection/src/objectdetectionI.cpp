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

objectDetectionI::objectDetectionI(GenericWorker *_worker)
{
	worker = _worker;
}


objectDetectionI::~objectDetectionI()
{
}

void objectDetectionI::grabTheAR(const Ice::Current&)
{
	worker->grabTheAR();
}

void objectDetectionI::aprilFitModel(const string  &model, const Ice::Current&)
{
	worker->aprilFitModel(model);
}

void objectDetectionI::segmentImage(const Ice::Current&)
{
	worker->segmentImage();
}

void objectDetectionI::mindTheGapPC(const Ice::Current&)
{
	worker->mindTheGapPC();
}

void objectDetectionI::getRotation( float  &rx,  float  &ry,  float  &rz, const Ice::Current&)
{
	worker->getRotation(rx, ry, rz);
}

void objectDetectionI::centroidBasedPose( float  &x,  float  &y,  float  &theta, const Ice::Current&)
{
	worker->centroidBasedPose(x, y, theta);
}

void objectDetectionI::reloadVFH(const string  &pathToSet, const Ice::Current&)
{
	worker->reloadVFH(pathToSet);
}

void objectDetectionI::ransac(const string  &model, const Ice::Current&)
{
	worker->ransac(model);
}

void objectDetectionI::euclideanClustering( int  &numCluseters, const Ice::Current&)
{
	worker->euclideanClustering(numCluseters);
}

void objectDetectionI::passThrough(const Ice::Current&)
{
	worker->passThrough();
}

void objectDetectionI::surfHomography( listType  &guesses, const Ice::Current&)
{
	worker->surfHomography(guesses);
}

void objectDetectionI::fitTheViewVFH(const Ice::Current&)
{
	worker->fitTheViewVFH();
}

void objectDetectionI::saveRegPose(const string  &label, const int  numPoseToSave, const pose6D  &tag1, const pose6D  &tag2, const pose6D  &tag3, const pose6D  &tag4, const pose6D  &tag5, const pose6D  &tag6, const pose6D  &tag7, const pose6D  &tag8, const pose6D  &tag9, const Ice::Current&)
{
	worker->saveRegPose(label, numPoseToSave, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9);
}

void objectDetectionI::showObject(const int  numObject, const Ice::Current&)
{
	worker->showObject(numObject);
}

void objectDetectionI::convexHull(const string  &model, const Ice::Current&)
{
	worker->convexHull(model);
}

void objectDetectionI::mirrorPC(const Ice::Current&)
{
	worker->mirrorPC();
}

bool objectDetectionI::findTheObject(const string  &objectTofind, const Ice::Current&)
{
	return worker->findTheObject(objectTofind);
}

void objectDetectionI::statisticalOutliersRemoval(const Ice::Current&)
{
	worker->statisticalOutliersRemoval();
}

void objectDetectionI::loadTrainedVFH(const Ice::Current&)
{
	worker->loadTrainedVFH();
}

void objectDetectionI::reset(const Ice::Current&)
{
	worker->reset();
}

void objectDetectionI::normalSegmentation(const string  &model, const Ice::Current&)
{
	worker->normalSegmentation(model);
}

void objectDetectionI::getInliers(const string  &model, const Ice::Current&)
{
	worker->getInliers(model);
}

void objectDetectionI::getPose( float  &x,  float  &y,  float  &z, const Ice::Current&)
{
	worker->getPose(x, y, z);
}

void objectDetectionI::vfh( listType  &guesses, const Ice::Current&)
{
	worker->vfh(guesses);
}

void objectDetectionI::grabThePointCloud(const string  &image, const string  &pcd, const Ice::Current&)
{
	worker->grabThePointCloud(image, pcd);
}

void objectDetectionI::fitModel(const string  &model, const string  &method, const Ice::Current&)
{
	worker->fitModel(model, method);
}

void objectDetectionI::projectInliers(const string  &model, const Ice::Current&)
{
	worker->projectInliers(model);
}

void objectDetectionI::guessPose(const string  &label,  pose6D  &guess, const Ice::Current&)
{
	worker->guessPose(label, guess);
}

void objectDetectionI::extractPolygon(const string  &model, const Ice::Current&)
{
	worker->extractPolygon(model);
}

void objectDetectionI::saveCanonPose(const string  &label, const int  numPoseToSave, const pose6D  &tag1, const pose6D  &tag2, const pose6D  &tag3, const pose6D  &tag4, const pose6D  &tag5, const pose6D  &tag6, const pose6D  &tag7, const pose6D  &tag8, const pose6D  &tag9, const Ice::Current&)
{
	worker->saveCanonPose(label, numPoseToSave, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9);
}






