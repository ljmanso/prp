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
#include "objectdetectionI.h"

ObjectDetectionI::ObjectDetectionI(GenericWorker *_worker)
{
	worker = _worker;
}


ObjectDetectionI::~ObjectDetectionI()
{
}

pose6D ObjectDetectionI::getPose(const Ice::Current&)
{
	return worker->getPose();
}

bool ObjectDetectionI::findObjects( listObject  &lObjects, const Ice::Current&)
{
	return worker->findObjects(lObjects);
}

void ObjectDetectionI::saveRegPose(const string  &label, const int  numPoseToSave, const Ice::Current&)
{
	worker->saveRegPose(label, numPoseToSave);
}

bool ObjectDetectionI::findTheObject(const string  &objectTofind,  pose6D  &pose, const Ice::Current&)
{
	return worker->findTheObject(objectTofind, pose);
}

void ObjectDetectionI::saveCanonPose(const string  &label, const int  numPoseToSave, const Ice::Current&)
{
	worker->saveCanonPose(label, numPoseToSave);
}

void ObjectDetectionI::reloadVFH(const string  &pathToSet, const Ice::Current&)
{
	worker->reloadVFH(pathToSet);
}






