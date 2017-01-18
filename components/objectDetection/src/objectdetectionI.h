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
	
	pose6D getPose(const Ice::Current&);
	bool findObjects( listObject  &lObjects, const Ice::Current&);
	void saveRegPose(const string  &label, const int  numPoseToSave, const Ice::Current&);
	bool findTheObject(const string  &objectTofind,  pose6D  &pose, const Ice::Current&);
	void saveCanonPose(const string  &label, const int  numPoseToSave, const Ice::Current&);
	void reloadVFH(const string  &pathToSet, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
