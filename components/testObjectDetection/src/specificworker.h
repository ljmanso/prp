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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "time.h"

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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
public slots:
	void compute();
// 	void grabThePointCloud();
// 	void ransac();
// 	void projectInliers();
// 	void convexHull();
// 	void extractPolygon();
// 	void euclideanExtract();
	void findTheObject();
// 	void getPose();
// 	void getRotation();
	void reloadVFH();
	void fullRun();
private:

};

#endif
