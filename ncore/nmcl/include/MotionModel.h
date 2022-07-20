/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MotionModel.h           				                               #
# ##############################################################################
**/


#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include <eigen3/Eigen/Dense>
#include <vector>

class MotionModel
{

	public:
		
		virtual Eigen::Vector3f Backward(Eigen::Vector3f p1, Eigen::Vector3f p2) = 0;

		virtual Eigen::Vector3f Forward(Eigen::Vector3f p1, Eigen::Vector3f u) = 0;

		virtual Eigen::Vector3f SampleMotion(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> u, std::vector<float> weights, Eigen::Vector3f noise) = 0;


		virtual ~MotionModel(){};

};

#endif