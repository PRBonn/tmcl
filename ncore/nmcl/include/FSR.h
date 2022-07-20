/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FSR.h                          		                               #
# ##############################################################################
**/

#ifndef FSR_H
#define FSR_H


#include "MotionModel.h"



class FSR: public MotionModel 
{
	public:
	    virtual Eigen::Vector3f Backward(Eigen::Vector3f p1, Eigen::Vector3f p2);

		virtual Eigen::Vector3f Forward(Eigen::Vector3f p1, Eigen::Vector3f u);

		virtual Eigen::Vector3f SampleMotion(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> u, std::vector<float> weights, Eigen::Vector3f noise);

};

#endif