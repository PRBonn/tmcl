/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: PredictStrategy.h      	          	                               #
# ##############################################################################
**/


#ifndef PREDICTSTRATEGY
#define PREDICTSTRATEGY

#include <vector>
#include <eigen3/Eigen/Dense>

class ReNMCL;

class PredictStrategy
{
public:

	virtual void Predict(ReNMCL* renmcl, std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise) = 0;

	virtual ~PredictStrategy() {};
};



#endif