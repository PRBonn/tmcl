/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: GaussianPredictStrategy.h      	          	                       #
# ##############################################################################
**/


#ifndef GAUSSIANPREDICTSTRATEGY
#define GAUSSIANPREDICTSTRATEGY

#include "PredictStrategy.h"



class GaussianPredictStrategy : public PredictStrategy
{
public:
	
	void Predict(ReNMCL* renmcl, std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise);

};



#endif