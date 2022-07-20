/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MixedFSR.h           					                               #
# ##############################################################################
**/


#ifndef MIXEDFSR_H
#define MIXEDFSR_H

#include "FSR.h"

class MixedFSR : public FSR 
{
	public:

		Eigen::Vector3f SampleMotion(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> u, std::vector<float> weights, Eigen::Vector3f noise);


};

#endif