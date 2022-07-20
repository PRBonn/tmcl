/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Resampling.h          				                           		   #
# ##############################################################################
**/

#ifndef RESAMPLING_H
#define RESAMPLING_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include "Particle.h"

class Resampling
{
	public:
		virtual void Resample(std::vector<Particle>& particles) = 0;

};

#endif 