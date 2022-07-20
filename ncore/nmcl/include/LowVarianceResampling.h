/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: LowVarianceResampling.h           		                               #
# ##############################################################################
**/


#ifndef LOWVARIANCERESAMPLING_H
#define LOWVARIANCERESAMPLING_H

#include "Resampling.h"
#include <vector>
#include <eigen3/Eigen/Dense>

class LowVarianceResampling: public Resampling
{
	public:	
		void Resample(std::vector<Particle>& particles);

};

#endif