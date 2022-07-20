/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: AggressiveLowVarianceResampling.h           	                       #
# ##############################################################################
**/


#ifndef AGGRESSIVELOWVARIANCERESAMPLING_H
#define AGGRESSIVELOWVARIANCERESAMPLING_H

#include "Resampling.h"
#include <vector>
#include <eigen3/Eigen/Dense>

class AggressiveLowVarianceResampling: public Resampling
{
	public:	
		void Resample(std::vector<Particle>& particles);

};

#endif