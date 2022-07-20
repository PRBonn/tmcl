/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: AggressiveLowVarianceResampling.cpp                                   #
# ##############################################################################
**/


#include "AggressiveLowVarianceResampling.h"

#include <numeric>
#include <functional> 

void AggressiveLowVarianceResampling::Resample(std::vector<Particle>& particles)
{
	std::vector<Particle> new_particles;
	int n_particles = particles.size();

	double unitW = 1.0 / n_particles;
	double r = drand48() * 1.0 / n_particles;
	double acc = particles[0].weight;
	int i = 0;

	for(int j = 0; j < n_particles; ++j)
	{
		double U = r + j * 1.0 / n_particles;
		while((U > acc) && (i < n_particles))
		{
			++i;
			acc += particles[i].weight;
		}
		particles[i].weight = unitW;
		new_particles.push_back(particles[i]);
	}
	particles = new_particles;
}