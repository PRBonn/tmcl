/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: UniformPredictStrategy.cpp    	          	                           #
# ##############################################################################
**/

#include "UniformPredictStrategy.h"
#include "Particle.h"
#include "ReNMCL.h"

void UniformPredictStrategy::Predict(ReNMCL* renmcl, std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise)
{
	for(int i = 0; i < renmcl->o_numParticles; ++i)
	{
		Eigen::Vector3f pose = renmcl->o_motionModel->SampleMotion(renmcl->o_particles[i].pose, u, odomWeights, noise);
		renmcl->o_particles[i].pose = pose;
		//particle pruning - if particle is outside the map, we replace it
		while (!renmcl->o_sensorModel->IsValid(renmcl->o_particles[i]))
		{
			Particle p = (renmcl->o_sensorModel->InitUniform(1))[0];
			p.weight = 1.0 / renmcl->o_numParticles;
			renmcl->o_particles[i].pose = p.pose;
		}
	}
}