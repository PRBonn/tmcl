/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: GaussianPredictStrategy.cpp     	          	                       #
# ##############################################################################
**/


#include "GaussianPredictStrategy.h"
#include "Particle.h"
#include "ReNMCL.h"



void GaussianPredictStrategy::Predict(ReNMCL* renmcl, std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise)
{
	Eigen::Matrix3d cov;
	cov << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;
	std::vector<Eigen::Matrix3d> covariances{cov};
	Eigen::Vector3d mean = renmcl->o_stats.Mean();
	std::vector<Eigen::Vector3f> initGuesses{Eigen::Vector3f(mean(0), mean(1), mean(2))};

	for(int i = 0; i < renmcl->o_numParticles; ++i)
	{
		Eigen::Vector3f pose = renmcl->o_motionModel->SampleMotion(renmcl->o_particles[i].pose, u, odomWeights, noise);
		renmcl->o_particles[i].pose = pose;
		//particle pruning - if particle is outside the map, we replace it
		while (!renmcl->o_sensorModel->IsValid(renmcl->o_particles[i]))
		{
			Particle p = (renmcl->o_sensorModel->InitGaussian(1, initGuesses, covariances))[0];
			p.weight = 1.0 / renmcl->o_numParticles;
			renmcl->o_particles[i].pose = p.pose;
		}
	}
}