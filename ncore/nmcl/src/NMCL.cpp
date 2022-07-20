/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NMCL.cpp                    			                               #
# ##############################################################################
**/


#include "NMCL.h"
#include <numeric>
#include <functional> 
#include <iostream>

NMCL::NMCL(std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n)
{
	motionModel = mm;
	sensorModel = sm;
	resampler = rs;
	n_particles = n;
	particles = sensorModel->InitUniform(n_particles);
	stats = SetStatistics::ComputeParticleSetStatistics(particles);

}

NMCL::NMCL(std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n, 
			std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances)
{
	motionModel = mm;
	sensorModel = sm;
	resampler = rs;
	n_particles = n;
	particles = sensorModel->InitGaussian(n_particles, initGuess, covariances );
	stats = SetStatistics::ComputeParticleSetStatistics(particles);

}


void NMCL::Predict(std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise)
{
	/*Eigen::Matrix3d cov;
	cov << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
	std::vector<Eigen::Matrix3d> covariances{cov};
	Eigen::Vector3d mean = stats.Mean();
	std::vector<Eigen::Vector3f> initGuesses{Eigen::Vector3f(mean(0), mean(1), mean(2))};*/

	for(int i = 0; i < n_particles; ++i)
	{
		Eigen::Vector3f pose = motionModel->SampleMotion(particles[i].pose, u, odomWeights, noise);
		particles[i].pose = pose;
		//particle pruning - if particle is on an occupied grid cell, we replace it
		//if (sensorModel->IsOccupied(particles[i]))
		//particle pruning - if particle is outside the map, we replace it
		while (!sensorModel->IsValid(particles[i]))
		{
			//Particle p = (sensorModel->InitGaussian(1, initGuesses, covariances))[0];
			Particle p = (sensorModel->InitUniform(1))[0];
			p.weight = 1.0 / n_particles;
			particles[i].pose = p.pose;
		}
	}
}


void NMCL::Correct(std::shared_ptr<SensorData> data)
{
	sensorModel->ComputeWeights(particles, data);

	normalizeWeights();
	resampler->Resample(particles);
	//normalizeWeights();
	computeStatistics();
}

void NMCL::normalizeWeights()
{
	auto lambda = [&](double total, Particle p){return total + p.weight; };
	double sumWeights = std::accumulate(particles.begin(), particles.end(), 0.0, lambda);
	std::for_each(particles.begin(), particles.end(), [sumWeights](Particle &p){ p.weight /= sumWeights; });
}



void NMCL::computeStatistics()
{
	stats = SetStatistics::ComputeParticleSetStatistics(particles);
}

void NMCL::Recover()
{
	particles = sensorModel->InitUniform(n_particles);
}
