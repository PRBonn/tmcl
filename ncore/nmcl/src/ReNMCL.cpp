/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ReNMCL.cpp                    			                 #
# ##############################################################################
**/


#include "ReNMCL.h"
#include <numeric>
#include <functional> 
#include <iostream>


ReNMCL::ReNMCL(std::shared_ptr<FloorMap> fm, std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n, float injectionRatio)
{
	o_motionModel = mm;
	o_sensorModel = sm;
	o_resampler = rs;
	o_numParticles = n;
	o_injectionRatio = injectionRatio;
	o_particles = o_sensorModel->InitUniform(o_numParticles);
	o_floorMap = fm;
	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);
	o_predict = std::make_shared<UniformPredictStrategy>(UniformPredictStrategy());
}

ReNMCL::ReNMCL(std::shared_ptr<FloorMap> fm, std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n, 
			std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances,
			float injectionRatio)
{
	o_motionModel = mm;
	o_sensorModel = sm;
	o_resampler = rs;
	o_numParticles = n;
	o_injectionRatio = injectionRatio;
	o_particles = o_sensorModel->InitGaussian(o_numParticles, initGuess, covariances );
	o_floorMap = fm;
	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);
	o_predict = std::make_shared<UniformPredictStrategy>(UniformPredictStrategy());
}


void ReNMCL::Predict(std::vector<Eigen::Vector3f> u, std::vector<float> odomWeights, Eigen::Vector3f noise)
{
	o_predict->Predict(this, u, odomWeights, noise);
}



void ReNMCL::Correct(std::shared_ptr<SensorData> data)
{
	o_sensorModel->ComputeWeights(o_particles, data);

	normalizeWeights();
	o_resampler->Resample(o_particles);
	computeStatistics();
}

void ReNMCL::normalizeWeights()
{
	auto lambda = [&](double total, Particle p){return total + p.weight; };
	double sumWeights = std::accumulate(o_particles.begin(), o_particles.end(), 0.0, lambda);
	std::for_each(o_particles.begin(), o_particles.end(), [sumWeights](Particle &p){ p.weight /= sumWeights; });
}



void ReNMCL::computeStatistics()
{
	o_stats = SetStatistics::ComputeParticleSetStatistics(o_particles);
}

void ReNMCL::Recover()
{
	o_particles = o_sensorModel->InitUniform(o_numParticles);
}

void ReNMCL::Relocalize(const std::vector<cv::Rect>& boundingBoxes, const std::vector<float>& orientations, float camAngle)
{
	int numMatches = boundingBoxes.size();
	if (numMatches)
	{
		int numInject = int(o_numParticles * o_injectionRatio);
		int perInject = int(numInject) / numMatches;
		int numKeep = o_numParticles - perInject * numMatches;

		auto lambda = [](const Particle & a, const Particle & b) {return a.weight > b.weight; };
		std::sort(o_particles.begin(), o_particles.end(), lambda);

		o_particles.erase(o_particles.begin() + numKeep, o_particles.end());

		std::vector<Eigen::Vector2f> tl;
		std::vector<Eigen::Vector2f> br;
		std::vector<float> yaw;
		for(int i = 0; i < numMatches; ++i)
		{
			cv::Rect r = boundingBoxes[i];
			tl.push_back(Eigen::Vector2f(r.x, r.y));
			br.push_back(Eigen::Vector2f(r.x + r.width, r.y + r.height));
			yaw.push_back(orientations[i] - camAngle);
		}
		std::vector<Particle> injParticles = o_sensorModel->InitBoundingBox(perInject, tl, br, yaw );
		o_particles.insert( o_particles.end(), injParticles.begin(), injParticles.end());
	}
}
