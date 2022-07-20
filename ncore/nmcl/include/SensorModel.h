/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: SensorModel.h          			                           		   #
# ##############################################################################
**/


#ifndef SENSORMODEL_H
#define SENSORMODEL_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include "Particle.h"
#include <SensorData.h>
#include <memory>


class SensorModel
{

	public: 

		virtual void ComputeWeights(std::vector<Particle>& particles, std::shared_ptr<SensorData> data) = 0;
		virtual bool IsOccupied(Particle particle) = 0;
		virtual bool IsValid(Particle particle) = 0;
		virtual std::vector<Particle> InitUniform(int n_particles) = 0;
		virtual std::vector<Particle> InitBoundingBox(int n_particles, std::vector<Eigen::Vector2f> tl, std::vector<Eigen::Vector2f> br, std::vector<float> yaw) = 0;
		virtual std::vector<Particle> InitGaussian(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances) = 0;
		virtual std::vector<Particle> InitOriented(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances) = 0;

		virtual ~SensorModel(){};

};

#endif