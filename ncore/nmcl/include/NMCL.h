/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NMCL.h      	          				                               #
# ##############################################################################
**/

#ifndef NMCL_H
#define NMCL_H

#include "MotionModel.h"
#include "SensorModel.h"
#include "Resampling.h"
#include "SetStatistics.h"
#include <memory>
#include "SensorData.h"

class NMCL
{
	public:

		//! A constructor
	    /*!
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a SensorModel object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	    */
		NMCL(std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, std::shared_ptr<Resampling> rs, int n_particles);


		//! A constructor
	    /*!
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a SensorModel object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	      \param initGuess is a vector of initial guess for the location of the robots
	      \param covariances is a vector of covariances (uncertainties) corresponding to the initial guesses
	    */
		NMCL(std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, std::shared_ptr<Resampling> rs, int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances);


		//! A getter for the prediction of the current 2D pose in Map frame. Basically a weighted average of the particles
		/*!
		   \return Eigen::Vector3f = (x, y, theta)
		*/


		SetStatistics Stats()
		{
			return stats;
		}


		//! A getter particles representing the pose hypotheses 
		/*!
		   \return A vector of points, where each is Eigen::Vector3f = (x, y, theta)
		*/
		std::vector<Particle> Particles()
		{
			return particles;
		}


		//! Advanced all particles according to the control and noise, using the chosen MotionModel's forward function
		/*!
		  \param control is a 3d control command. In the FSR model it's (forward, sideways, rotation)
		  \param odomWeights is the corresponding weight to each odometry source
	      \param noise is the corresponding noise to each control component
		*/
		void Predict(std::vector<Eigen::Vector3f> control, std::vector<float> odomWeights, Eigen::Vector3f noise);

		//! Considers the likelihood of observation for all hypotheses, and then performs resampling 
		/*!
		  \param scan is a vector of homogeneous points (x, y, 1), in the sensor's frame. So the sensor location is (0, 0, 0)
		  		Notice that for the LaserScan messages you need to first transform the ranges to homo points, and then center them 
		*/
		void Correct(std::shared_ptr<SensorData> data);


		//! Initializes filter with new particles upon localization failure
		void Recover();

	private:

	
		void computeStatistics();
		void normalizeWeights();


		std::shared_ptr<MotionModel> motionModel;
		std::shared_ptr<SensorModel> sensorModel;
		std::shared_ptr<Resampling> resampler; 
		int n_particles = 0;
		std::vector<Particle> particles;
		Eigen::Vector3f prediction;
		SetStatistics stats;
};

#endif