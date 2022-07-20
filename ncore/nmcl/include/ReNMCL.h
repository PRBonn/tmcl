/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ReNMCL.h      	          				          #
# ##############################################################################
**/

#ifndef RENMCL_H
#define RENMCL_H

#include "MotionModel.h"
#include "SensorModel.h"
#include "Resampling.h"
#include "SetStatistics.h"
#include "FloorMap.h"
#include "PlaceRecognition.h"
#include <memory>
#include "PredictStrategy.h"
#include "UniformPredictStrategy.h"
#include "GaussianPredictStrategy.h"
#include "SensorData.h"

class ReNMCL
{
	public:

		friend class PredictStrategy;
		friend class UniformPredictStrategy;
		friend class GaussianPredictStrategy;

		//! A constructor
	    /*!
	     \param fm is a ptr to a FloorMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a SensorModel object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	      \param injectionRatio is an float, and it determines which portions of the particles are replaced when relocalizing
	    */
		ReNMCL(std::shared_ptr<FloorMap> fm, std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n_particles, float injectionRatio = 0.2);


		//! A constructor
	    /*!
	     * \param fm is a ptr to a FloorMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a SensorModel object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	      \param initGuess is a vector of initial guess for the location of the robots
	      \param covariances is a vector of covariances (uncertainties) corresponding to the initial guesses
	      \param injectionRatio is an float, and it determines which portions of the particles are replaced when relocalizing
	    */
		ReNMCL(std::shared_ptr<FloorMap> fm, std::shared_ptr<MotionModel> mm, std::shared_ptr<SensorModel> sm, 
			std::shared_ptr<Resampling> rs, int n_particles, std::vector<Eigen::Vector3f> initGuess, 
			std::vector<Eigen::Matrix3d> covariances, float injectionRatio = 0.2);


		SetStatistics Stats()
		{
			return o_stats;
		}


		//! A getter particles representing the pose hypotheses 
		/*!
		   \return A vector of points, where each is Eigen::Vector3f = (x, y, theta)
		*/
		std::vector<Particle> Particles()
		{
			return o_particles;
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


		// Replaces some portion of the particles if we recognized being in a certain place in the map

		void Relocalize(const std::vector<cv::Rect>& boundingBoxes, const std::vector<float>& orientations, float camAngle);


		//! Initializes filter with new particles upon localization failure
		void Recover();

		Eigen::Vector3f Backward(Eigen::Vector3f p1, Eigen::Vector3f p2)
		{
			return o_motionModel->Backward(p1, p2);
		}

		void SetInjRation(float ratio)
		{
			o_injectionRatio = ratio;
		}

		const std::shared_ptr<FloorMap>& GetFloorMap()
		{
			return o_floorMap;
		}


		void SetPredictStrategy(std::shared_ptr<PredictStrategy> strategy)
		{
			o_predict = strategy;
		}

		std::shared_ptr<PredictStrategy> GetPredictStrategy()
		{
			return o_predict;
		}

	private:

	
		void computeStatistics();
		void normalizeWeights();

		std::shared_ptr<PredictStrategy> o_predict;

		std::shared_ptr<MotionModel> o_motionModel;
		std::shared_ptr<SensorModel> o_sensorModel;
		std::shared_ptr<Resampling> o_resampler; 
		std::shared_ptr<FloorMap> o_floorMap;
		int o_numParticles = 0;
		std::vector<Particle> o_particles;
		SetStatistics o_stats;
		float o_injectionRatio = 0.5;

};

#endif
