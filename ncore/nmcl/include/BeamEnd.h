/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: BeamEnd.h                      		                               #
# ##############################################################################
**/


#ifndef BEAMEND_H
#define BEAMEND_H

#include <memory>
#include <string>

#include "SensorModel.h"
#include "GMap.h"
#include "SensorData.h"


class BeamEnd: public SensorModel
{
	public:

		enum class Weighting 
		{   
			NAIVE = 0, 
		    INTEGRATION = 1, 
		    LAPLACE = 2,
		    GEOMETRIC = 3,
		    GPOE = 4
		};


		//! A constructor
	    /*!
	      \param Gmap is a ptr to a GMAP object, which holds the gmapping map
	      \param sigma is a float that determine how forgiving the model is (small sigma will give a very peaked likelihood)
	      \param maxRange is a float specifying up to what distance from the sensor a reading is valid
	      \param Weighting is an int specifying which weighting scheme to use
	    */

		BeamEnd(std::shared_ptr<GMap> Gmap, float sigma = 8, float maxRange = 15, Weighting weighting = Weighting::LAPLACE);

		//! Computes weights for all particles based on how well the observation matches the map
		/*!
		  \param particles is a vector of Particle elements
		  \param SensorData is an abstract container for sensor data. This function expects LidarData type
		*/

		void ComputeWeights(std::vector<Particle>& particles, std::shared_ptr<SensorData> data);
		
		//! Returns truth if a particle is in an occupied grid cell, false otherwise. Notice that for particles in unknown areas the return is false.
		/*!
		  \param Particle is a particle with pose and weight
		*/

		bool IsOccupied(Particle particle);

		//! Returns truth if a particle is in valid pose, false otherwise. Valid pose means it is in a grid cell which is free
		/*!
		  \param Particle is a particle with pose and weight
		*/

		bool IsValid(Particle particle);

		//! Spawns the desired number of new particles, only in areas of the map that are free, in a uniform distribution 
		/*!
		  \param n_particles is an int specifying the number of particles to spawn
		*/
		
		std::vector<Particle> InitUniform(int n_particles);

		//! Spawns the desired number of new particles, in a Guassian distribution around specific locations
		/*!
		  \param n_particles is an int specifying the number of particles to spawn
		  \param initGuess is a vector of poses, specifying locations that will be use as the means of each Guassian distribution 
		  \param covariances is a vector of matrices describing the spread of the Guassian distribution  
		*/

		std::vector<Particle> InitGaussian(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances);

		std::vector<Particle> InitOriented(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances);

 		std::vector<Particle> InitBoundingBox(int n_particles, std::vector<Eigen::Vector2f> tl, std::vector<Eigen::Vector2f> br, std::vector<float> yaw);


		void plotParticles(std::vector<Particle>& particles, std::string title, bool show=true); 


	private:	

		std::vector<float> getLikelihood(const std::vector<float>& distances);

		float getLikelihood(float distance);

		void plotScan(Eigen::Vector3f laser, std::vector<Eigen::Vector2f>& zMap); 

		std::vector<Eigen::Vector2f> scan2Map(Eigen::Vector3f pose, const std::vector<Eigen::Vector3f>& scan);

		double naive(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);

		double geometric(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);

		double gPoE(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);


		double integration(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);

		double laplace(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask);

		bool isValid(Eigen::Vector2f uv);



		std::shared_ptr<GMap> Gmap;
		float maxRange = 15;
		float sigma = 8;
		cv::Mat edt;
		Weighting o_weighting;

};

#endif