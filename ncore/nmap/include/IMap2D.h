/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: IMap2D.h                                                              #
# ##############################################################################
**/

#ifndef IMAP2D_H
#define IMAP2D_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

class IMap2D
{
	public:


		//! Converts (x, y) from the map frame to the pixels coordinates
		/*!
			\param (x, y) position in map frame
		   \return (u, v) pixel coordinates for the gridmap
		*/
		virtual Eigen::Vector2f World2Map(Eigen::Vector2f xy) = 0;


		//! Converts a vector of (x, y) from the map frame to the pixels coordinates
		/*!
			\param vector of (x, y) positions in map frame
		   \return vector of (u, v) pixel coordinates for the gridmap
		*/
		virtual std::vector<Eigen::Vector2f> World2Map(std::vector<Eigen::Vector3f> xyPoints) = 0;
		

		//! Converts (u, v) pixel coordinates to map frame (x, y)
		/*!
			\param (u, v) pixel coordinates for the gridmap
		   \return (x, y) position in map frame
		*/
		virtual Eigen::Vector2f Map2World(Eigen::Vector2f uv) = 0;


		virtual ~IMap2D() {};

		
		virtual const cv::Mat& Map() const = 0;

	
};

#endif // IMap2D