/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FSR.cpp                                                               #
# ##############################################################################
**/


#include "FSR.h"
#include <math.h>
#include <stdlib.h>
#include "Utils.h"
#include <iostream>


Eigen::Vector3f FSR::Backward(Eigen::Vector3f p1, Eigen::Vector3f p2)
{
	float dx = p2(0) - p1(0);
	float dy = p2(1) - p1(1);
	float dtheta = p2(2) - p1(2);

	float a = cos(p1(2));
	float b = sin(p1(2));

	float f = (dx * a + dy * b) / (pow(a, 2.0) + pow(b, 2.0));
    float s = (dy - f * b) / a;
    float r = dtheta;
	
	return Eigen::Vector3f(f, s, r);

}

Eigen::Vector3f FSR::Forward(Eigen::Vector3f p1, Eigen::Vector3f u)
{
	float f = u(0);
	float s = u(1);
	float r = u(2);

	float x = p1(0) + f * cos(p1(2)) - s * sin(p1(2));
	float y = p1(1) + f * sin(p1(2)) + s * cos(p1(2));
	float theta = Wrap2Pi(r + p1(2));
	// write code to make theta between 0 and 2pi

	return Eigen::Vector3f(x, y, theta);

}

Eigen::Vector3f FSR::SampleMotion(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> command,  std::vector<float> weights, Eigen::Vector3f noise)
{
	Eigen::Vector3f u = command[0];
	float f = u(0);
	float s = u(1);
	float r = u(2);

	float f_h = f - SampleGuassian(noise(0) * fabs(f));
	float s_h = s - SampleGuassian(noise(1) * fabs(s));
	float r_h = r - SampleGuassian(noise(2) * fabs(r));

	//std::cout << "FSR" << std::endl;


	Eigen::Vector3f new_p = Forward(p1, Eigen::Vector3f(f_h, s_h, r_h));

	return new_p;

}

