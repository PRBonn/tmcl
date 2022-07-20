/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MixedFSR.cpp                			                               #
# ##############################################################################
**/

#include "MixedFSR.h"
#include <math.h>
#include <stdlib.h>
#include "Utils.h"
#include <iostream>



Eigen::Vector3f MixedFSR::SampleMotion(Eigen::Vector3f p1, std::vector<Eigen::Vector3f> command,  std::vector<float> weights, Eigen::Vector3f noise)
{
	Eigen::Vector3f u(0, 0, 0);
	float choose = drand48();

	float w = 0.0;

	for(long unsigned int i = 0; i < command.size(); ++i)
	{
		w += weights[i];
		if(choose <= w)
		{
			u = command[i];
			break;
		} 
	}

	float f = u(0);
	float s = u(1);
	float r = u(2);

	float f_h = f - SampleGuassian(noise(0) * fabs(f));
	float s_h = s - SampleGuassian(noise(1) * fabs(s));
	float r_h = r - SampleGuassian(noise(2) * fabs(r));


	Eigen::Vector3f new_p = Forward(p1, Eigen::Vector3f(f_h, s_h, r_h));

	return new_p;

}




