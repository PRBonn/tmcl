/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Object.cpp                                                            #
# ##############################################################################
**/

#include "Object.h"
#include <random>

Object::Object(int semLabel, Eigen::Vector3f pose, std::string modelPath)
{
	o_id = Object::generateID();
	o_semLabel = semLabel;
	o_pose = pose;
	o_modelPath = modelPath;
}

Object::Object()
{
	o_id = Object::generateID();
	o_pose = Eigen::Vector3f(0, 0, 0);
	o_semLabel = 0;
	o_modelPath = "";
}

int Object::generateID()
{
	//srand(); 
	int id = (rand()%10000)+1; 

	return id;
}

void Object::loadModel()
{
	
}


