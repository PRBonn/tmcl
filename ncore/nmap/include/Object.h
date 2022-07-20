/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Object.h                                                              #
# ##############################################################################
**/


#ifndef OBJECT
#define OBJECT


#include <eigen3/Eigen/Dense>
#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#pragma once
class Object
{

public:


	Object(int semLabel, Eigen::Vector3f pose = Eigen::Vector3f(), std::string modelPath = "");

	Object();


	int ID() const
	{
		return o_id;
	}

	int SemLabel() const
	{
		return o_semLabel;
	}

	void SemLabel(int semLabel) 
	{
		o_semLabel = semLabel;
	}

	Eigen::Vector3f Pose() const
	{
		return o_pose;
	}

	void Pose(Eigen::Vector3f pose) 
	{
		o_pose = pose;
	}

	std::string ModelPath() const
	{
		return o_modelPath;
	}

	void ModelPath(std::string modelPath) 
	{
		o_modelPath = modelPath;
		//loadModel();
	}

private:

	friend class boost::serialization::access;
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & o_id;
        ar & o_semLabel;
        ar & o_pose;
        ar & o_modelPath;
    }

    static int generateID();
    void loadModel();

	int o_id;
	int o_semLabel;
	Eigen::Vector3f o_pose;
	std::string o_modelPath;
};


namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive & ar, Eigen::Vector3f & pose, const unsigned int version)
{
    ar & pose(0);
    ar & pose(1);
    ar & pose(2);
}

} // namespace serialization
} // namespace boost


#endif // !OBJECT


