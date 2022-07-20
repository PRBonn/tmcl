/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: Room.h                                                                #
# ##############################################################################
**/

#ifndef ROOM
#define ROOM

#pragma once

#include <string>
#include <vector>
#include "Object.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


class Room
{
public:

	Room(std::string name, int id, std::string purpose = "general");

	Room() {};

	std::string Name() const
	{
		return o_name;
	}


	void Name(std::string name) 
	{
		o_name = name;
	}

	std::string Purpose() const
	{
		return o_purpose;
	}

	int ID() const
	{
		return o_id;
	}

	const std::vector<Object> Objects() const
	{
		return o_objects;
	}

	Object& GetObject(int id);
	

	int AddObject(Object& obj);

	//NOT IMPLEMENTED YET!!!!
	int RemoveObject(int id);

private:


	friend class boost::serialization::access;
	template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & o_name;
        ar & o_purpose;
        ar & o_id;
        ar & o_objects;
    }


	std::string o_name = "room";
	std::string o_purpose = "general";
	int o_id = -1;
	std::vector<Object> o_objects;

};

#endif // ROOM
