/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Room.cpp                                                              #
# ##############################################################################
**/

#include "Room.h"


Room::Room(std::string name, int id, std::string purpose)
{
	o_name = name;
	o_purpose = purpose;
	o_id = id;
}

Object& Room::GetObject(int id)
{
	auto it = find_if(o_objects.begin(), o_objects.end(), [&id](const Object& obj) {return obj.ID() == id;});
	auto index = std::distance(o_objects.begin(), it);
	return o_objects[index];
}


int Room::AddObject(Object& obj)
{
	o_objects.push_back(obj);
	return 0;
}

int Room::RemoveObject(int id)
{
	auto it = find_if(o_objects.begin(), o_objects.end(), [&id](const Object& obj) {return obj.ID() == id;});
	if (it != o_objects.end())
	{
	  auto index = std::distance(o_objects.begin(), it);
	  o_objects.erase (o_objects.begin() + index);

	  return 0;
	}

	return -1;
}
