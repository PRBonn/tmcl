/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: PlaceRecognition.h               		                               #
# ##############################################################################
**/


#ifndef PLACERECOGNITION_H
#define PLACERECOGNITION_H

#include <memory>
#include <string>
#include <vector>

class PlaceRecognition
{
public:
	PlaceRecognition(const std::vector<std::string>& dict);

	std::vector<int> Match(const std::vector<std::string>& places);

private:

	std::vector<std::string> divideWord(const std::string& word, const char delim = ' ');

	std::vector<std::string> o_dict;
	std::vector<std::vector<std::string>> o_extDict;

};




#endif //PLACERECOGNITION