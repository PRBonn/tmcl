/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FloorMap.cpp                                                          #
# ##############################################################################
**/

#include "FloorMap.h"
#include <string>
#include <nlohmann/json.hpp>
#include <fstream>
#include <boost/filesystem.hpp>

FloorMap::FloorMap(std::shared_ptr<GMap> map, cv::Mat& roomSeg, std::string name)
{
	o_map = map;
    o_name = name;
    
    cv::Mat split[3];
    cv::split(roomSeg, split);
    o_roomSeg = split[2];

    //std::cout << o_roomSeg << std::endl;

	extractRoomSegmentation();

    o_seeds = std::vector<Eigen::Vector3f>(o_rooms.size(), Eigen::Vector3f::Zero());
}

FloorMap::FloorMap(std::string jsonPath)
{
    using json = nlohmann::json;

    std::string folderPath = boost::filesystem::path(jsonPath).parent_path().string() + "/";

    std::ifstream file(jsonPath);
    json config;
    file >> config;

    o_name = config["name"];

    std::string segPath = config["roomSeg"];
    cv::Mat roomSeg = cv::imread(folderPath + segPath);
    cv::Mat split[3];
    cv::split(roomSeg, split);
    o_roomSeg = split[2];

    if(config["map"]["type"] == "GMap")
    {
        float resolution = config["map"]["resolution"];
        std::vector<float> origin = config["map"]["origin"];
        std::string imgPath = config["map"]["image"];
        cv::Mat img = cv::imread(folderPath + imgPath);
        o_map = std::make_shared<GMap>(GMap(img, Eigen::Vector3f(origin[0], origin[1], origin[2]), resolution));
    }

    extractRoomSegmentation();

    o_seeds = std::vector<Eigen::Vector3f>(o_rooms.size(), Eigen::Vector3f::Zero());

    std::string editorPath = config["editor"];
    if (boost::filesystem::exists(folderPath + editorPath))
    {
        std::ifstream ifs(folderPath + editorPath);
        if(ifs.peek() != std::ifstream::traits_type::eof())
        {
            boost::archive::text_iarchive ia(ifs);
            ia >> *this;
        }
        ifs.close(); 
    }
    else
    {
        std::ofstream ofs(folderPath + editorPath);
        ofs.close();
        std::cout << "no editor.xml file found. Creating empty one" << std::endl;
    }
}




int FloorMap::GetRoomID(float x, float y)
{
    uchar val = o_roomSeg.at<uchar>(y, x);
    return val;
}


void FloorMap::extractRoomSegmentation()
{
	std::vector<int> roomIDs = extractRoomIDs();

	for (long unsigned int i = 0; i < roomIDs.size(); ++i)
	{
		int id = roomIDs[i];
		o_rooms.push_back(Room(std::to_string(id), id));
	}
}



std::vector<int> FloorMap::extractRoomIDs()
{
    cv::Mat flat = o_roomSeg.reshape(1, o_roomSeg.total() * o_roomSeg.channels());
    std::vector<uchar> vec = o_roomSeg.isContinuous() ? flat : flat.clone();
    std::set<uchar> s( vec.begin(), vec.end() );
    vec.assign( s.begin(), s.end() );
    std::sort(vec.begin(), vec.end());

    std::vector<int> roomIDs;
    roomIDs.push_back(0);

    for(long unsigned int i = 1; i < vec.size(); ++i)
    {
       
        if (vec[i] == vec[i - 1] + 1)
        {
            roomIDs.push_back(i);
        }
        else break;
    }

    return roomIDs;
}

void FloorMap::findNeighbours()
{
    std::vector<int> roomIDs = extractRoomIDs();
    std::vector<cv::Rect> boundRect(roomIDs.size());
    std::vector<cv::RotatedRect> rotRect;

    cv::Mat orig = cv::Mat::zeros( o_roomSeg.size(), CV_8UC3 );

    for(long unsigned int r = 0; r < roomIDs.size(); ++r)
    {
        int roomID = roomIDs[r];
        cv::Mat dst;
        cv::threshold( o_roomSeg, dst, roomID, 255, 4 );
        cv::threshold( dst, dst, roomID - 1, 255, 0 );

        cv::Mat threshold_output;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( dst, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
        int bigContID = 0;
        double maxArea = 0;
        cv::RNG rng(12345);
        cv::Mat drawing = cv::Mat::zeros( dst.size(), CV_8UC3 );

        for( size_t i = 0; i < contours.size(); i++ )
        {
            double newArea = cv::contourArea(contours[i]);
            if (newArea > maxArea)
            {
                bigContID = i;
                maxArea = newArea;
            }
        }
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
     
        cv::RotatedRect box = cv::minAreaRect(contours[bigContID]); 
        cv::Point2f vertices[4];
        cv::Point2f center = box.center;
        box.points(vertices);

        std::vector<cv::Point> scaledVertices;
        float scale = 1.2;
        for(int w = 0; w < 4;  ++w)
        {
            cv::Point p = scale * (vertices[w] - center) + center;
            scaledVertices.push_back(p);
        }

        cv::RotatedRect scaledBox = cv::minAreaRect(scaledVertices);
        rotRect.push_back(scaledBox);

        // scaledBox.points(vertices);
        // for (int j = 0; j < 4; j++)
        // {
        //     cv::line(orig, vertices[j], vertices[(j+1)%4], color, 2);
        // }
    }

    //cv::imwrite("boxes.png", orig);
    o_neighbors = std::vector<std::vector<int>>(roomIDs.size());

    for(long unsigned int r = 0; r < roomIDs.size(); ++r)
    {
        cv::RotatedRect A = rotRect[r];
        for(long unsigned int s = 0; s < roomIDs.size(); ++s)
        {
            if (s == r) continue;
    
            cv::RotatedRect B = rotRect[s];
            std::vector<cv::Point2f> intersection;
            cv::rotatedRectangleIntersection(A, B, intersection);
            if (intersection.size())
            {
                o_neighbors[r].push_back(int(s));
            }
        }
    }
}


cv::Mat FloorMap::ColorizeRoomSeg()
{
    cv::RNG rng(12345);
    cv::Mat roomSegRGB;
    cv::cvtColor(o_roomSeg, roomSegRGB, cv::COLOR_GRAY2BGR);


    int maxElm = o_rooms.size();
    std::vector<cv::Scalar> colors;
    for(int i = 1; i <= maxElm; ++i)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        colors.push_back(color);
    }

    for(int i = 0; i < o_roomSeg.rows; i++)
    {
        for(int j = 0; j < o_roomSeg.cols; j++)
        {
            uchar val = o_roomSeg.at<uchar>(i, j);
            if ((val >= 1) && (val <= maxElm))
            {
                cv::Scalar color = colors[val - 1];
                roomSegRGB.at<cv::Vec3b>(i, j) = cv::Vec3b(color[0], color[1], color[2]);
            }
        }
    }

    return roomSegRGB;
}


std::vector<std::string> FloorMap::GetRoomNames()
{
    std::vector<std::string> places;
    for(int i = 0; i < o_rooms.size(); ++i)
    {
        places.push_back(o_rooms[i].Name());
    }
    return places;
}


void FloorMap::Seed(int id, const Eigen::Vector2f& seed)
{
    Eigen::Vector2f p = o_map->Map2World(seed);
    o_seeds[id] = Eigen::Vector3f(p(0), p(1), 0);
}