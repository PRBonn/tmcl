/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                            		   #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                     				   #
#                                                                              #
#  File: MSensorsUnitTests.cpp                                                            #
# ##############################################################################
**/

#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "GMap.h"
#include "Utils.h"
#include "Object.h"
#include <fstream>
#include "Room.h"
#include "FloorMap.h"


std::string dataPath = PROJECT_TEST_DATA_DIR + std::string("/8/");
std::string dataPathFloor = PROJECT_TEST_DATA_DIR + std::string("/floor/test/");
std::string configPath = PROJECT_TEST_DATA_DIR + std::string("/config/");
std::string faroPath = PROJECT_TEST_DATA_DIR + std::string("/floor/Faro/");



TEST(TestGMap, test1) {
    
    Eigen::Vector3f origin = Eigen::Vector3f(-12.200000, -17.000000, 0.000000);
	float resolution = 0.05;
	cv::Mat gridMap = cv::imread(dataPath + "YouBotMap.pgm");
	GMap gmap = GMap(gridMap, origin, resolution);

	Eigen::Vector2f o = gmap.World2Map(Eigen::Vector2f(0, 0));
	ASSERT_EQ(o, Eigen::Vector2f(244, 332));
}

TEST(TestGMap, test2) {

	Eigen::Vector3f origin = Eigen::Vector3f(-12.200000, -17.000000, 0.000000);
	float resolution = 0.05;
	cv::Mat gridMap = cv::imread(dataPath + "YouBotMap.pgm");
	GMap gmap = GMap(gridMap, origin, resolution);

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
	ASSERT_EQ(p, Eigen::Vector2f(0, 0));
}


TEST(TestGMap, test3) {

	std::string mapFolder = dataPath;
	GMap gmap = GMap(mapFolder);

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
	ASSERT_EQ(p, Eigen::Vector2f(0, 0));
}

TEST(TestGMap, test4) {

   std::string mapFolder = dataPath;
    GMap gmap = GMap(mapFolder);

    Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
    Eigen::Vector2f pm = gmap.World2Map(p);

    ASSERT_EQ(pm(0), 244);
    ASSERT_EQ(pm(1), 332);
}


TEST(TestObject, test1) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);
    int id = ob.ID();


    std::string filename = "object.xml";
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << ob;
    ofs.close();


    Object ob2(6);
    std::ifstream ifs(filename.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> ob2;
    ifs.close();

    int id2 = ob2.ID();
    int semLabel2 = ob2.SemLabel();
    Eigen::Vector3f pose2 = ob2.Pose();
    std::string modelPath2 = ob2.ModelPath();


    ASSERT_EQ(id, id2);
    ASSERT_EQ(semLabel, semLabel2);
    ASSERT_EQ(pose, pose2);
    ASSERT_EQ(modelPath, modelPath2);

}


TEST(TestRoom, test1) {
    
    std::string name = "room";
    int roomID = 9;
    Room r(name, roomID);


    std::string filename = "room.xml";
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << r;
    ofs.close();

    Room r2("room2", 11);
    std::ifstream ifs(filename.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> r2;
    ifs.close();

    ASSERT_EQ(roomID, r2.ID());
    ASSERT_EQ(name, r2.Name());

}


TEST(TestRoom, test2) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);
    int id = ob.ID();

    int semLabel2 = 7;
    Eigen::Vector3f pose2(1.3, -10.0, -2.7);
    std::string modelPath2 = "/nicky/poo2.pcl";
    Object ob2(semLabel2, pose2, modelPath2);
  
    std::string name = "room";
    int roomID = 9;
    Room r(name, roomID);
    r.AddObject(ob);
    r.AddObject(ob2);


    std::string filename = "room.xml";
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << r;
    ofs.close();

    Room r2("room2", 11);
    std::ifstream ifs(filename.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> r2;
    ifs.close();

    std::vector<Object> objects = r2.Objects();

    ASSERT_EQ(roomID, r2.ID());
    ASSERT_EQ(name, r2.Name());

    ASSERT_EQ(id, objects[0].ID());
    ASSERT_EQ(semLabel2, objects[1].SemLabel());

}

TEST(TestRoom, test3) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);

    int semLabel2 = 7;
    Eigen::Vector3f pose2(1.3, -10.0, -2.7);
    std::string modelPath2 = "/nicky/poo2.pcl";
    Object ob2(semLabel2, pose2, modelPath2);

    std::string name = "room";
    int roomID = 9;
    Room r(name, roomID);
    r.AddObject(ob);
    r.AddObject(ob2);

    int size = r.Objects().size();
    ASSERT_EQ(2, size);
}

TEST(TestRoom, test4) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);
    int id = ob.ID();

    int semLabel2 = 7;
    Eigen::Vector3f pose2(1.3, -10.0, -2.7);
    std::string modelPath2 = "/nicky/poo2.pcl";
    Object ob2(semLabel2, pose2, modelPath2);
    int id2 = ob2.ID();


    std::string name = "room";
    int roomID = 9;
    Room r(name, roomID);
    r.AddObject(ob);
    r.AddObject(ob2);


    r.RemoveObject(id);
    int size = r.Objects().size();

    ASSERT_EQ(1, size);
    ASSERT_EQ(id2, r.Objects()[0].ID());
}

TEST(TestRoom, test5) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);

    int semLabel2 = 7;
    Eigen::Vector3f pose2(1.3, -10.0, -2.7);
    std::string modelPath2 = "/nicky/poo2.pcl";
    Object ob2(semLabel2, pose2, modelPath2);
 
    std::string name = "room";
    int roomID = 9;
    Room r(name, roomID);
    r.AddObject(ob);
    r.AddObject(ob2);

    int badID = 0;


    int res = r.RemoveObject(badID);
    int size = r.Objects().size();

    ASSERT_EQ(2, size);
    ASSERT_EQ(-1, res);
}




TEST(TestFloorMap, test1) {
    
    int semLabel = 10;
    Eigen::Vector3f pose(1.5, 0.0, 2.7);
    std::string modelPath = "/nicky/poo.pcl";
    Object ob(semLabel, pose, modelPath);
    int id = ob.ID();

    int semLabel2 = 7;
    Eigen::Vector3f pose2(1.3, -10.0, -2.7);
    std::string modelPath2 = "/nicky/poo2.pcl";
    Object ob2(semLabel2, pose2, modelPath2);

    std::string mapFolder = dataPathFloor;
    GMap gmap = GMap(mapFolder);

    cv::Mat roomSeg = cv::imread(dataPathFloor + "YouBotMapRoomSeg.png");
    FloorMap fp = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    
    fp.GetRoom(1).AddObject(ob);
    fp.GetRoom(1).Name("room");

    fp.GetRoom(2).AddObject(ob2);
    fp.GetRoom(2).Name("room2");

    std::string filename = "floor.xml";
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << fp;
    ofs.close();


    FloorMap fp2 = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    std::ifstream ifs(filename.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> fp2;
    ifs.close();

    std::vector<Object> objects = fp.GetRoom(1).Objects();
    std::vector<Room> rooms = fp.GetRooms();

    ASSERT_EQ(13, rooms.size());
    ASSERT_EQ("room2", fp.GetRoom(2).Name());

    ASSERT_EQ(id, objects[0].ID());

}

TEST(TestFloorMap, test2) {
    
    std::string mapFolder = dataPathFloor;
    GMap gmap = GMap(mapFolder);

    cv::Mat roomSeg = cv::imread(dataPathFloor + "YouBotMapRoomSeg.png");
    FloorMap fp = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    
    float x = 200;
    float y = 340;
    int id = fp.GetRoomID(x, y);

    ASSERT_EQ(1, id);

}


TEST(TestFloorMap, test3) {
    

    std::string mapFolder = dataPathFloor;
    GMap gmap = GMap(mapFolder);

    cv::Mat roomSeg = cv::imread(dataPathFloor + "YouBotMapRoomSeg.png");
    FloorMap fp = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    
    fp.GetRoom(1).Name("room");
    fp.GetRoom(2).Name("room2");

    ASSERT_EQ("room2", fp.GetRoom(2).Name());

}

TEST(TestFloorMap, test4) {
    
    std::string mapFolder = dataPathFloor;
    GMap gmap = GMap(mapFolder);

    cv::Mat roomSeg = cv::imread(dataPathFloor + "YouBotMapRoomSeg.png");
    FloorMap fp = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    fp.findNeighbours();

    std::vector<std::vector<int>> neighbours = fp.Neighbors();
    //std::cout << fp.Neighbors().size() << std::endl;
    // for (int i = 0; i < neighbours.size(); ++i)
    // {
    //     std::cout << "Room " << i + 1 << " neighbours:" << std::endl;
    //     for(int j = 0; j < neighbours[i].size(); ++j)
    //     {
    //         std::cout << neighbours[i][j] + 1 << ", " << std::endl;
    //     }
    //     std::cout <<  std::endl;
    // }

    ASSERT_EQ(13, fp.Neighbors().size());
    ASSERT_EQ(2, fp.Neighbors()[0][0] + 1);

}

TEST(TestFloorMap, test5) {
    
    std::string mapFolder = dataPathFloor;
    GMap gmap = GMap(mapFolder);

    cv::Mat roomSeg = cv::imread(dataPathFloor + "YouBotMapRoomSeg.png");
    FloorMap fp = FloorMap(std::make_shared<GMap>(gmap), roomSeg);

    std::vector<Eigen::Vector2f> mapSeeds{Eigen::Vector2f(0, 0), Eigen::Vector2f(340, 400), Eigen::Vector2f(360, 470), Eigen::Vector2f(380, 650),
        Eigen::Vector2f(440, 650), Eigen::Vector2f(400, 700), Eigen::Vector2f(500, 750), Eigen::Vector2f(460, 660),
        Eigen::Vector2f(430, 600), Eigen::Vector2f(400, 520), Eigen::Vector2f(330, 340), Eigen::Vector2f(4000, 500),
    Eigen::Vector2f(480, 750)};
    std::vector<Eigen::Vector3f> seeds;
    for(long unsigned int i = 0; i < mapSeeds.size(); ++i)
    {
        Eigen::Vector2f p = gmap.Map2World(mapSeeds[i]);
        seeds.push_back(Eigen::Vector3f(p(0), p(1), 0));
       // std::cout << p(0) << ", " << p(1)<< std::endl;
    }

    fp.Seeds(seeds);

    std::string filename = dataPathFloor + "floor_seeds.xml";
    std::ofstream ofs(filename.c_str());
    boost::archive::text_oarchive oa(ofs);
    oa << fp;
    ofs.close();


    FloorMap fp2 = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
    std::ifstream ifs(filename.c_str());
    boost::archive::text_iarchive ia(ifs);
    ia >> fp2;
    ifs.close();

    std::vector<Eigen::Vector3f> loadedSeeds = fp2.Seeds();
    ASSERT_EQ(loadedSeeds[0](0), seeds[0](0));
    ASSERT_EQ(loadedSeeds[0](1), seeds[0](1));
    ASSERT_EQ(loadedSeeds[0](2), seeds[0](2));
}

TEST(TestFloorMap, test6) {
    
    std::string jsonPath = dataPathFloor + "floor.config";
    FloorMap fp = FloorMap(jsonPath);

    fp.findNeighbours();

    std::vector<std::vector<int>> neighbours = fp.Neighbors();

    ASSERT_EQ(13, fp.Neighbors().size());
    ASSERT_EQ(2, fp.Neighbors()[0][0] + 1);

}

TEST(TestFloorMap, test7) {
    
    std::string jsonPath = faroPath + "floor.config";
    FloorMap fp = FloorMap(jsonPath);
   
    // cv::Mat mat = fp.ColorizeRoomSeg();
    // cv::imshow("", mat);
    // cv::waitKey();



    fp.findNeighbours();
    std::vector<std::vector<int>> neighbours = fp.Neighbors();

    ASSERT_EQ(13, fp.Neighbors().size());
    ASSERT_EQ(2, fp.Neighbors()[0][0] + 1);

}




int main(int argc, char **argv) {
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}



