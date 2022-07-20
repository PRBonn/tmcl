/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NMCLUnitTests.cpp             		                           		   #
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

#include "FSR.h"
#include "Utils.h"
#include "Lidar2D.h"
#include "OptiTrack.h"

#include "BeamEnd.h"
#include "NMCL.h"

#include "MixedFSR.h"
#include "SetStatistics.h"

#include "LowVarianceResampling.h"
#include "Camera.h"
#include "PlaceRecognition.h"
#include "FloorMap.h"
#include "ReNMCL.h"
#include <nlohmann/json.hpp>
#include "NMCLFactory.h"
#include "LidarData.h"

std::string dataPath = PROJECT_TEST_DATA_DIR + std::string("/8/");
std::string floorPath = PROJECT_TEST_DATA_DIR + std::string("/floor/test/");



TEST(TestFSR, test1) {
    
    Eigen::Vector3f p1 = Eigen::Vector3f(1.2, -2.5, 0.67);
	Eigen::Vector3f p2 = Eigen::Vector3f(-1.5, 4.3, -1.03);

	FSR fsr = FSR();
	Eigen::Vector3f u = fsr.Backward(p1, p2);
	Eigen::Vector3f p2_comp = fsr.Forward(p1, u);
	ASSERT_EQ(p2_comp, p2);
}

TEST(TestFSR, test2) {

	FSR fsr = FSR();
	//test against know python result
	Eigen::Vector3f u2_gt = Eigen::Vector3f(-13.755235632332337, -3.971585665576746, 0.62);
	Eigen::Vector3f u2 = fsr.Backward(Eigen::Vector3f(13.6, -6.1, -0.33), Eigen::Vector3f(-0.7, -5.4, 0.29));
	ASSERT_NEAR(u2_gt(0), u2(0), 0.001);
	ASSERT_NEAR(u2_gt(1), u2(1), 0.001);
	ASSERT_NEAR(u2_gt(2), u2(2), 0.001);
	
	std::vector<float> commandWeights{1.0f};
	std::vector<Eigen::Vector3f> command{Eigen::Vector3f(0.05, -0.1, 0.02)};
	Eigen::Vector3f noisy_p = fsr.SampleMotion(Eigen::Vector3f(13.6, -6.1, -0.33), command, commandWeights, Eigen::Vector3f(0.1, 0.1, 0.1));
}

TEST(TestMixedFSR, test1) {

	MixedFSR mfsr = MixedFSR();
	//float choose  = drand48();
	//test against know python result
	Eigen::Vector3f p_gt = Eigen::Vector3f(13.61489781, -6.21080639, -0.31);
	std::vector<float> commandWeights{0.5f, 0.5f};

	std::vector<Eigen::Vector3f> command{Eigen::Vector3f(0.05, -0.1, 0.02), Eigen::Vector3f(0.05, -0.1, 0.02)};
	Eigen::Vector3f noisy_p = mfsr.SampleMotion(Eigen::Vector3f(13.6, -6.1, -0.33), command, commandWeights, Eigen::Vector3f(0.0, 0.0, 0.0));
	ASSERT_NEAR(noisy_p(0), p_gt(0), 0.001);
	ASSERT_NEAR(noisy_p(1), p_gt(1), 0.001);
	ASSERT_NEAR(noisy_p(2), p_gt(2), 0.001);
	
}


TEST(TestBeamEnd, test1)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap), 8, 15, BeamEnd::Weighting(0));

	Eigen::Vector3f p3d0_gt = Eigen::Vector3f(0.33675906, -0.84122932,  1. );
	std::vector<Eigen::Vector3f> scan{p3d0_gt};
	Particle p(Eigen::Vector3f(0, 0, 0), 1.0);
	std::vector<Particle> particles{p};
	std::vector<double> scanMask(1, 1.0);

	LidarData data = LidarData(scan, scanMask);

	be.ComputeWeights(particles, std::make_shared<LidarData>(data));

	//ASSERT_EQ(weights.size(), 1);

	// from python verified code weight should be 0.09063308, but the EDT is different therefore we expect some variance
	// Also sigma valued of BeamEnd was 8
	ASSERT_NEAR(particles[0].weight, 0.09063308, 0.01);
}


TEST(TestBeamEnd, test2)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap));
	int n = 10;

	std::vector<Eigen::Vector3f> initGuesses{Eigen::Vector3f(0.1, 0.1 , 0), Eigen::Vector3f(-0.1, -0.1, 0)};
	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero(3, 3);
	std::vector<Eigen::Matrix3d> covariances{cov, cov};

	std::vector<Particle> particles = be.InitGaussian(n, initGuesses, covariances);
	ASSERT_EQ(particles.size(), n * initGuesses.size());
	ASSERT_EQ(particles[0].pose, Eigen::Vector3f(0.1, 0.1 , 0));
	ASSERT_EQ(particles[n].pose, Eigen::Vector3f(-0.1, -0.1, 0));
	//for(int i = 0; i < particles.size(); ++i) std::cout << particles[i] << std::endl;

}

TEST(TestBeamEnd, test3)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap));
	int n = 10;

	Eigen::Vector3f v1(0.05, 0.05 , 0);
	Eigen::Vector3f v2(-0.1, -0.1, 0);


	std::vector<Eigen::Vector3f> initGuesses{v1, v2};
	Eigen::Matrix3d cov;
	cov << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
	std::vector<Eigen::Matrix3d> covariances{cov, cov};

	std::vector<Particle> particles = be.InitGaussian(n, initGuesses, covariances);

	Eigen::Vector3f avg1 = Eigen::Vector3f::Zero();
	Eigen::Vector3f avg2 = Eigen::Vector3f::Zero();
	for(int i = 0; i < n; ++i) avg1 += particles[i].pose;

	for(int i = n; i < 2 * n; ++i) avg2 += particles[i].pose;

	avg1 = avg1 / n;
	avg2 = avg2 / n;

	// std::cout << avg1 << std::endl;
	// std::cout << avg2 << std::endl;

	ASSERT_NEAR(avg1(0), v1(0) , 0.1);
	ASSERT_NEAR(avg1(1), v1(1) , 0.1);
	ASSERT_NEAR(avg1(2), v1(2), 0.1);
	ASSERT_NEAR(avg2(0), v2(0) , 0.1);
	ASSERT_NEAR(avg2(1), v2(1) , 0.1);
	ASSERT_NEAR(avg2(2), v2(2), 0.1);
	
}

TEST(TestBeamEnd, test4)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap));

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(244, 332));
	Particle pr(Eigen::Vector3f(p(0), p(1), 0), 0.5);
	bool valid = be.IsValid(pr);

	ASSERT_EQ(true, valid);
}

TEST(TestBeamEnd, test5)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap));

	Eigen::Vector2f p = gmap.Map2World(Eigen::Vector2f(0, 0));
	Particle pr(Eigen::Vector3f(p(0), p(1), 0), 0.5);
	bool valid = be.IsValid(pr);

	ASSERT_EQ(false, valid);
}

TEST(TestBeamEnd, test6)
{
	GMap gmap = GMap(dataPath);
	BeamEnd be = BeamEnd(std::make_shared<GMap>(gmap));
	Eigen::Matrix3d cov;
	cov << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.0;

	std::vector<Eigen::Matrix3d> covariances = {cov};


	for(int a = 1; a < 2; ++a)
	{
		std::ofstream csvFile;
	    csvFile.open(std::to_string(90 * a) + "particles.csv", std::ofstream::out);
    	std::vector<Eigen::Vector3f> seeds = {Eigen::Vector3f(0,0, -M_PI/2 * a)};
   		std::vector<Particle> particles = be.InitOriented(100, seeds, covariances);

		for(long unsigned int i = 0; i < particles.size(); ++i)
		{
			Eigen::Vector3f p = particles[i].pose;
			csvFile << p(0) << "," << p(1) << "," << p(2) << std::endl;
		}	
		csvFile.close();
	}
}




TEST(TestSetStatistics, test1)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1,1,1), Eigen::Vector3f(1,1,1)};
	std::vector<double> weights {0.5, 0.5};
	Particle p1(Eigen::Vector3f(1,1,1), 0.5);
	Particle p2(Eigen::Vector3f(1,1,1), 0.5);
	std::vector<Particle> particles{p1, p2};


	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	ASSERT_EQ(mean, Eigen::Vector3d(1,1,1));

	ASSERT_NEAR(cov(0,0), 0, 0.000001);
	ASSERT_NEAR(cov(1,0), 0, 0.000001);
	ASSERT_NEAR(cov(0,1), 0, 0.000001);
	ASSERT_NEAR(cov(1,1), 0, 0.000001);
	ASSERT_NEAR(cov(2,2), 0, 0.000001);

}

TEST(TestSetStatistics, test2)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1.3 ,1 ,1), Eigen::Vector3f(0.8, 0.7, 0)};
	std::vector<double> weights {0.5, 0.5};
	Particle p1(Eigen::Vector3f(1.3,1,1), 0.5);
	Particle p2(Eigen::Vector3f(0.8,0.7,0), 0.5);
	std::vector<Particle> particles{p1, p2};

	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	//ASSERT_EQ(mean, Eigen::Vector3d(1.05, 0.85, 0.5));
	ASSERT_NEAR(mean(0), 1.05 , 0.000001);
	ASSERT_NEAR(mean(1), 0.85 , 0.000001);
	ASSERT_NEAR(mean(2), 0.5 , 0.000001);
	ASSERT_NEAR(cov(0,0), 0.0625 , 0.000001);
	ASSERT_NEAR(cov(1,0), 0.0375 , 0.000001);
	ASSERT_NEAR(cov(0,1), 0.0375 , 0.000001);
	ASSERT_NEAR(cov(1,1), 0.0225, 0.000001);
	ASSERT_GE(cov(2,2), 0);  

}

TEST(TestSetStatistics, test3)
{
	std::vector<Eigen::Vector3f> poses{Eigen::Vector3f(1 ,1 ,1), Eigen::Vector3f(0, 0, 0)};
	std::vector<double> weights {1.0, 0.0};
	Particle p1(Eigen::Vector3f(1, 1,1 ), 1.0);
	Particle p2(Eigen::Vector3f(0, 0, 0), 0.0);
	std::vector<Particle> particles{p1, p2};

	SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
	Eigen::Vector3d mean = stats.Mean();
	Eigen::Matrix3d cov = stats.Cov();

	ASSERT_EQ(mean, Eigen::Vector3d(1, 1, 1));

	ASSERT_NEAR(cov(0,0), 0, 0.000001);
	ASSERT_NEAR(cov(1,0), 0, 0.000001);
	ASSERT_NEAR(cov(0,1), 0, 0.000001);
	ASSERT_NEAR(cov(1,1), 0, 0.000001);
	ASSERT_NEAR(cov(2,2), 0, 0.000001);

}


TEST(TestPlaceRecognition, test1)
{
    	std::vector<std::string> dict = {"NotValid", "Room 1", "Room 2", "Room 3", "Room 10"};
	PlaceRecognition placeRec = PlaceRecognition(dict);

	std::vector<std::string> places = {"Room'1"};
	std::vector<int> matches = placeRec.Match(places);


	ASSERT_EQ(1, matches.size());
	ASSERT_EQ(1, matches[0]);
}


TEST(TestPlaceRecognition, test2)
{
	std::vector<std::string> places = {"[Rooné 3", "{Rooms 4", "Room 8", "+ Room 6§", "(Rooms 7"};
	std::vector<std::string> dict = {"NotValid", "Room 1", "Room 2","Room 3","Room 4","Room 5","Room 6","Room 7","Room 8","Room 9","Room 10","Room 11"};
	PlaceRecognition pr = PlaceRecognition(dict);
	std::vector<int> res = pr.Match(places);


	ASSERT_EQ(4, res.size());
	ASSERT_EQ(4, res[0] );
	ASSERT_EQ(8, res[1]);
	ASSERT_EQ(6, res[2]);

}

TEST(TestPlaceRecognition, test3)
{
	std::vector<std::string> dict = {"Room 2"};
	PlaceRecognition placeRec = PlaceRecognition(dict);
	std::vector<std::string> places = {"Room2"};
	std::vector<int> matches = placeRec.Match(places);


	ASSERT_EQ(matches.size(), 1);
}

TEST(TestNMCLFactory, test1)
{
	std::string configPath = floorPath + "nmcltest.config";
	NMCLFactory::Dump(floorPath, "nmcltest.config");

	std::shared_ptr<ReNMCL> renmcl = NMCLFactory::Create(configPath);
	std::string roonName = renmcl->GetFloorMap()->GetRoomNames()[1];
	//std::cout << renmcl->GetFloorMap()->GetRoomNames()[1] << std::endl;
	bool same = (roonName == "Room 1");

	std::shared_ptr<PredictStrategy> ps = renmcl->GetPredictStrategy();
	UniformPredictStrategy* ups = dynamic_cast<UniformPredictStrategy*>(ps.get());


	ASSERT_EQ(roonName, "Room 1");
	//ASSERT(ups);

}

TEST(TestNMCLFactory, test2)
{
	std::string configPath = floorPath + "nmcl.config";
	//NMCLFactory::Dump(configPath);

	std::shared_ptr<ReNMCL> renmcl = NMCLFactory::Create(configPath);
	std::string roonName = renmcl->GetFloorMap()->GetRoomNames()[1];
	//std::cout << renmcl->GetFloorMap()->GetRoomNames()[1] << std::endl;
	bool same = (roonName == "Room 1");

	std::shared_ptr<PredictStrategy> ps = renmcl->GetPredictStrategy();
	UniformPredictStrategy* ups = dynamic_cast<UniformPredictStrategy*>(ps.get());


	ASSERT_EQ(roonName, "Room 1");
	assert(ups);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
