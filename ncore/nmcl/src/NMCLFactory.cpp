/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NMCLFactory.cpp          				                               #
# ##############################################################################
**/


#include <fstream> 

#include "NMCLFactory.h"

#include "MotionModel.h"
#include "SensorModel.h"
#include "Resampling.h"
#include "SetStatistics.h"
#include "GMap.h"
#include "BeamEnd.h"
#include "MixedFSR.h"
#include "FSR.h"
#include "FloorMap.h"
#include "AggressiveLowVarianceResampling.h"
#include "PredictStrategy.h"
#include "UniformPredictStrategy.h"
#include "GaussianPredictStrategy.h"
#include "LowVarianceResampling.h"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

std::shared_ptr<ReNMCL> NMCLFactory::Create(const std::string& configPath)
{

	std::ifstream file(configPath);
	json config;
	file >> config;

	std::string sensorModel = config["sensorModel"]["type"];
	std::string motionModel = config["motionModel"];
	std::string resampling = config["resampling"];
	bool tracking = config["tracking"];
	std::string predictStrategy = config["predictStrategy"];


	std::shared_ptr<SensorModel> sm;
	std::shared_ptr<MotionModel> mm;
	std::shared_ptr<Resampling> rs;
	std::shared_ptr<FloorMap> fp;
	std::shared_ptr<ReNMCL> renmcl;
	std::shared_ptr<PredictStrategy> ps;

	int numParticles = config["numParticles"];
	float injRatio = config["injRatio"];

	// std::string mapFolder = config["mapFolder"];
	// GMap gmap = GMap(mapFolder);

	// cv::Mat roomSeg = cv::imread(mapFolder + "YouBotMapRoomSeg.png");
 //    FloorMap floormap = FloorMap(std::make_shared<GMap>(gmap), roomSeg);
	 // std::ifstream ifs(mapFolder + "editor.xml");
  //   boost::archive::text_iarchive ia(ifs);
  //   ia >> floormap;
  //   ifs.close(); 

    std::string jsonPath = config["floorMapPath"];
    FloorMap floormap = FloorMap(jsonPath);

  	fp = std::make_shared<FloorMap>(floormap);

	if(sensorModel == "BeamEnd")
	{
		float likelihoodSigma = config["sensorModel"]["likelihoodSigma"];
		float maxRange = config["sensorModel"]["maxRange"];
		int wScheme = config["sensorModel"]["weightingScheme"];
		sm = std::make_shared<BeamEnd>(BeamEnd(fp->Map(), likelihoodSigma, maxRange, BeamEnd::Weighting(wScheme)));
	}


	if(motionModel == "MixedFSR")
	{
		mm = std::make_shared<MixedFSR>(MixedFSR());
	}
	else if(motionModel == "FSR")
	{
		mm = std::make_shared<FSR>(FSR());
	}

	if(resampling == "LowVarianceResampling")
	{
		rs = std::make_shared<LowVarianceResampling>(LowVarianceResampling());
	}
	else if(resampling == "AggressiveLowVarianceResampling")
	{
		rs = std::make_shared<AggressiveLowVarianceResampling>(AggressiveLowVarianceResampling());
	}
	if(tracking)
	{
		//TODO implement these
		std::vector<Eigen::Vector3f> initGuesses;
		std::vector<Eigen::Matrix3d> covariances;
		renmcl = std::make_shared<ReNMCL>(ReNMCL(fp, mm, sm, rs, numParticles, initGuesses, covariances, injRatio));
	}
	else
	{
		renmcl = std::make_shared<ReNMCL>(ReNMCL(fp, mm, sm, rs, numParticles, injRatio));
	}
	if (predictStrategy == "Uniform")
	{
		renmcl->SetPredictStrategy(std::make_shared<UniformPredictStrategy>(UniformPredictStrategy()));
	}
	else if (predictStrategy == "Gaussian")
	{
		renmcl->SetPredictStrategy(std::make_shared<GaussianPredictStrategy>(GaussianPredictStrategy()));
	}


	return renmcl;
}


void NMCLFactory::Dump(const std::string& configFolder, const std::string& configFile)
{
	std::string configPath = configFolder + configFile;
	json config;
	config["sensorModel"]["type"] = "BeamEnd";
	config["sensorModel"]["likelihoodSigma"] = 8;
	config["sensorModel"]["maxRange"] = 15;
	config["sensorModel"]["weightingScheme"] = 2;
	config["motionModel"] = "MixedFSR";
	config["resampling"] = "LowVarianceResampling";
	config["tracking"] =  false;
	config["predictStrategy"] = "Uniform";
	config["floorMapPath"] = configFolder + "floor.config";
	config["numParticles"] = 10000;
	config["injRatio"] = 0.5;

	std::ofstream file(configPath);
	file << std::setw(4) << config << std::endl;
}
