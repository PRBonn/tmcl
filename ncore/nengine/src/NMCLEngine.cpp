

#include "NMCLEngine.h"
#include "NMCLFactory.h"
#include "LidarData.h"
#include "Utils.h"
#include <numeric>



NMCLEngine::NMCLEngine(const std::string& nmclConfigPath, const std::string& sensorConfigFolder, const std::string& textMapDir)
{
	o_scanMask = std::vector<double>(1041 * 2, 1.0);
	o_odomNoise = Eigen::Vector3f(0.02, 0.02, 0.02); 

	o_renmcl = NMCLFactory::Create(nmclConfigPath);
	std::vector<std::string> dict = o_renmcl->GetFloorMap()->GetRoomNames();
	o_placeRec = std::make_shared<PlaceRecognition>(PlaceRecognition(dict));

	o_textBBs.push_back(cv::Rect(0, 0, 0, 0));
	o_textOrientations.push_back(0.0);

	for(int i = 1; i < 11; ++i)
	{
		cv::Mat img = cv::imread(textMapDir + dict[i] + ".png");
		cv::Mat bgr[3];   
		cv::split(img, bgr);
 
		cv::Mat heatmap = bgr[0];
		cv::Mat yawmap = bgr[1];
		cv::Mat textmap = bgr[2];

		std::vector<cv::Point2i> locations; 
		cv::findNonZero(heatmap, locations);
		float avgAngle = yawmap.at<uchar>(locations[0]);
		avgAngle = 2 * M_PI* (avgAngle / 255) - M_PI;
		
		cv::Rect bb = boundingRect(locations);
		o_textBBs.push_back(bb);
		o_textOrientations.push_back(avgAngle);
	}



	o_cameras.push_back(std::make_shared<Camera>(Camera(sensorConfigFolder + "cam0.config")));
	o_cameras.push_back(std::make_shared<Camera>(Camera(sensorConfigFolder + "cam1.config")));
	o_cameras.push_back(std::make_shared<Camera>(Camera(sensorConfigFolder + "cam2.config")));
	o_cameras.push_back(std::make_shared<Camera>(Camera(sensorConfigFolder + "cam3.config")));

}


void NMCLEngine::Predict(Eigen::Vector3f wheelCurrPose)
{

	if(o_initPose)
	{
		o_wheelPrevPose = wheelCurrPose;
		o_initPose = false;	
	}
	else
	{
		Eigen::Vector3f delta = wheelCurrPose - o_wheelPrevPose;

		if((((sqrt(delta(0) * delta(0) + delta(1) * delta(1))) > o_triggerDist) || (fabs(delta(2)) > o_triggerAngle)) || o_first)
		{
			o_first = false;
			Eigen::Vector3f uWheel = o_renmcl->Backward(o_wheelPrevPose, wheelCurrPose);

			std::vector<Eigen::Vector3f> command{uWheel};
			o_renmcl->Predict(command, o_odomWeights, o_odomNoise);

			o_wheelPrevPose = wheelCurrPose;   
			o_step = true; 
		}
	}
}


int NMCLEngine::Correct(const std::vector<Eigen::Vector3f>& fullScan)
{
	if (o_step && (fullScan.size()))
	{
		std::vector<Eigen::Vector3f> scan = fullScan;
		
		LidarData data = LidarData(scan, o_scanMask);
		o_renmcl->Correct(std::make_shared<LidarData>(data)); 
	
		SetStatistics stas = o_renmcl->Stats();
		Eigen::Matrix3d cov = stas.Cov();
		Eigen::Vector3d pred = stas.Mean(); 
		if(pred.array().isNaN().any() || cov.array().isNaN().any() || cov.array().isInf().any())
		{ 
			std::cerr << "fails to Localize!" << std::endl;
			o_renmcl->Recover();
		}
		o_step = false;
		return 1;
	}
	return 0;
}


void NMCLEngine::TextMask(const cv::Mat& img, int camID)
{
	std::vector<std::string> places = o_textSpotter->Infer(img);
	std::vector<int> matches = o_placeRec->Match(places);   
	int numMatches = matches.size();
	std::vector<cv::Rect> bbs;
	std::vector<float> orientations;

	if (numMatches)
	{	float camAngle = 0;
		if (camID == 1) camAngle = -0.5 * M_PI;
		else if (camID == 2) camAngle = M_PI;
		else if (camID == 3) camAngle = 0.5 * M_PI;

		std::vector<Eigen::Matrix3d> covariances;
		for(int i = 0; i < matches.size(); ++i)
		{
			int id = matches[i];

			if ((camID != o_lastDetectedCamera) || (id != o_lastDetectedMatch))				
			{	
				o_lastDetectedCamera = camID;
				o_lastDetectedMatch = id;
				std::cout << o_renmcl->GetFloorMap()->GetRoomNames()[matches[i]] << std::endl;
		
      		
		              	bbs.push_back(o_textBBs[id]);
				orientations.push_back(o_textOrientations[id]);
			}
		}

		if (bbs.size()) o_renmcl->Relocalize(bbs, orientations, camAngle); 
	} 
}


void NMCLEngine::TextMask(std::vector<std::string>& places, int camID)
{
	std::vector<int> matches = o_placeRec->Match(places);   
	int numMatches = matches.size();
	std::vector<cv::Rect> bbs;
	std::vector<float> orientations;

	if (numMatches)
	{	
		float camAngle = 0;
		if (camID == 1) camAngle = -0.5 * M_PI;
		else if (camID == 2) camAngle = M_PI;
		else if (camID == 3) camAngle = 0.5 * M_PI;

		for(int i = 0; i < matches.size(); ++i)
		{
			int id = matches[i];

			if ((camID != o_lastDetectedCamera) || (id != o_lastDetectedMatch))			
			{	
				o_lastDetectedCamera = camID;
				o_lastDetectedMatch = id;
				std::cout << o_renmcl->GetFloorMap()->GetRoomNames()[matches[i]] << std::endl;
		            
                		bbs.push_back(o_textBBs[id]);
				orientations.push_back(o_textOrientations[id]);
				//std::cout << "Injecting particles" << std::endl;
			}
		}

		if (bbs.size()) o_renmcl->Relocalize(bbs, orientations, camAngle); 
	} 
}

