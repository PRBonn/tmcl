


#include "ReNMCL.h"
#include "TextSpotting.h"
#include "PlaceRecognition.h"
#include "Camera.h"
#include <fstream>



class NMCLEngine
{
public:

	NMCLEngine(const std::string& nmclConfigPath, const std::string& sensorConfigFolder, const std::string& textMapDir);

	void Predict(Eigen::Vector3f odom);

	int Correct(const std::vector<Eigen::Vector3f>& scan);

	void TextMask(const cv::Mat& img, int camID);

	void TextMask(std::vector<std::string>& places, int camID);

	
	SetStatistics PoseEstimation() const
	{
		return o_renmcl->Stats();
	}

	std::vector<Particle> Particles() const
	{
		return o_renmcl->Particles();
	}

	std::vector<double> ScanMask() const
	{
		return o_scanMask;
	}


private:	



	std::shared_ptr<TextSpotting> o_textSpotter;
	std::shared_ptr<ReNMCL> o_renmcl;
	std::shared_ptr<PlaceRecognition> o_placeRec;
	Eigen::Vector3f o_wheelPrevPose = Eigen::Vector3f(0, 0, 0);
	std::vector<double> o_scanMask;
	bool o_first = true;
	int o_dsFactor = 1;
	Eigen::Vector3f o_odomNoise = Eigen::Vector3f(0.02, 0.02, 0.02);
	std::vector<float> o_odomWeights = {1.0};
	float o_triggerDist = 0.05;
	float o_triggerAngle = 0.05;
	bool o_step = false;
	bool o_initPose = true;
	

	std::vector<std::shared_ptr<Camera>> o_cameras;
	//std::vector<cv::Mat> textMaps;
	std::vector<cv::Rect> o_textBBs;
	std::vector<float> o_textOrientations;
	int o_lastDetectedCamera = -1;
	int o_lastDetectedMatch = -1;
};
