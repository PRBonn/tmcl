/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: TextRecoNode.cpp                                                       #
# ##############################################################################
**/


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nmcl_msgs/TextArray.h>
#include "opencv2/opencv.hpp"
#include <opencv2/dnn/dnn.hpp>

#include "TextSpotting.h"

class TextRecoNode
{
public:

	TextRecoNode(std::string name)
	{
		ros::NodeHandle nh(name);

		std::string jsonPath;
	    std::string cameraImgTopic;
	    std::string cameraInfoTopic;
	    std::string textTopic;

	    nh.getParam("textSpottingConfig", jsonPath);
	    nh.getParam("cameraImgTopic", cameraImgTopic);
	    nh.getParam("camID", o_camID);
	    nh.getParam("textTopic", textTopic);

    	TextSpotting ts = TextSpotting(jsonPath);

	    o_textSpotter = std::make_shared<TextSpotting>(ts);
	    o_textPub = nh.advertise<nmcl_msgs::TextArray>(textTopic, 10);
	    o_camSub = nh.subscribe(cameraImgTopic, 1, &TextRecoNode::callback, this);
	}

	void callback(const sensor_msgs::ImageConstPtr& imgMsg)
	{
		cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_8UC3);
		cv::Mat frame = cvPtr->image;

		try
		{
			std::vector<std::string> recRes = o_textSpotter->Infer(frame);
			nmcl_msgs::TextArray msg;
			msg.header = imgMsg->header;
			msg.text =  recRes;
			msg.id = o_camID;
			o_textPub.publish(msg);
		}
		catch (...) 
		{
  			ROS_INFO_STREAM(std::string("Failed to infer text from camera ") + std::to_string(o_camID));
		}
	
	}


private:

	std::shared_ptr<TextSpotting> o_textSpotter;
	ros::Publisher o_textPub;
	ros::Subscriber o_camSub;
	int o_camID;

};


int main(int argc, char** argv)
{
	std::string name = argv[1];
	ros::init(argc, argv, name);
	TextRecoNode tr = TextRecoNode(name);
	ros::spin();
	

	return 0;
}
