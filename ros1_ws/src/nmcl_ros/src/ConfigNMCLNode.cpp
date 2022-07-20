/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ConfigNMCLNode.cpp                                                   #
# ##############################################################################
**/

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nmcl_msgs/TextArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_ros/transform_listener.h>  
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <mutex> 
#include <sstream>  

#include "Utils.h"
#include "ReNMCL.h"
#include "RosUtils.h"
#include "PlaceRecognition.h"
#include <boost/archive/text_iarchive.hpp>
#include <nlohmann/json.hpp>
#include "NMCLFactory.h"
#include "LidarData.h"
#include <nmcl_msgs/LidarScanMask.h>



class ConfigNMCLNode
{
public:
	ConfigNMCLNode()
	{
		
		ros::NodeHandle nh;
		int numParticles;

		if (!ros::param::has("dataFolder"))
		{
			ROS_FATAL_STREAM("Data folder not found!");
		}


		std::string dataFolder;
		std::string scanTopic;
		std::string odomTopic;
		std::vector<double> odomNoise;
		std::string textTopic;
		std::string maskTopic;
		std::string poseTopic;

		nh.getParam("dataFolder", dataFolder);		
		nh.getParam("scanTopic", scanTopic);
		nh.getParam("odomTopic", odomTopic);
		nh.getParam("mapTopic", o_mapTopic);
		nh.getParam("odomNoise", odomNoise);
		nh.getParam("textTopic", textTopic);
		nh.getParam("dsFactor", o_dsFactor);
		nh.getParam("maskTopic", maskTopic);
		nh.getParam("triggerDist", o_triggerDist);
		nh.getParam("triggerAngle", o_triggerAngle);
		nh.getParam("poseTopic", poseTopic);


		nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>(odomTopic, ros::Duration(60));
		o_prevPose = OdomMsg2Pose2D(odom);

		sensor_msgs::PointCloud2ConstPtr pcl = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(scanTopic, ros::Duration(60));
		int scanSize = pcl->width;
		std::vector<double> mask(scanSize, 1.0);
		o_scanMask = Downsample(mask, o_dsFactor); 

		o_mtx = new std::mutex();   
		  
		o_scanSub = nh.subscribe(scanTopic, 10, &ConfigNMCLNode::observationCallback, this); 
		o_odomSub = nh.subscribe(odomTopic, 10, &ConfigNMCLNode::motionCallback, this);   

		o_odomNoise = Eigen::Vector3f(odomNoise[0], odomNoise[1], odomNoise[2]); 		

		o_renmcl = NMCLFactory::Create(dataFolder + "nmcl.config");

		std::vector<std::string> dict = o_renmcl->GetFloorMap()->GetRoomNames();
		o_placeRec = std::make_shared<PlaceRecognition>(PlaceRecognition(dict));

		o_textBBs.push_back(cv::Rect(0, 0, 0, 0));
		o_textOrientations.push_back(0.0);

		for(int i = 1; i < 11; ++i)
		{
			cv::Mat img = cv::imread(dataFolder + "TextMaps/" + dict[i] + ".png");
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

		o_posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, 10);
		o_particlePub = nh.advertise<geometry_msgs::PoseArray>("Particles", 10);
		o_textSub = nh.subscribe(textTopic, 10, &ConfigNMCLNode::relocalizationCallback, this);     


		ROS_INFO_STREAM("Engine running!");  

	}


	void relocalizationCallback(const nmcl_msgs::TextArrayConstPtr& msg)  
	{
		std::vector<std::string> places;
		places = msg->text;
		int camID = msg->id;

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
					std::cout << "Injecting particles" << std::endl;
				}
			}
			if (bbs.size())
			{
				o_mtx->lock(); 
				o_renmcl->Relocalize(bbs, orientations, camAngle); 
				o_mtx->unlock();
				//std::cout << "relocalized" << std::endl;
			}
		} 
	}

	void motionCallback(const nav_msgs::OdometryConstPtr& odom)
	{

		Eigen::Vector3f currPose = OdomMsg2Pose2D(odom);

		Eigen::Vector3f delta = currPose - o_prevPose;

		if((((sqrt(delta(0) * delta(0) + delta(1) * delta(1))) > o_triggerDist) || (fabs(delta(2)) > o_triggerAngle)) || o_first)
		{
			o_first = false;
			Eigen::Vector3f u = o_renmcl->Backward(o_prevPose, currPose);

			std::vector<Eigen::Vector3f> command{u};
			o_mtx->lock();
			o_renmcl->Predict(command, o_odomWeights, o_odomNoise);
			o_mtx->unlock(); 

			o_prevPose = currPose;   
			o_step = true; 
		}
	}

	void observationCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
	{
		if (o_step)
		{
			int scanSize = o_scanMask.size();
			std::vector<Eigen::Vector3f> points_3d(scanSize);

			pcl::PCLPointCloud2 pcl;
	    		pcl_conversions::toPCL(*pcl_msg, pcl);
	    		pcl::PointCloud<pcl::PointXYZ> cloud;
	    		pcl::fromPCLPointCloud2(pcl, cloud);

	    		for(int i = 0; i < scanSize ; ++i)
	    		{
	    			points_3d[i] = Eigen::Vector3f(cloud.points[i * o_dsFactor].x, cloud.points[i * o_dsFactor].y, cloud.points[i * o_dsFactor].z);
	    		}

			LidarData data = LidarData(points_3d, o_scanMask);
			o_mtx->lock();
			o_renmcl->Correct(std::make_shared<LidarData>(data)); 
			o_mtx->unlock(); 

			SetStatistics stas = o_renmcl->Stats();
			Eigen::Matrix3d cov = stas.Cov();
			Eigen::Vector3d pred = stas.Mean(); 
		

			if(pred.array().isNaN().any() || cov.array().isNaN().any() || cov.array().isInf().any())
			{ 
				ROS_FATAL_STREAM("ASYNCNMCL fails to Localize!");
				o_renmcl->Recover();
			}
			else    
			{
				geometry_msgs::PoseWithCovarianceStamped poseStamped = Pred2PoseWithCov(pred, cov);
				poseStamped.header.frame_id = o_mapTopic;
				poseStamped.header.stamp = pcl_msg->header.stamp; 
				o_posePub.publish(poseStamped); 

				std::vector<Particle> particles = o_renmcl->Particles();

				geometry_msgs::PoseArray  posearray;
				posearray.header.stamp = pcl_msg->header.stamp;  
				posearray.header.frame_id = o_mapTopic;

				for (int i = 0; i < particles.size(); ++i)
				{
					geometry_msgs::Pose p;
					p.position.x = particles[i].pose(0); 
					p.position.y = particles[i].pose(1);
					p.position.z = 0; 
					tf2::Quaternion q;
					q.setRPY( 0, 0, particles[i].pose(2)); 
					p.orientation.x = q[0];
					p.orientation.y = q[1];
					p.orientation.z = q[2];
					p.orientation.w = q[3];

					posearray.poses.push_back(p);
				}

				o_particlePub.publish(posearray);
			}
			o_step = false; 		
		}	
		
	}


private:

	ros::Publisher o_posePub;
	ros::Publisher o_particlePub;
	ros::Subscriber o_textSub;
	ros::Subscriber o_scanSub;
	ros::Subscriber o_odomSub;
	std::vector<float> o_odomWeights = {1.0};

	Eigen::Vector3f o_prevPose = Eigen::Vector3f(0, 0, 0);
	Eigen::Vector3f o_odomNoise = Eigen::Vector3f(0.02, 0.02, 0.02);

	
	std::shared_ptr<ReNMCL> o_renmcl;
	std::shared_ptr<PlaceRecognition> o_placeRec;
	
	int o_dsFactor = 10;
	std::vector<double> o_scanMask;
	std::string o_mapTopic;
	bool o_first = true;
	bool o_step = false;
	std::mutex* o_mtx;
	float o_triggerDist = 0.05;
	float o_triggerAngle = 0.05;
	//std::vector<cv::Mat> textMaps;
	std::vector<cv::Rect> o_textBBs;
	std::vector<float> o_textOrientations;
	int o_lastDetectedCamera = -1;
	int o_lastDetectedMatch = -1;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "ConfigNMCLNode");
	ConfigNMCLNode nmcl = ConfigNMCLNode();
	ros::spin();
	

	return 0;
}
