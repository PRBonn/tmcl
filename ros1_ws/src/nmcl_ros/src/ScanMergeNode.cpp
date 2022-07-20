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
#include <mutex> 
#include <sstream>  


#include "Utils.h"
#include "Lidar2D.h"
#include "RosUtils.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> LidarSyncPolicy;


class ScanMergeNode
{
public:
	ScanMergeNode()
	{
		
		ros::NodeHandle nh("~");
		
		std::string configFolder;
		std::string scanFrontTopic;
		std::string scanRearTopic;
		
		nh.getParam("configFolder", configFolder);
		nh.getParam("scanFrontTopic", scanFrontTopic);  
		nh.getParam("scanRearTopic", scanRearTopic); 
		
		std::string fldrName =  "front_laser";      
		std::string rldrName =  "rear_laser";  

		l2d_f = std::make_shared<Lidar2D>(Lidar2D(fldrName, configFolder));
		l2d_r = std::make_shared<Lidar2D>(Lidar2D(rldrName, configFolder));
		
		o_scanPub =  nh.advertise<sensor_msgs::PointCloud2>("scan_merged", 10);
		
		laserFrontSub = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh, scanFrontTopic, 10);
		laserRearSub = std::make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh, scanRearTopic, 10);
		

		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		lidarSync = std::make_shared<message_filters::Synchronizer<LidarSyncPolicy>>(LidarSyncPolicy(10), *laserFrontSub, *laserRearSub);

		lidarSync->registerCallback(boost::bind(&ScanMergeNode::callback, this, _1, _2)); 

		ROS_INFO_STREAM("ScanMergeNode running!");   

	}



	void callback(const sensor_msgs::LaserScanConstPtr& laserFront, const sensor_msgs::LaserScanConstPtr& laserRear)
	{

		std::vector<float> scanFront = laserFront->ranges;
		std::vector<float> scanRear = laserRear->ranges;

		std::vector<Eigen::Vector3f> points_3d = MergeScansSimple(scanFront, *l2d_f, scanRear, *l2d_r);

		pcl::PointCloud<pcl::PointXYZ> pcl = Vec2PointCloud(points_3d);
		sensor_msgs::PointCloud2 pcl_msg;
		pcl::toROSMsg(pcl, pcl_msg);
		pcl_msg.header.stamp = laserFront->header.stamp;
		pcl_msg.header.frame_id = laserFront->header.frame_id;
		o_scanPub.publish(pcl_msg);
	}


private:


	ros::Publisher o_scanPub;
	std::shared_ptr<Lidar2D> l2d_f;
	std::shared_ptr<Lidar2D> l2d_r;
	
	std::shared_ptr<message_filters::Synchronizer<LidarSyncPolicy>> lidarSync;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laserFrontSub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> laserRearSub;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "ScanMergeNode");
	ScanMergeNode nmcl = ScanMergeNode();
	ros::spin();
	

	return 0;
}