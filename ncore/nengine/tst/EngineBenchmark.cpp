

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "NMCLEngine.h"
#include "Utils.h"
#include <math.h>
#include <chrono>

#include "DataFrameLoader.h"

std::vector<Eigen::Vector3f> GenerateRandomScan()
{
	 std::vector<Eigen::Vector3f> scan;
	 int max_dist = 10;
	 for(int i = 0; i < 2082; ++i)
	 {
	 	float x  = drand48() * (2 * max_dist) - max_dist;
	 	float y  = drand48() * (2 * max_dist) - max_dist;
	 	float theta =  drand48() * 2 * M_PI - M_PI;
	 	scan.push_back(Eigen::Vector3f(x, y, theta));
	 }
	 return scan;
}





int main(int argc, char** argv)
{
    std::string moduleFolder = "/home/nuc20/Code/YouBotMCL/ncore/nengine/src";
    std::string moduleName = "df_loader";
    std::string sensorConfigFolder = "/home/nuc20/Code/YouBotMCL/ros1_ws/src/nmcl_ros/config/480x640/";
    std::string nmclConfigFile = "/home/nuc20/Code/YouBotMCL/ncore/data/floor/Faro/nmcl.config";
    std::string textMapDir = "/home/nuc20/Code/YouBotMCL/ncore/data/floor/Faro/TextMaps/";
    std::string dataFolder = "/home/nuc20/data/MCL/2022_01_14/Run2/";
    std::string pickleName = "textgtmerged.pickle";

    DataFrameLoader df(moduleFolder, moduleName);
    df.Load(dataFolder + pickleName);
    int numFrames = df.GetNumFrames();

    int count = 0;
  	srand48(1);
    NMCLEngine engine(nmclConfigFile, sensorConfigFolder, textMapDir);
    auto millis = 0;
    for(int i = 0; i < numFrames; ++i)
    {
    	FrameData fd = df.GetData(i);
    	//     	std::vector<Eigen::Vector3f> scan = GenerateRandomScan();

    	if (fd.type == FrameTypes::LIDAR)
    	{
    		std::vector<Eigen::Vector3f> scan = fd.scan;
    		auto start = std::chrono::system_clock::now();
		    int ret = engine.Correct(scan);
		    auto end = std::chrono::system_clock::now();
		   	std::chrono::duration<double> duration = end - start;
	        if(ret)
	        {
	        	if (count > 499) break;
	        	++count;
	        	millis += std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
	        	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << std::endl;
	        }
    	}
    	
    }

    std::cout << "finished computation at " << millis/500 << std::endl;



    return 0;

  }
