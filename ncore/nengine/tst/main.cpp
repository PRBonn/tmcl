#include <iostream>
#include "Python.h"
#include <string>
#include <vector>
#include <fstream>

#include "DataFrameLoader.h"
#include "NMCLEngine.h"
#include "Utils.h"
#include <math.h>
#include <boost/filesystem.hpp>



Eigen::Vector3f cam2BaselinkTF(Eigen::Vector3f pose)
{
    Eigen::Vector3f camTF(0.08, 0, 1);
    Eigen::Vector3f v(0, 0, pose(2) + M_PI / 2);

    Eigen::Matrix3f trans = Vec2Trans(v);

    Eigen::Vector3f gt = trans  * camTF;
    gt(0) += pose(0);
    gt(1) += pose(1);
    gt(2) = pose(2) + M_PI / 2;

    return gt;
}


void dumpParticles(const std::vector<Particle>& particles, std::string path)
{
    std::ofstream particleFile;
    particleFile.open(path, std::ofstream::out);
    particleFile << "x" << "," << "y" << "," << "yaw" << "," << "w" << std::endl;
    for(int p = 0; p < particles.size(); ++p)
    {
        Eigen::Vector3f pose = particles[p].pose;
        float w = particles[p].weight;
        particleFile << pose(0) << "," << pose(1) << "," << pose(2) << "," << w << std::endl;
    }
    particleFile.close();
}

void dumpScanMask(const std::vector<Eigen::Vector3f>& scan, const std::vector<double>& scanMask, std::string path)
{
    if (scan.empty()) return;

    std::ofstream scanFile;
    scanFile.open(path, std::ofstream::out);
    scanFile << "x" << "," << "y" << "," << "mask" << std::endl;
    int ds_factor = scan.size() / scanMask.size();

    for(int p = 0; p < scan.size(); ++p)
    {
        Eigen::Vector3f pnt = scan[p];
        double val = scanMask[int(p / ds_factor)];
        scanFile << pnt(0) << "," << pnt(1) << "," << val << std::endl;
    }
    scanFile.close();
}

void computeStartSecond(DataFrameLoader& df, int startFrame)
{
    FrameData fd = df.GetData(0);
    FrameData fd2 = df.GetData(startFrame);
    unsigned long diff = fd2.stamp - fd.stamp;
    std::cout << float(diff) / 1000000000 << std::endl;
    std::cout << fd2.stamp << std::endl;
}

#include <chrono>

void singleRun(const std::string& nmclConfigFile, const std::string& sensorConfigFolder, const std::string& textMapDir,
 const std::string& resultsDir,  DataFrameLoader& df, int runID, int startFrame, int endFrame, bool text)
{

    NMCLEngine engine(nmclConfigFile, sensorConfigFolder, textMapDir);

    if(!( boost::filesystem::exists(resultsDir)))
    {
        boost::filesystem::create_directory(resultsDir);
    }
    if(!( boost::filesystem::exists(resultsDir + "Run" + std::to_string(runID))))
    {
        boost::filesystem::create_directory(resultsDir + "Run" + std::to_string(runID));
    }

    std::string resultsFolder = resultsDir + "Run" + std::to_string(runID) + "/";


    std::string csvFilePath = resultsFolder + "poseestimation.csv";
    std::ofstream csvFile;
    csvFile.open(csvFilePath, std::ofstream::out);
    csvFile << "t" << "," << "pose_x" << "," << "pose_y" << "," << "pose_yaw" << "," << "cov_x" << "," << "cov_y" << "," << "cov_yaw"<< "," << "gt_x" << "," << "gt_y" << "," << "gt_yaw" << std::endl;


    int numFrames = endFrame;
    Eigen::Vector3f gt;
    std::vector<Eigen::Vector3f> lidar;

     for(int i = startFrame; i < numFrames; ++i)
    {
        FrameData fd = df.GetData(i);
        char frame[8];
        sprintf(frame, "%07d", i);

        switch (fd.type)
        {
            case FrameTypes::TEXT0: case FrameTypes::TEXT1: case FrameTypes::TEXT2: case FrameTypes::TEXT3: 
            {
                if(text)
                {
                    //std::cout << "Run " << runID << ", frame " << i << "/" << numFrames << std::endl;
                    std::vector<std::string> places = fd.places;
                    engine.TextMask(places, int(fd.type) % 4);
                }
                break;
            }
            case FrameTypes::LIDAR:
             {
                std::vector<Eigen::Vector3f> scan = fd.scan;
                lidar = scan;

                int ret = engine.Correct(scan);
                if(ret)
                {
                    SetStatistics stats = engine.PoseEstimation();
                    Eigen::Vector3d state = stats.Mean(); 
                    Eigen::Matrix3d cov = stats.Cov();

                   std::cout << "Run " << runID << ", frame " << i << "/" << numFrames << std::endl;
                   std::cout << "state:" << state(0) << ", " << state(1) << ", " << state(2) << std::endl;
                   std::cout << "gt:" << gt(0) << ", " << gt(1) << ", " << gt(2) << std::endl;

                    csvFile << fd.stamp << "," << state(0) << "," << state(1) << "," << state(2) << "," << cov(0,0) << "," << cov(1,1) << "," << cov(2, 2) << "," << gt(0) << "," << gt(1) << "," << gt(2) << std::endl;
                    std::vector<Particle> particles = engine.Particles();
                    std::string particlesPath = resultsFolder + std::string(frame) + "_particles.csv";
                    dumpParticles(particles, particlesPath);
                  
                }
                break;
            }
            case FrameTypes::ODOM:
            {
                Eigen::Vector3f odom = fd.odom;
                engine.Predict(odom);
                break;
            }
            case FrameTypes::GT:
            {
                gt = fd.gt;               
                break;
            }
            default:
                //std::cerr << "wrong input type!" << std::endl;
                break;
        }
    }

    csvFile.close();

}


int main(int argc, char** argv)
{
    std::string moduleFolder = "/home/nuc20/Code/YouBotMCL/ncore/nengine/src";
    std::string moduleName = "df_loader";
    std::string sensorConfigFolder = "/home/nuc20/Code/YouBotMCL/ros1_ws/src/nmcl_ros/config/480x640/";

    std::string mapType = "GMap";
    int particles = 300;
    std::string model = "nicky";
    int sequenceID = 0;

    std::string sequences[5] = {"S", "D1", "D2", "D3", "D4"};

    std::string nmclConfigFile = "/home/nuc20/Code/YouBotMCL/ncore/data/floor/"+ mapType + "/nmcl" + std::to_string(particles) + ".config";
    std::string textMapDir = "/home/nuc20/Code/YouBotMCL/ncore/data/floor/"+ mapType + "/TextMaps/";

    std::string picklePath = "/home/nuc20/data/MCL/"+ sequences[sequenceID] + "/textgtmerged.pickle";
    std::string resultsDir = "/home/nuc20/data/iros2022/" + mapType + "/" + sequences[sequenceID] + "/particles_" + std::to_string(particles) + "/" + model + "/";

    DataFrameLoader df(moduleFolder, moduleName);
    df.Load(picklePath);
    int numFrames = df.GetNumFrames();
    std::vector<int> startFrames;

    switch(sequenceID) {
        case 0 : 
            startFrames = {113, 3492, 9674, 15788, 21993, 32764, 40781, 57690, 62345, 78045};
            break;
        case 1 : 
            startFrames = {3766, 8971, 19154, 27379, 44003};
            break;
        case 2:
            startFrames = {245, 7371};
            break;
        case 3:
            startFrames = {142969, 154881, 163884, 188863, 196157};
            break;
        case 4:
            startFrames = {27, 13709, 24710, 30045};
            break;
    }
    std::vector<int> endFrames = {numFrames, numFrames, numFrames, 266951, 85535};

    bool text = true;

    for(long unsigned int s = 3; s < startFrames.size(); ++s)
    {
       
        srand48(1);
        singleRun(nmclConfigFile, sensorConfigFolder, textMapDir, resultsDir, df, s, startFrames[s], endFrames[sequenceID], text);
    }
    std::cout << "text :" << text << std::endl;

    return 0;
}
