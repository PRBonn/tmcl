/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: BeamEnd.cpp                                                           #
# ##############################################################################
**/

#include "BeamEnd.h"
#include "Utils.h"

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include "LidarData.h"

BeamEnd::BeamEnd(std::shared_ptr<GMap> Gmap_, float sigma_, float maxRange_, Weighting weighting )
{
	Gmap = Gmap_;
	maxRange = maxRange_;
	sigma = sigma_;   //value of 8 for map resolution 0.05, 40 for 0.01
	o_weighting = weighting;
	cv::threshold(Gmap->Map(), edt, 127, 255, 0);
	edt = 255 - edt;
	cv::distanceTransform(edt, edt, cv::DIST_L2, cv::DIST_MASK_3);
	cv::threshold(edt, edt, maxRange, maxRange, 2); //Threshold Truncated
}


void BeamEnd::ComputeWeights(std::vector<Particle>& particles, std::shared_ptr<SensorData> data)
{
	LidarData* ptr = dynamic_cast<LidarData*>(data.get());
	assert(ptr);
	std::vector<Eigen::Vector3f> scan = ptr->Scan();
	std::vector<double> scanMask = ptr->Mask();

	std::vector<double> weights;

	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		double w = 0;
		switch(o_weighting) 
		{
		    case Weighting::NAIVE : 
		    	w = geometric(particles[i].pose, scan, scanMask);
		    	break;
		    case Weighting::INTEGRATION : 
		    	w = integration(particles[i].pose, scan, scanMask);
		    	break;
		    case Weighting::LAPLACE : 
		    	w = laplace(particles[i].pose, scan, scanMask);
		    	break;
		    case Weighting::GEOMETRIC : 
		    	w = geometric(particles[i].pose, scan, scanMask);
		    	break;
		    case Weighting::GPOE : 
		    	w = gPoE(particles[i].pose, scan, scanMask);
		    	break;
		}

		//double w = geometric(particles[i].pose, scan, scanMask);
		// considering previous particle weight
		//particles[i].weight *= w;
		// not considering previous particle weight
		particles[i].weight = w;
	}
}

double BeamEnd::naive(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle, mapPoints);

	Eigen::Vector2f br = Gmap->BottomRight();

	double weight = 1.0;
	int nonValid = 0;

	for(long unsigned int i = 0; i < mapPoints.size(); ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					++nonValid;
			}
			else
			{
				float dist = edt.at<float>(mp(1) ,mp(0));
				double w = getLikelihood(dist);
				weight *= w;
			}
		}
	}

	float penalty = pow(getLikelihood(maxRange), nonValid);
	weight *= penalty;

	return weight;
}

double BeamEnd::geometric(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle, mapPoints);

	Eigen::Vector2f br = Gmap->BottomRight();
	float geoW = 1.0 / scan.size();

	double weight = 1.0;
	int nonValid = 0;
	int valid = 0;

	for(long unsigned int i = 0; i < mapPoints.size(); ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					++nonValid;
			}
			else
			{
				float dist = edt.at<float>(mp(1) ,mp(0));
				double w = getLikelihood(dist);
				weight *= pow(w, geoW);
			}
		}
	}

	float penalty = pow(getLikelihood(maxRange), nonValid * geoW);
	weight *= penalty;


	return weight;
}

double BeamEnd::gPoE(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle, mapPoints);

	Eigen::Vector2f br = Gmap->BottomRight();
	float geoW = 1.0 / scan.size();

	double weight = 1.0;
	int nonValid = 0;
	int valid = 0;

	for(long unsigned int i = 0; i < mapPoints.size(); ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					++nonValid;
			}
			else
			{
				float dist = edt.at<float>(mp(1) ,mp(0));
				double w = getLikelihood(dist);
				if (dist < sigma)
				{
					weight *= pow(w, geoW);
					++valid;
				}
				else ++nonValid;
			}
		}
	}

	float penalty = pow(getLikelihood(maxRange), nonValid * geoW);

	if(valid)
	{
		weight *= penalty;
		weight *= valid;
	}
	else
	{
		weight = penalty;
	}

	return weight;
}


double BeamEnd::integration(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);
	//plotScan(particle, mapPoints);

	Eigen::Vector2f br = Gmap->BottomRight();

	double sumDist = 0;
	long unsigned numPoints =  mapPoints.size();

	for(long unsigned int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
					sumDist += maxRange;
			}
			else
			{
				float dist = edt.at<float>(mp(1) ,mp(0));
				sumDist += dist;
			}
		}
	}

	double weight = getLikelihood(sumDist / numPoints);

	return weight;
}


double BeamEnd::laplace(Eigen::Vector3f particle, const std::vector<Eigen::Vector3f>& scan, std::vector<double> scanMask)
{
	std::vector<Eigen::Vector2f> mapPoints = scan2Map(particle, scan);

	Eigen::Vector2f br = Gmap->BottomRight();
	std::vector<float> distances;

	
	int valid = 0;
	double weight = 1.0;
	double sumDist = 0;
	long unsigned numPoints =  mapPoints.size();

	for(long unsigned int i = 0; i < numPoints; ++i)
	{
		Eigen::Vector2f mp = mapPoints[i];

		if(scanMask[i] > 0.0)
		{
			if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1)))
			{
		
					sumDist += maxRange;
			}
			else
			{
				float dist = edt.at<float>(mp(1) ,mp(0));
				sumDist += dist;
				// change sigma to better name when I have nothing to do with my life
				if (dist < sigma)
				{
					++valid;
				}
			}
		}
	}


	double avgDist = sumDist / valid;

	if (valid == 0)
	{
		weight = exp(-maxRange);
		return weight;
	}

	weight = exp(-avgDist);

	return weight;
}

bool BeamEnd::IsValid(Particle particle)
{
	Eigen::Vector3f pose = particle.pose;
	Eigen::Vector2f xy = Eigen::Vector2f(pose(0), pose(1));
	Eigen::Vector2f mp = Gmap->World2Map(xy);

	return isValid(mp);
}

bool BeamEnd::isValid(Eigen::Vector2f mp)
{
	Eigen::Vector2f br = Gmap->BottomRight();
	if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) > br(0)) || (mp(1) > br(1))) return false;

	int val = Gmap->Map().at<uchar>(mp(1) ,mp(0));

	if (val > 1) return false;

	return true;
}


bool BeamEnd::IsOccupied(Particle particle)
{
	Eigen::Vector3f pose = particle.pose;
	Eigen::Vector2f xy = Eigen::Vector2f(pose(0), pose(1));
	Eigen::Vector2f uv = Gmap->World2Map(xy);

	float dist = edt.at<float>(uv(0) ,uv(1));
	float l = getLikelihood(dist);
	if(l > 0.82) return true;
	else return false;
}

std::vector<Particle> BeamEnd::InitUniform(int n_particles)
{
	std::vector<Particle> particles;
	Eigen::Vector2f tl = Gmap->Map2World(Gmap->TopLeft());
	Eigen::Vector2f br = Gmap->Map2World(Gmap->BottomRight());

	int i = 0;
	while(i < n_particles)
	{
			float x = drand48() * (br(0) - tl(0)) + tl(0);
			float y = drand48() * (br(1) - tl(1)) + tl(1);
			//if(IsOccupied(Particle(Eigen::Vector3f(x, y, 0)))) continue;
			if(!IsValid(Particle(Eigen::Vector3f(x, y, 0)))) continue;

			float theta = drand48() * 2 * M_PI - M_PI;
			Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
			particles.push_back(p);
			++i;
	}

	//particles[0] = Eigen::Vector3f(0.00, -0.00, 0);

	return particles;
}


 std::vector<Particle> BeamEnd::InitBoundingBox(int n_particles, std::vector<Eigen::Vector2f> tls, std::vector<Eigen::Vector2f> brs, std::vector<float> yaws)
{
	std::vector<Particle> particles;


	for(long unsigned int i = 0; i < tls.size(); ++i)
	{
		Eigen::Vector2f tl = Gmap->Map2World(tls[i]);
		Eigen::Vector2f br = Gmap->Map2World(brs[i]);
		/*std::cout << "w_tl " << tl << std::endl;
		std::cout << "w_br " << br << std::endl;*/
		float yaw = yaws[i];

		int n = 0;
		while(n < n_particles)
		{
				float x = drand48() * (br(0) - tl(0)) + tl(0);
				float y = drand48() * (br(1) - tl(1)) + tl(1);
				//if(IsOccupied(Particle(Eigen::Vector3f(x, y, 0)))) continue;
				if(!IsValid(Particle(Eigen::Vector3f(x, y, 0)))) continue;

				float theta = 0.05 * (drand48() * 2 * M_PI - M_PI);
				theta += yaw;
				Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
				particles.push_back(p);
				++n;
		}
	}

	return particles;
}


std::vector<Particle> BeamEnd::InitGaussian(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances)
{
	std::vector<Particle> particles;

	for(long unsigned int i = 0; i < initGuess.size(); ++i)
	{
		Eigen::Vector3f initG = initGuess[i];
		Eigen::Matrix3d cov = covariances[i];

		float dx = fabs(SampleGuassian(cov(0, 0)));
		float dy = fabs(SampleGuassian(cov(1, 1)));
		float dt = fabs(SampleGuassian(cov(2, 2)));

		if (cov(2, 2) < 0.0) dt = M_PI;
		//if (cov(2, 2) > 1.0) dt = M_PI;

		Eigen::Vector3f delta(dx, dy, dt);

		Eigen::Vector3f tl = initG + delta;
		Eigen::Vector3f br = initG - delta;

		int n = 0;
		while(n < n_particles)
		{
				float x = drand48() * (br(0) - tl(0)) + tl(0);
				float y = drand48() * (br(1) - tl(1)) + tl(1);
				//if(IsOccupied(Particle(Eigen::Vector3f(x, y, 0)))) continue;
				if(!IsValid(Particle(Eigen::Vector3f(x, y, 0)))) continue;

				float theta = drand48() * (br(2) - tl(2)) + tl(2);
				Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
				particles.push_back(p);
				++n;
		}
	}


	return particles;
}


std::vector<Particle> BeamEnd::InitOriented(int n_particles, std::vector<Eigen::Vector3f> initGuess, std::vector<Eigen::Matrix3d> covariances)
{
	std::vector<Particle> particles;

	for(long unsigned int i = 0; i < initGuess.size(); ++i)
	{
		Eigen::Vector3f initG = initGuess[i];
		Eigen::Matrix3d cov = covariances[i];

		// float dx = fabs(SampleGuassian(cov(0, 0)));
		// float dy = fabs(SampleGuassian(cov(1, 1)));
		float dt = SampleGuassian(cov(2, 2));
		float dx = cov(0, 0);
		float dy = cov(1, 1);
		

		Eigen::Vector3f delta(dx, dy, 0);

		Eigen::Vector3f tl = initG + delta;
		Eigen::Vector3f br = initG - delta;
		float camAngle = initG(2);

		int n = 0;
		while(n < n_particles)
		{
				float x = drand48() * (br(0) - tl(0)) + tl(0);
				float y = drand48() * (br(1) - tl(1)) + tl(1);

				//if(IsOccupied(Particle(Eigen::Vector3f(x, y, 0)))) continue;
				if(!IsValid(Particle(Eigen::Vector3f(x, y, 0)))) continue;

				Eigen::Vector2f orientation = Eigen::Vector2f(initG(0), initG(1)) - Eigen::Vector2f(x, y);
				orientation.normalize();
				float theta = Wrap2Pi(atan2(orientation(1), orientation(0)) - camAngle + dt);

				Particle p(Eigen::Vector3f(x, y, theta), 1.0 / n_particles);
				particles.push_back(p);
				++n;
		}
	}


	return particles;
}


float BeamEnd::getLikelihood(float distance)
{
	float coeff = 1.0 / sqrt(2 * M_PI * sigma);
	float l = coeff * exp(-0.5 * pow(distance / sigma, 2));
	return l;
}

std::vector<float> BeamEnd::getLikelihood(const std::vector<float>& distances)
{
	std::vector<float> likelihood;
	float coeff = 1.0 / sqrt(2 * M_PI * sigma);
	for(long unsigned int i = 0; i < distances.size(); ++i)
	{
		float l = coeff * exp(-0.5 * pow(distances[i] / sigma, 2));
		likelihood.push_back(l);
	}

	return likelihood;
}

void BeamEnd::plotParticles(std::vector<Particle>& particles, std::string title, bool show)
{
	cv::Mat img; 
	cv::cvtColor(Gmap->Map(), img, cv::COLOR_GRAY2BGR);

	cv::namedWindow("Particles", cv::WINDOW_NORMAL);

	for(long unsigned int i = 0; i < particles.size(); ++i)
	{
		Eigen::Vector3f p = particles[i].pose;
		Eigen::Vector2f uv = Gmap->World2Map(Eigen::Vector2f(p(0), p(1)));

		cv::circle(img, cv::Point(uv(0), uv(1)), 5,  cv::Scalar(0, 0, 255), -1);
	}
	if(show)
	{
		cv::imshow("Particles", img);
		cv::waitKey(0);
	}
	cv::imwrite("/home/nickybones/Code/YouBotMCL/nmcl/" + title + ".png", img);

	cv::destroyAllWindows();
}


void BeamEnd::plotScan(Eigen::Vector3f laser, std::vector<Eigen::Vector2f>& zMap)
{
	cv::Mat img; 
	cv::cvtColor(Gmap->Map(), img, cv::COLOR_GRAY2BGR);
	
	cv::namedWindow("Scan", cv::WINDOW_NORMAL);
	Eigen::Vector2f p = Gmap->World2Map(Eigen::Vector2f(laser(0), laser(1)));

	cv::circle(img, cv::Point(p(0), p(1)), 5,  cv::Scalar(255, 0, 0), -1);

	for(long unsigned int i = 0; i < zMap.size(); ++i)
	{
		Eigen::Vector2f p = zMap[i];
		cv::circle(img, cv::Point(p(0), p(1)), 1,  cv::Scalar(0, 0, 255), -1);
	}

	cv::Rect myROI(475, 475, 400, 600);
	// Crop the full image to that image contained by the rectangle myROI
	// Note that this doesn't copy the data
	cv::Mat img_(img, myROI);

	cv::imwrite("scan.png", img_);
	cv::imshow("Scan", img_);
	cv::waitKey(0);


	cv::destroyAllWindows();
}


std::vector<Eigen::Vector2f> BeamEnd::scan2Map(Eigen::Vector3f pose, const std::vector<Eigen::Vector3f>& scan)
{
	Eigen::Matrix3f trans = Vec2Trans(pose);
	std::vector<Eigen::Vector2f> mapPoints;
	//std::vector<Eigen::Vector3f> transPoints;

	for(long unsigned int i = 0; i < scan.size(); ++i)
	{
		Eigen::Vector3f ts = trans * scan[i];
		Eigen::Vector2f mp = Gmap->World2Map(Eigen::Vector2f(ts(0), ts(1)));
		//transPoints.push_back(ts);
		mapPoints.push_back(mp);
	}

	return mapPoints;
}