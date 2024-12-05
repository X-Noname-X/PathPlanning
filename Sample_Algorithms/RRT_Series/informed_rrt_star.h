#pragma once
#ifndef _INFORMED_RRT_STAR_
#define _INFORMED_RRT_STAR_
#include "rrt_star.h"

class Informed_RRT_STAR : public RRT_STAR
{
public:
	Informed_RRT_STAR(std::pair<float, float> mapsize);
	bool PathSearch_IN_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal);
	std::pair<float, float> Informed_SampleNode(float Cmax, float Cmin,
		std::vector<std::vector<float>> Xcenter, std::vector<std::vector<float>> C);
	std::vector<std::vector<float>> SampleUnitBall();  //单位圆内采样
};

#endif // !_INFORMED_RRT_STAR_
