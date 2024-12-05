#pragma once
#ifndef _RRT_H_
#define _RRT_H_
#include "common.h"

class RRT : public Common
{
public:
	RRT(std::pair<float, float> mapsize);
	bool PathSearch(std::pair<float, float> start, std::pair<float, float> goal);
};

#endif // !_RRT_H_

