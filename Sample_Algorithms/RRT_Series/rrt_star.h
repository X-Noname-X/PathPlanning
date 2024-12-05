#pragma once
#ifndef _RRT_STAR_H_
#define _RRT_STAR_H_
#include "common.h"

class RRT_STAR : public Common
{
public:
	RRT_STAR(std::pair<float, float> mapsize);
	bool PathSearch_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal);
	std::vector<Node*> findNodeInR(Node& newNode); //找出新节点半径R范围内的节点
	void ChooseParents(Node& newnode, std::vector<Node*>& nearnodelist); //为新节点选择父节点
	void Rewire(Node& newnode, std::vector<Node*>& nearnodelist);  //重连
};
#endif // !_RRT_STAR_H_
