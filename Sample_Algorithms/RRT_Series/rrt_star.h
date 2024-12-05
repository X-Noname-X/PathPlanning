#pragma once
#ifndef _RRT_STAR_H_
#define _RRT_STAR_H_
#include "common.h"

class RRT_STAR : public Common
{
public:
	RRT_STAR(std::pair<float, float> mapsize);
	bool PathSearch_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal);
	std::vector<Node*> findNodeInR(Node& newNode); //�ҳ��½ڵ�뾶R��Χ�ڵĽڵ�
	void ChooseParents(Node& newnode, std::vector<Node*>& nearnodelist); //Ϊ�½ڵ�ѡ�񸸽ڵ�
	void Rewire(Node& newnode, std::vector<Node*>& nearnodelist);  //����
};
#endif // !_RRT_STAR_H_
