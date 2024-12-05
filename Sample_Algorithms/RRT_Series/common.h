#pragma once
#ifndef _COMMON_H_
#define _COMMON_H_
#include <iostream>
#include <list>
#include <vector>
#include <random>
#include <stack>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cmath>

class Node
{
public:
	std::pair<float, float> NodePos;
	Node* Parents = NULL;
	float Cost = 0; //起点到该节点的代价（路径长度）
};

class Common
{
public:
	Node StartNode;
	Node GoalNode;
	std::pair<float, float> MapSize;
	float Step = 0.5f;
	float Goal_R = 0.3f; //目标点的容差半径
	
	std::vector<std::vector<float>> BlockCircleList; //(y,x,r)
	std::vector<std::vector<float>> BlockSquareList; //左下角顶点和右上顶点{y1,x1,y2,x2}
	/*千万不要用vector，因为原vector空间满之后，会开辟一个新的空间，原空间被释放
	也无法根据Parents指针找到父节点了*/
	std::list<Node> List;
	std::vector<Node*> PathNode; //用于存放最短路径的节点
	int ImageR = 16; //分辨率
	float GenerateSampleNode(float min, float max);  //生成随机采样点
	Node* findNearestNode(std::pair<float, float> sampleNode, float step); //找离随机采样点最近的节点
	bool CheckCollisoin(std::pair<float, float> samplenode, std::pair<float, float> nearnode); //碰撞检测
	void ShowPath(); //显示路径
	void ShowPathNew();
	float Cross2D(std::pair<float, float> vec1, std::pair<float, float> vec2); //计算二维向量叉积
	bool CheckIntersect(std::pair<float, float> A, std::pair<float, float> B,
		std::pair<float, float> V1, std::pair<float, float> V2); //判断是否交叉
	bool isInBlock(std::pair<float, float> samplenode); //判断采样点是否在障碍物内
};
#endif // !_COMMON_H_
