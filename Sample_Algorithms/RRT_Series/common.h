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
	float Cost = 0; //��㵽�ýڵ�Ĵ��ۣ�·�����ȣ�
};

class Common
{
public:
	Node StartNode;
	Node GoalNode;
	std::pair<float, float> MapSize;
	float Step = 0.5f;
	float Goal_R = 0.3f; //Ŀ�����ݲ�뾶
	
	std::vector<std::vector<float>> BlockCircleList; //(y,x,r)
	std::vector<std::vector<float>> BlockSquareList; //���½Ƕ�������϶���{y1,x1,y2,x2}
	/*ǧ��Ҫ��vector����Ϊԭvector�ռ���֮�󣬻Ὺ��һ���µĿռ䣬ԭ�ռ䱻�ͷ�
	Ҳ�޷�����Parentsָ���ҵ����ڵ���*/
	std::list<Node> List;
	std::vector<Node*> PathNode; //���ڴ�����·���Ľڵ�
	int ImageR = 16; //�ֱ���
	float GenerateSampleNode(float min, float max);  //�������������
	Node* findNearestNode(std::pair<float, float> sampleNode, float step); //�����������������Ľڵ�
	bool CheckCollisoin(std::pair<float, float> samplenode, std::pair<float, float> nearnode); //��ײ���
	void ShowPath(); //��ʾ·��
	void ShowPathNew();
	float Cross2D(std::pair<float, float> vec1, std::pair<float, float> vec2); //�����ά�������
	bool CheckIntersect(std::pair<float, float> A, std::pair<float, float> B,
		std::pair<float, float> V1, std::pair<float, float> V2); //�ж��Ƿ񽻲�
	bool isInBlock(std::pair<float, float> samplenode); //�жϲ������Ƿ����ϰ�����
};
#endif // !_COMMON_H_
