#pragma once
#ifndef _ASTAR_H_
#define _ASTAR_H_
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>

class Node
{
public:
	std::pair<int, int> NodePos; //�ڵ�����{y,x}
	int G, H, F; //G=��㵽��ǰ�ڵ��ʵ�ʴ��ۣ�H��ǰ�ڵ㵽�յ�Ĵ��ۣ������پ��룩��F = G + H
	Node* Parents; //���ڵ��ָ��
	bool Block; //�ϰ���
	Node() :NodePos(0, 0),G(0),H(0),F(0),Parents(NULL),Block(false){} //���캯������Ա��ֵ
	void PlanCost(Node* currentnode, std::pair<int, int> goal_); //·������
};

class AStar
{
public:
	std::vector<std::vector<Node>> Map; //�ڵ��ͼ
	std::pair<int, int> MapSize; //��ͼ�ĳߴ� {y,x}
	std::pair<int, int> StartNode; //���
	std::pair<int, int> GoalNode; //�յ�
	std::vector<Node> OpenList;
	std::vector<Node> CloseList;

	AStar(std::pair<int, int> mapsize); //���캯��������Ϊ��ͼ�ߴ�
	void SetBlock(std::vector<std::pair<int, int>> block);    //�����ϰ���
	bool SearchPath(std::pair<int, int> start, std::pair<int, int> goal);  //·������
	void ShowMap();  //��ʾ��ͼ
	void ShowPath(); //��ʾ���·��

private:
	//̽������ÿ���ڵ��̽����������4������   ˳ʱ��̽��˳��{y,x}
	int motions[4][2] = { {0,1},{-1,0},{0,-1},{1,0} };
};

#endif