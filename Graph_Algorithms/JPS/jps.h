#pragma once
#ifndef _JPS_H_
#define _JPS_H_
#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
class Node
{
public:
	std::pair<int, int> NodePos; //节点坐标{y,x}
	std::vector<std::pair<int, int>> ForceNeighborList; //强迫邻居列表
	std::vector<std::pair<int, int>> SearchMotion; //探索方向列表
	double G, H, F; //G=起点到当前节点的实际代价，H当前节点到终点的代价（曼哈顿距离），F = G + H
	Node* Parents; //父节点的指针
	bool Block; //障碍物
	Node() :NodePos(0, 0), G(0), H(0), F(0), Parents(NULL), Block(false) {} //构造函数，成员赋值
	void PathCost(Node* currentnode, std::pair<int, int> goal_);
};

class JPS
{
public:
	std::vector<std::vector<Node>> Map; //节点地图
	std::pair<int, int> MapSize; //地图的尺寸 {y,x}
	std::pair<int, int> StartNode; //起点
	std::pair<int, int> GoalNode; //终点
	std::vector<Node> OpenList;
	std::vector<Node> CloseList;
	JPS(std::pair<int, int> mapsize); //构造函数，参数为地图尺寸
	void SetBlock(std::vector<std::pair<int, int>> block);    //设置障碍物
	bool SearchPath(std::pair<int, int> start, std::pair<int, int> goal);  //路径搜索
	//递归探索，flag：斜线扩展时调用的横向纵向扩展遇到强迫邻居无需添加
	bool Jump(Node currentnode, int motion[], int flag); 
	bool DelectForceNeighbor(Node currentnode,int motion[],int flag);   //检测强迫邻居
	void ShowMap();  //显示地图
	void ShowPath(); //显示最短路径

private:
	//探索方向，每个节点探索8个方向   顺时针探索顺序{y,x}
	int motions[8][2] = { {0,1},{-1,0},{0,-1},{1,0},
						  {1,1},{-1,1},{-1,-1},{1,-1} };
};
#endif // !_JPS_H_
