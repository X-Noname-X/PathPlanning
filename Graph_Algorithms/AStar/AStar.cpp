#include "AStar.h"

//路径代价计算
void Node::PlanCost(Node *currentnode,std::pair<int,int> goal_)
{
	this->Parents = currentnode;
	this->G = currentnode->G + 1;
	this->H = abs(this->NodePos.first - goal_.first) + abs(this->NodePos.second - goal_.second);
	this->F = this->G + this->H;
}

//A*的构造函数，用于初始化地图
AStar::AStar(std::pair<int, int> mapsize)
{
	this->MapSize = mapsize;
	for (int i = 0; i < MapSize.first; i++)
	{
		std::vector<Node> mapline; //初始化内嵌容器，存储地图的每一行
		for (int j = 0; j < MapSize.second; j++)
		{
			Node node; //初始化节点
			//节点坐标赋值
			node.NodePos.first = i;
			node.NodePos.second = j;
			//将节点放进内嵌容器中
			mapline.push_back(node);
		}
		this->Map.push_back(mapline); //将每一行内嵌容器放进地图中
	}
}

void AStar::SetBlock(std::vector<std::pair<int,int>> block)
{
	//遍历障碍物容器，将地图中对应的节点设为障碍物
	for (std::vector<std::pair<int, int>>::iterator it = block.begin(); it != block.end(); it++)
	{
		this->Map[it->first][it->second].Block = true;
	}
}

//定义坐标相等，即重载==号
bool operator==(const Node& node, const std::pair<int, int> snode)
{
	if (node.NodePos == snode) return true;
	else return false;
}

bool AStar::SearchPath(std::pair<int, int> start, std::pair<int, int> goal)
{
	this->StartNode = start;
	this->GoalNode = goal;
	this->OpenList.push_back(this->Map[start.first][start.second]);  //先将起点放入open表中
	 //定义仿函数用于比较两个Node对象的f值
	struct CompareNodes { bool operator()(const Node& lf, const Node& rf) const {return lf.F < rf.F;}};
	
	while(!this->OpenList.empty()) //open表不为空就继续搜索
	{
		//open表排序，F值小的节点在前
		std::sort(this->OpenList.begin(), this->OpenList.end(), CompareNodes());
		//取出F值最小的节点，即open表首元素
		Node fLeastNode = this->OpenList[0];
		this->OpenList.erase(this->OpenList.begin()); //移除open表首元素
		this->CloseList.push_back(fLeastNode); //添加到close表中
		//判断该节点是否为终点，是返回true，否则继续安装
		if (fLeastNode.NodePos == goal)
			return true;

		//向上下左右4个方向探索
		for (int i = 0; i < 4; i++)
		{
			std::pair<int, int> SearchNode;
			SearchNode.first = fLeastNode.NodePos.first + this->motions[i][0];
			SearchNode.second = fLeastNode.NodePos.second + this->motions[i][1];
			
			//判断搜索节点是否在地图内，不在则跳出此次搜索
			if (SearchNode.first < 0 || SearchNode.first >= this->MapSize.first ||
				SearchNode.second < 0 || SearchNode.second >= this->MapSize.second)
				continue;
			//判断搜索节点是否是障碍物，或者在close表内
			if (this->Map[SearchNode.first][SearchNode.second].Block ||
				find(CloseList.begin(),CloseList.end(), SearchNode) != CloseList.end())
				continue;
			
			//判断是否在open表内
			if (find(OpenList.begin(), OpenList.end(), SearchNode) == OpenList.end())//不在open表
			{
				this->Map[SearchNode.first][SearchNode.second].PlanCost(
					&(this->Map[fLeastNode.NodePos.first][fLeastNode.NodePos.second]),goal);
				OpenList.push_back(this->Map[SearchNode.first][SearchNode.second]);
			}
			else
			{
				if ((this->Map[fLeastNode.NodePos.first][fLeastNode.NodePos.second].G + 1)
					< this->Map[SearchNode.first][SearchNode.second].G)
				{
					this->Map[SearchNode.first][SearchNode.second].PlanCost(
						&(this->Map[fLeastNode.NodePos.first][fLeastNode.NodePos.second]), goal);
				}
			}
		}
	}
	//open表为空后，还没有搜索结果，则返回false
	return false;
}

void AStar::ShowMap()
{
	std::cout << "----------显示地图----------" << std::endl;
	//下面遍历用到的[]是重载过的，因此用法类似数组
	for (int i = this->MapSize.first - 1; i >= 0; i--)
	{
		for (int j = 0; j < this->MapSize.second; j++)
		{
			std::cout << "(" << this->Map[i][j].NodePos.first << "," << this->Map[i][j].NodePos.second
				<< ")" << "/" << this->Map[i][j].Block << " ";
		}
		std::cout << std::endl;
	}
}

void AStar::ShowPath()
{
	std::stack<Node> Stk; //定义一个栈，使路径节点能正向显示
	Node PlanNode = this->Map[this->GoalNode.first][this->GoalNode.second];
	Stk.push(PlanNode); //终点先入栈
	while(true)
	{
		PlanNode = *(PlanNode.Parents);
		Stk.push(PlanNode);
		if (PlanNode.NodePos == this->StartNode)
			break;
	}
	std::cout << "----------显示最短路径----------" << std::endl;
	while (!Stk.empty())
	{
		if (Stk.size() != 1)
		{
			std::cout << "(" << Stk.top().NodePos.first << "," 
					  << Stk.top().NodePos.second << ")" << " -> ";
		}
		else
		{
			std::cout << "(" << Stk.top().NodePos.first << ","
				<< Stk.top().NodePos.second << ")";
		}
		Stk.pop(); //栈顶元素出栈
	}
	std::cout << std::endl;
}