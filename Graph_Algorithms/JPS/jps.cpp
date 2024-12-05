#include "jps.h"

void Node::PathCost(Node* currentnode, std::pair<int, int> goal_)
{
	this->Parents = currentnode;
	this->H = abs(this->NodePos.first - goal_.first) + abs(this->NodePos.second - goal_.second);
	this->F = this->H;
}

JPS::JPS(std::pair<int, int> mapsize)
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

void JPS::SetBlock(std::vector<std::pair<int, int>> block)
{
	//遍历障碍物容器，将地图中对应的节点设为障碍物
	for (std::vector<std::pair<int, int>>::iterator it = block.begin(); it != block.end(); it++)
	{
		this->Map[it->first][it->second].Block = true;
	}
}
std::pair<int, int> _JumpNode_ = {0,0}; //全局
bool JPS::SearchPath(std::pair<int, int> start, std::pair<int, int> goal)
{
	this->StartNode = start;
	this->GoalNode = goal;
	//添加起点的遍历方向（8个方向都要遍历）
	for (auto& motion : this->motions)
		this->Map[this->StartNode.first][this->StartNode.second].SearchMotion.push_back({motion[0],motion[1]});
	this->OpenList.push_back(this->Map[start.first][start.second]);  //先将起点放入open表中
	//定义仿函数用于比较两个Node对象的f值
	struct CompareNodes { bool operator()(const Node& lf, const Node& rf) const { return lf.F < rf.F; } };

	while (!this->OpenList.empty())
	{
		std::sort(this->OpenList.begin(), this->OpenList.end(), CompareNodes());
		//取出F值最小的节点，即open表首元素
		Node fLeastNode = this->OpenList[0];
		this->OpenList.erase(this->OpenList.begin()); //移除open表首元素
		this->CloseList.push_back(fLeastNode);
		if (fLeastNode.NodePos == this->GoalNode)
			return true;
		for (auto& motion : fLeastNode.SearchMotion)
		{
			int temp_motion[2] = {motion.first,motion.second};
			bool jumpnode = Jump(fLeastNode, temp_motion,1);
			if (jumpnode)
			{
				if (find(OpenList.begin(), OpenList.end(), _JumpNode_) == OpenList.end()) //不在open表
				{
					this->Map[_JumpNode_.first][_JumpNode_.second].PathCost(
					&(this->Map[fLeastNode.NodePos.first][fLeastNode.NodePos.second]),this->GoalNode);
					this->OpenList.push_back(this->Map[_JumpNode_.first][_JumpNode_.second]);
				}
			}
		}
	}
	return false;
}

bool operator==(const Node& node, const std::pair<int, int> snode)
{
	if (node.NodePos == snode) return true;
	else return false;
}

int goal_flag = 0;
bool JPS::Jump(Node currentnode, int motion[],int flag)
{
	std::pair<int, int> JumpNode; //局部
	//节点坐标按方向递增
	JumpNode.first = currentnode.NodePos.first + motion[0];
	JumpNode.second = currentnode.NodePos.second + motion[1];
	//判断是否边界
	if (JumpNode.first < 0 || JumpNode.first >= this->MapSize.first ||
		JumpNode.second < 0 || JumpNode.second >= this->MapSize.second)
		return false;
	//判断是否为障碍物或者已在close表
	if (this->Map[JumpNode.first][JumpNode.second].Block ||
		find(CloseList.begin(), CloseList.end(), JumpNode) != CloseList.end())
		return false;
	//判断是否为目标点
	if (this->Map[JumpNode.first][JumpNode.second].NodePos == this->GoalNode)
	{
		if (flag) //独立直线扩展
		{
			_JumpNode_ = this->GoalNode;
		}
		goal_flag = 1;
		return true;
	}
	//检测强迫邻居
	bool fNeighbor = DelectForceNeighbor(this->Map[JumpNode.first][JumpNode.second],motion,flag);
	if (fNeighbor)
	{
		if (flag) {
			_JumpNode_ = JumpNode;
			//添加SearchMotion（扩展方向）父节点到该节点的扩展方向
			this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ motion[0],motion[1] });
			//以及该节点到强迫邻居的扩展方向（遍历强迫邻居列表）
			for (std::vector<std::pair<int, int>>::iterator it = this->Map[JumpNode.first][JumpNode.second].ForceNeighborList.begin();
				it != this->Map[JumpNode.first][JumpNode.second].ForceNeighborList.end(); it++)
				this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ it->first,it->second });
		}
		return true;
	}
	//如果是对角扩展，还需调用纵向和横向的扩展
	if (motion[0] != 0 && motion[1] != 0)
	{
		int y_motion[2] = {motion[0],0};
		int x_motion[2] = {0,motion[1]};
		if (Jump(this->Map[JumpNode.first][JumpNode.second], x_motion, 0) || 
			Jump(this->Map[JumpNode.first][JumpNode.second], y_motion, 0))
		{
			_JumpNode_ = JumpNode;
			this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ 0,motion[1] });
			this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ motion[0],0 });
			this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ motion[0],motion[1] });
			if (goal_flag)
			{
				goal_flag = 0;
				this->Map[this->GoalNode.first][this->GoalNode.second].Parents = &(this->Map[JumpNode.first][JumpNode.second]);
			}
			return true;
		}
	}
	return Jump(this->Map[JumpNode.first][JumpNode.second], motion, 1);
}

bool JPS::DelectForceNeighbor(Node currentnode,int motion[],int flag)
{
	int y = currentnode.NodePos.first;
	int x = currentnode.NodePos.second;
	//横向扩展
	if (motion[0] == 0 && motion[1] != 0)
	{
		//防止数组溢出
		if ((y+1>=0 && y+1<this->MapSize.first) && (x+motion[1] >= 0 && x+motion[1] < this->MapSize.second))
		{
			//强迫邻居判断
			if (this->Map[y + 1][x].Block == true && this->Map[y + 1][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({1,motion[1]}); //添加强迫邻居的方向到列表
				return true;
			}
		}
		//防止数组溢出
		if ((y - 1 >= 0 && y - 1 < this->MapSize.first) && (x + motion[1] >= 0 && x + motion[1] < this->MapSize.second))
		{
			//强迫邻居判断
			if (this->Map[y - 1][x].Block == true && this->Map[y - 1][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ -1,motion[1] }); //添加强迫邻居的方向到列表
				return true;
			}
		}
			
	}
	//纵向扩展
	else if (motion[0] != 0 && motion[1] == 0)
	{
		if ((x + 1 >= 0 && x + 1 < this->MapSize.second) && (y + motion[0] >= 0 && y + motion[0] < this->MapSize.first))
		{
			if (this->Map[y][x + 1].Block == true && this->Map[y + motion[0]][x + 1].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ motion[0],1 }); //添加强迫邻居的方向到列表
				return true;
			}
		}
		if ((x - 1 >= 0 && x - 1 < this->MapSize.second) && (y + motion[0] >= 0 && y + motion[0] < this->MapSize.first))
		{
			if (this->Map[y][x - 1].Block == true && this->Map[y + motion[0]][x - 1].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ motion[0],-1 }); 
				return true;
			}
		}
	}
	//对角扩展
	else if(motion[0] != 0 && motion[1] != 0)
	{
		if ((x - motion[1]>=0 && x - motion[1]<this->MapSize.second) && (y + motion[0]>=0 && y + motion[0] < this->MapSize.first))
		{
			if (this->Map[y][x - motion[1]].Block == true && this->Map[y + motion[0]][x - motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ motion[0],-motion[1]}); //添加强迫邻居的方向到列表
				return true;
			}
		}
		if ((x + motion[1] >= 0 && x + motion[1] < this->MapSize.second) && (y - motion[0] >= 0 && y - motion[0] < this->MapSize.first))
		{
			if (this->Map[y - motion[0]][x].Block == true && this->Map[y - motion[0]][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ -motion[0],motion[1] }); //添加强迫邻居的方向到列表
				return true;
			}
		}
	}
	return false;
}

void JPS::ShowMap()
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

void JPS::ShowPath()
{
	std::stack<Node> Stk; //定义一个栈，使路径节点能正向显示
	Node PathNode = this->Map[this->GoalNode.first][this->GoalNode.second];
	Stk.push(PathNode); //终点先入栈
	while (true)
	{
		PathNode = *(PathNode.Parents);
		Stk.push(PathNode);
		if (PathNode.NodePos == this->StartNode)
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