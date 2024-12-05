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
		std::vector<Node> mapline; //��ʼ����Ƕ�������洢��ͼ��ÿһ��
		for (int j = 0; j < MapSize.second; j++)
		{
			Node node; //��ʼ���ڵ�
			//�ڵ����긳ֵ
			node.NodePos.first = i;
			node.NodePos.second = j;
			//���ڵ�Ž���Ƕ������
			mapline.push_back(node);
		}
		this->Map.push_back(mapline); //��ÿһ����Ƕ�����Ž���ͼ��
	}
}

void JPS::SetBlock(std::vector<std::pair<int, int>> block)
{
	//�����ϰ�������������ͼ�ж�Ӧ�Ľڵ���Ϊ�ϰ���
	for (std::vector<std::pair<int, int>>::iterator it = block.begin(); it != block.end(); it++)
	{
		this->Map[it->first][it->second].Block = true;
	}
}
std::pair<int, int> _JumpNode_ = {0,0}; //ȫ��
bool JPS::SearchPath(std::pair<int, int> start, std::pair<int, int> goal)
{
	this->StartNode = start;
	this->GoalNode = goal;
	//������ı�������8������Ҫ������
	for (auto& motion : this->motions)
		this->Map[this->StartNode.first][this->StartNode.second].SearchMotion.push_back({motion[0],motion[1]});
	this->OpenList.push_back(this->Map[start.first][start.second]);  //�Ƚ�������open����
	//����º������ڱȽ�����Node�����fֵ
	struct CompareNodes { bool operator()(const Node& lf, const Node& rf) const { return lf.F < rf.F; } };

	while (!this->OpenList.empty())
	{
		std::sort(this->OpenList.begin(), this->OpenList.end(), CompareNodes());
		//ȡ��Fֵ��С�Ľڵ㣬��open����Ԫ��
		Node fLeastNode = this->OpenList[0];
		this->OpenList.erase(this->OpenList.begin()); //�Ƴ�open����Ԫ��
		this->CloseList.push_back(fLeastNode);
		if (fLeastNode.NodePos == this->GoalNode)
			return true;
		for (auto& motion : fLeastNode.SearchMotion)
		{
			int temp_motion[2] = {motion.first,motion.second};
			bool jumpnode = Jump(fLeastNode, temp_motion,1);
			if (jumpnode)
			{
				if (find(OpenList.begin(), OpenList.end(), _JumpNode_) == OpenList.end()) //����open��
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
	std::pair<int, int> JumpNode; //�ֲ�
	//�ڵ����갴�������
	JumpNode.first = currentnode.NodePos.first + motion[0];
	JumpNode.second = currentnode.NodePos.second + motion[1];
	//�ж��Ƿ�߽�
	if (JumpNode.first < 0 || JumpNode.first >= this->MapSize.first ||
		JumpNode.second < 0 || JumpNode.second >= this->MapSize.second)
		return false;
	//�ж��Ƿ�Ϊ�ϰ����������close��
	if (this->Map[JumpNode.first][JumpNode.second].Block ||
		find(CloseList.begin(), CloseList.end(), JumpNode) != CloseList.end())
		return false;
	//�ж��Ƿ�ΪĿ���
	if (this->Map[JumpNode.first][JumpNode.second].NodePos == this->GoalNode)
	{
		if (flag) //����ֱ����չ
		{
			_JumpNode_ = this->GoalNode;
		}
		goal_flag = 1;
		return true;
	}
	//���ǿ���ھ�
	bool fNeighbor = DelectForceNeighbor(this->Map[JumpNode.first][JumpNode.second],motion,flag);
	if (fNeighbor)
	{
		if (flag) {
			_JumpNode_ = JumpNode;
			//���SearchMotion����չ���򣩸��ڵ㵽�ýڵ����չ����
			this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ motion[0],motion[1] });
			//�Լ��ýڵ㵽ǿ���ھӵ���չ���򣨱���ǿ���ھ��б�
			for (std::vector<std::pair<int, int>>::iterator it = this->Map[JumpNode.first][JumpNode.second].ForceNeighborList.begin();
				it != this->Map[JumpNode.first][JumpNode.second].ForceNeighborList.end(); it++)
				this->Map[JumpNode.first][JumpNode.second].SearchMotion.push_back({ it->first,it->second });
		}
		return true;
	}
	//����ǶԽ���չ�������������ͺ������չ
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
	//������չ
	if (motion[0] == 0 && motion[1] != 0)
	{
		//��ֹ�������
		if ((y+1>=0 && y+1<this->MapSize.first) && (x+motion[1] >= 0 && x+motion[1] < this->MapSize.second))
		{
			//ǿ���ھ��ж�
			if (this->Map[y + 1][x].Block == true && this->Map[y + 1][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({1,motion[1]}); //���ǿ���ھӵķ����б�
				return true;
			}
		}
		//��ֹ�������
		if ((y - 1 >= 0 && y - 1 < this->MapSize.first) && (x + motion[1] >= 0 && x + motion[1] < this->MapSize.second))
		{
			//ǿ���ھ��ж�
			if (this->Map[y - 1][x].Block == true && this->Map[y - 1][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ -1,motion[1] }); //���ǿ���ھӵķ����б�
				return true;
			}
		}
			
	}
	//������չ
	else if (motion[0] != 0 && motion[1] == 0)
	{
		if ((x + 1 >= 0 && x + 1 < this->MapSize.second) && (y + motion[0] >= 0 && y + motion[0] < this->MapSize.first))
		{
			if (this->Map[y][x + 1].Block == true && this->Map[y + motion[0]][x + 1].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ motion[0],1 }); //���ǿ���ھӵķ����б�
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
	//�Խ���չ
	else if(motion[0] != 0 && motion[1] != 0)
	{
		if ((x - motion[1]>=0 && x - motion[1]<this->MapSize.second) && (y + motion[0]>=0 && y + motion[0] < this->MapSize.first))
		{
			if (this->Map[y][x - motion[1]].Block == true && this->Map[y + motion[0]][x - motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ motion[0],-motion[1]}); //���ǿ���ھӵķ����б�
				return true;
			}
		}
		if ((x + motion[1] >= 0 && x + motion[1] < this->MapSize.second) && (y - motion[0] >= 0 && y - motion[0] < this->MapSize.first))
		{
			if (this->Map[y - motion[0]][x].Block == true && this->Map[y - motion[0]][x + motion[1]].Block == false)
			{
				if(flag) this->Map[y][x].ForceNeighborList.push_back({ -motion[0],motion[1] }); //���ǿ���ھӵķ����б�
				return true;
			}
		}
	}
	return false;
}

void JPS::ShowMap()
{
	std::cout << "----------��ʾ��ͼ----------" << std::endl;
	//��������õ���[]�����ع��ģ�����÷���������
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
	std::stack<Node> Stk; //����һ��ջ��ʹ·���ڵ���������ʾ
	Node PathNode = this->Map[this->GoalNode.first][this->GoalNode.second];
	Stk.push(PathNode); //�յ�����ջ
	while (true)
	{
		PathNode = *(PathNode.Parents);
		Stk.push(PathNode);
		if (PathNode.NodePos == this->StartNode)
			break;
	}
	std::cout << "----------��ʾ���·��----------" << std::endl;
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
		Stk.pop(); //ջ��Ԫ�س�ջ
	}
	std::cout << std::endl;
}