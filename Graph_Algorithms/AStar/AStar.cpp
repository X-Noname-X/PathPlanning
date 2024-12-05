#include "AStar.h"

//·�����ۼ���
void Node::PlanCost(Node *currentnode,std::pair<int,int> goal_)
{
	this->Parents = currentnode;
	this->G = currentnode->G + 1;
	this->H = abs(this->NodePos.first - goal_.first) + abs(this->NodePos.second - goal_.second);
	this->F = this->G + this->H;
}

//A*�Ĺ��캯�������ڳ�ʼ����ͼ
AStar::AStar(std::pair<int, int> mapsize)
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

void AStar::SetBlock(std::vector<std::pair<int,int>> block)
{
	//�����ϰ�������������ͼ�ж�Ӧ�Ľڵ���Ϊ�ϰ���
	for (std::vector<std::pair<int, int>>::iterator it = block.begin(); it != block.end(); it++)
	{
		this->Map[it->first][it->second].Block = true;
	}
}

//����������ȣ�������==��
bool operator==(const Node& node, const std::pair<int, int> snode)
{
	if (node.NodePos == snode) return true;
	else return false;
}

bool AStar::SearchPath(std::pair<int, int> start, std::pair<int, int> goal)
{
	this->StartNode = start;
	this->GoalNode = goal;
	this->OpenList.push_back(this->Map[start.first][start.second]);  //�Ƚ�������open����
	 //����º������ڱȽ�����Node�����fֵ
	struct CompareNodes { bool operator()(const Node& lf, const Node& rf) const {return lf.F < rf.F;}};
	
	while(!this->OpenList.empty()) //open��Ϊ�վͼ�������
	{
		//open������FֵС�Ľڵ���ǰ
		std::sort(this->OpenList.begin(), this->OpenList.end(), CompareNodes());
		//ȡ��Fֵ��С�Ľڵ㣬��open����Ԫ��
		Node fLeastNode = this->OpenList[0];
		this->OpenList.erase(this->OpenList.begin()); //�Ƴ�open����Ԫ��
		this->CloseList.push_back(fLeastNode); //��ӵ�close����
		//�жϸýڵ��Ƿ�Ϊ�յ㣬�Ƿ���true�����������װ
		if (fLeastNode.NodePos == goal)
			return true;

		//����������4������̽��
		for (int i = 0; i < 4; i++)
		{
			std::pair<int, int> SearchNode;
			SearchNode.first = fLeastNode.NodePos.first + this->motions[i][0];
			SearchNode.second = fLeastNode.NodePos.second + this->motions[i][1];
			
			//�ж������ڵ��Ƿ��ڵ�ͼ�ڣ������������˴�����
			if (SearchNode.first < 0 || SearchNode.first >= this->MapSize.first ||
				SearchNode.second < 0 || SearchNode.second >= this->MapSize.second)
				continue;
			//�ж������ڵ��Ƿ����ϰ��������close����
			if (this->Map[SearchNode.first][SearchNode.second].Block ||
				find(CloseList.begin(),CloseList.end(), SearchNode) != CloseList.end())
				continue;
			
			//�ж��Ƿ���open����
			if (find(OpenList.begin(), OpenList.end(), SearchNode) == OpenList.end())//����open��
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
	//open��Ϊ�պ󣬻�û������������򷵻�false
	return false;
}

void AStar::ShowMap()
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

void AStar::ShowPath()
{
	std::stack<Node> Stk; //����һ��ջ��ʹ·���ڵ���������ʾ
	Node PlanNode = this->Map[this->GoalNode.first][this->GoalNode.second];
	Stk.push(PlanNode); //�յ�����ջ
	while(true)
	{
		PlanNode = *(PlanNode.Parents);
		Stk.push(PlanNode);
		if (PlanNode.NodePos == this->StartNode)
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