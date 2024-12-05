#include "common.h"

bool Common::isInBlock(std::pair<float, float> samplenode)
{
	//遍历方形障碍物
	for (int i = 0; i < BlockSquareList.size(); i++)
	{
		if ((samplenode.first >= BlockSquareList[i][2] && samplenode.first <= BlockSquareList[i][0])
			&& (samplenode.second >= BlockSquareList[i][1] && samplenode.second <= BlockSquareList[i][3]))
			return true;
	}
	//遍历圆形障碍物

	return false;
}

const float EPSILON = 1e-4;
float Common::Cross2D(std::pair<float, float> vec1, std::pair<float, float> vec2)
{
	//x1*y2 - x2*y1
	return vec1.second * vec2.first - vec2.second * vec1.first;
}

bool Common::CheckIntersect(std::pair<float, float> A, std::pair<float, float> B,
	std::pair<float, float> V1, std::pair<float, float> V2)
{
	//检测线段AB与线段V12是否相交,判断V1, V2与线段AB的位置
	std::pair<float, float> AB = { B.first - A.first,B.second - A.second };
	std::pair<float, float> AV1 = { V1.first - A.first,V1.second - A.second };
	std::pair<float, float> AV2 = { V2.first - A.first,V2.second - A.second };
	float result1 = Cross2D(AB, AV1); // 叉积
	float result2 = Cross2D(AB, AV2);
	if (fabs(result1) > EPSILON && fabs(result2) > EPSILON) //(result1!=0 && result2!=0)
	{
		//同号
		if (result1 * result2 > 0)
			return false;
	}
	//两线段共线(result1 = result2 = 0)视为不相交,某点在线段上(result1 = 0 || result2 = 0)视为不相交
	else
		return false;

	//判断A, B与线段V12的位置
	std::pair<float, float> V12 = { V2.first - V1.first,V2.second - V1.second };
	std::pair<float, float> V1A = { A.first - V1.first,A.second - V1.second };
	std::pair<float, float> V1B = { B.first - V1.first,B.second - V1.second };
	result1 = Cross2D(V12, V1A);
	result2 = Cross2D(V12, V1B);
	if (fabs(result1) > EPSILON && fabs(result2) > EPSILON) //(result1!=0 && result2!=0)
	{
		if (result1 * result2 > 0)
			return false;
	}
	else
		return false;
	//不是上面的所有情况，则两线段相交
	return true;
}

bool Common::CheckCollisoin(std::pair<float, float> samplenode, std::pair<float, float> nearnode)
{
	//检测线段是否与方形障碍物的各边相交，只要与一条边相交，就判定为碰撞
	for (int i = 0; i < BlockSquareList.size(); i++)
	{
		std::vector<std::pair<float, float>> Point_ABCD{
			{BlockSquareList[i][0],BlockSquareList[i][1]}, //A
			{BlockSquareList[i][0],BlockSquareList[i][3]}, //B
			{BlockSquareList[i][2],BlockSquareList[i][3]}, //C
			{BlockSquareList[i][2],BlockSquareList[i][1]} }; //D

		for (int j = 0; j < 4; j++)
		{
			//AB BC CD DA
			if (CheckIntersect(samplenode, nearnode, Point_ABCD[j], Point_ABCD[(j + 1) % 4]))
				return true;
		}
	}

	//检测线段是否与圆形障碍物相交


	//线段与方形、圆形障碍物都没有相交
	return false;
}

float Common::GenerateSampleNode(float min, float max)
{
	std::random_device rd;  // 生成随机种子
	std::mt19937 gen(rd()); // 使用 Mersenne Twister 算法生成随机数，种子来自rd
	std::uniform_real_distribution<> dis(min, max); // 创建在[min,max)的均匀分布

	return dis(gen);
}

Node* Common::findNearestNode(std::pair<float, float> sampleNode, float step)
{
	Node* NearestP = NULL;
	double min_dis = std::numeric_limits<double>::infinity();
	double distance = 0;
	for (std::list<Node>::iterator it = this->List.begin(); it != this->List.end(); it++)
	{
		distance = sqrt(pow(sampleNode.first - it->NodePos.first, 2) + pow(sampleNode.second - it->NodePos.second, 2));
		if (distance < min_dis)
		{
			min_dis = distance;
			NearestP = &(*it);
		}
	}
	if (min_dis < step)
		this->Step = min_dis;
	else
		this->Step = step;

	return NearestP;
}

void Common::ShowPath()
{
	std::stack<Node> Stk; //定义一个栈，使路径节点能正向显示
	Node PathNode = this->GoalNode;
	Stk.push(PathNode); //终点先入栈
	while (true)
	{
		PathNode = *(PathNode.Parents);
		Stk.push(PathNode);
		if (PathNode.NodePos == this->StartNode.NodePos)
			break;
	}
	std::cout << "----------显示路径----------" << std::endl;
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

void Common::ShowPathNew()
{
	std::cout << "----------显示路径----------" << std::endl;
	for (int i = this->PathNode.size() - 1; i >= 0; --i) {
		if (i != 0)
			std::cout << "(" << this->PathNode[i]->NodePos.first << "," << this->PathNode[i]->NodePos.second << ")" << "->";
		else
			std::cout << "(" << this->PathNode[i]->NodePos.first << "," << this->PathNode[i]->NodePos.second << ")";
	}
	std::cout << std::endl;
}