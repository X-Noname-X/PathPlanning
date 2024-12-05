#include "rrt_star.h"

RRT_STAR::RRT_STAR(std::pair<float, float> mapsize)
{
	this->MapSize = mapsize;
}

bool RRT_STAR::PathSearch_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal)
{
	cv::Mat background(MapSize.first * ImageR, MapSize.second * ImageR, CV_8UC3, cv::Scalar(200, 200, 200));
	//起点可视化
	cv::circle(background, cv::Point(start.second * ImageR, start.first * ImageR),
		8, cv::Scalar(255, 0, 0), -1);
	//终点可视化
	cv::circle(background, cv::Point(goal.second * ImageR, goal.first * ImageR),
		8, cv::Scalar(0, 0, 255), -1);
	//障碍物可视化
	for (int i = 0; i < BlockSquareList.size(); i++)
		cv::rectangle(background, cv::Point(BlockSquareList[i][1] * ImageR, BlockSquareList[i][0] * ImageR),
			cv::Point(BlockSquareList[i][3] * ImageR, BlockSquareList[i][2] * ImageR),
			cv::Scalar(128, 128, 128), -1);
	cv::imshow("RRT_STAR", background);
	cv::waitKey(5);
	this->StartNode.NodePos = start;
	this->List.push_back(this->StartNode); //起点加入节点列表
	float LastPathCost = 1000.0f; //先定义一个很大数值的最终路径长度
	int iteration = 0; // 迭代的次数
	while (iteration < 4500)
	{
		std::pair<float, float> SampleNode;
		Node* nearestPos;
		std::pair<float, float> newPos;
		while (true)
		{
			while (true)
			{
				//随机生成采样点
				SampleNode.first = this->GenerateSampleNode(0, this->MapSize.first);
				SampleNode.second = this->GenerateSampleNode(0, this->MapSize.second);
				if (!isInBlock(SampleNode) && SampleNode.first >= 0.0f && SampleNode.second >= 0.0f)
					break;
			}
			//找到距离采样点最近的点
			nearestPos = this->findNearestNode(SampleNode, 0.5f);
			//碰撞检测
			if (!CheckCollisoin(SampleNode, nearestPos->NodePos))
				break;
		}
		float theta = std::atan2(SampleNode.first - nearestPos->NodePos.first, SampleNode.second - nearestPos->NodePos.second);
		//生成新节点坐标
		newPos.first = nearestPos->NodePos.first + this->Step * sin(theta);
		newPos.second = nearestPos->NodePos.second + this->Step * cos(theta);
		//判断是否为目标点
		if (sqrt(pow(goal.first - newPos.first, 2) + pow(goal.second - newPos.second, 2)) < Goal_R)
		{
			this->GoalNode.NodePos = newPos;
			this->GoalNode.Parents = nearestPos;
			this->GoalNode.Cost = nearestPos->Cost + sqrt(pow(this->GoalNode.NodePos.first - nearestPos->NodePos.first, 2) +
				pow(this->GoalNode.NodePos.second - nearestPos->NodePos.second, 2));
			std::cout << "迭代次数:" << iteration << std::endl;
			Node tmp_node = this->GoalNode;
			if (LastPathCost > this->GoalNode.Cost)
			{
				this->PathNode.clear();  //清空原先的路径
				this->PathNode.push_back(&this->GoalNode);
				LastPathCost = this->GoalNode.Cost;
				std::cout << LastPathCost << std::endl;
				while (tmp_node.Parents != NULL) //起点的父节点指针指向NULL
				{
					this->PathNode.push_back(tmp_node.Parents);
					cv::line(background, cv::Point(tmp_node.NodePos.second * ImageR, tmp_node.NodePos.first * ImageR),
						cv::Point(tmp_node.Parents->NodePos.second * ImageR, tmp_node.Parents->NodePos.first * ImageR),
						cv::Scalar(0, 165, 255), 2);
					tmp_node = *(tmp_node.Parents);
					cv::imshow("RRT_STAR", background);
					cv::waitKey(3);
				}
			}
		}
		//新节点加入列表
		Node newNode;
		newNode.NodePos = newPos;
		newNode.Parents = nearestPos;
		//先将最近节点定义为新节点的父节点，并计算代价，以免R范围内无节点时，新节点没有父节点和代价
		newNode.Cost = nearestPos->Cost + sqrt(pow(newNode.NodePos.first - nearestPos->NodePos.first, 2) +
			pow(newNode.NodePos.second - nearestPos->NodePos.second, 2));
		std::vector<Node*> nearNodeList = this->findNodeInR(newNode);
		this->ChooseParents(newNode, nearNodeList);
		this->List.push_back(newNode);
		this->Rewire(newNode, nearNodeList);
		iteration++;
		//生长方向可视化
		cv::line(background, cv::Point(newNode.NodePos.second * ImageR, newNode.NodePos.first * ImageR),
			cv::Point(newNode.Parents->NodePos.second * ImageR, newNode.Parents->NodePos.first * ImageR),
			cv::Scalar(0, 128, 0), 1);
		cv::imshow("RRT_STAR", background);
		cv::waitKey(3);
	}
	if (this->PathNode.size() != 0)
	{
		this->ShowPathNew();
		for (int i = 0; i < this->PathNode.size() - 1; i++)
		{
			cv::line(background, cv::Point(this->PathNode[i]->NodePos.second * ImageR, this->PathNode[i]->NodePos.first * ImageR),
				cv::Point(this->PathNode[i + 1]->NodePos.second * ImageR, this->PathNode[i + 1]->NodePos.first * ImageR),
				cv::Scalar(0, 0, 255), 3);
		}
	}
	cv::imshow("RRT_STAR", background);
	cv::waitKey(0);
	cv::destroyAllWindows();
	return true;
}

std::vector<Node*> RRT_STAR::findNodeInR(Node& newnode)
{
	//获得当前树的所有节点
	int node_num = this->List.size();
	//以新节点为圆心，求半径
	float newNode_R = 50.0f * sqrt(log(node_num) / node_num);
	std::vector<Node*> nearNodeList;
	//遍历所有节点
	for (std::list<Node>::iterator it = this->List.begin(); it != this->List.end(); it++)
	{
		float dis = pow(newnode.NodePos.first - it->NodePos.first, 2) + pow(newnode.NodePos.second - it->NodePos.second, 2);
		if (dis < pow(newNode_R, 2))
			nearNodeList.push_back(&(*it)); //存储节点的指针
	}
	return nearNodeList;
}

void RRT_STAR::ChooseParents(Node& newnode, std::vector<Node*>& nearnodelist)
{
	if (!nearnodelist.empty())
	{
		std::vector<float> dList;
		for (int i = 0; i < nearnodelist.size(); i++)
		{
			float d = 0;
			if (!CheckCollisoin(nearnodelist[i]->NodePos, newnode.NodePos)) //该点与newNode之间无障碍
			{
				d = sqrt(pow(newnode.NodePos.first - nearnodelist[i]->NodePos.first, 2) +
					pow(newnode.NodePos.second - nearnodelist[i]->NodePos.second, 2));
			}
			else
				d = 1000.0f; //有障碍，一个很大的数值
			dList.push_back(nearnodelist[i]->Cost + d); //保证dList和nearnodelist索引一一对应
		}
		// 使用 std::min_element 找到最小元素的迭代器
		auto min_it = std::min_element(dList.begin(), dList.end());
		// 计算最小元素的索引
		int min_index = std::distance(dList.begin(), min_it);
		//设置该点为新节点的父节点
		newnode.Parents = nearnodelist[min_index];
		newnode.Cost = *min_it;
	}
}

void RRT_STAR::Rewire(Node& newnode, std::vector<Node*>& nearnodelist)
{
	for (int i = 0; i < nearnodelist.size(); i++)
	{
		float d = 0;
		if (!CheckCollisoin(nearnodelist[i]->NodePos, newnode.NodePos)) //该点与newNode之间无障碍
		{
			d = sqrt(pow(newnode.NodePos.first - nearnodelist[i]->NodePos.first, 2) +
				pow(newnode.NodePos.second - nearnodelist[i]->NodePos.second, 2));
		}
		else
			d = 1000.0f; //有障碍，一个很大的数值
		//重连代价
		float rewire_cost = newnode.Cost + d;
		if (nearnodelist[i]->Cost > rewire_cost)
		{
			nearnodelist[i]->Parents = &(this->List.back());
			nearnodelist[i]->Cost = rewire_cost;
		}
	}
}