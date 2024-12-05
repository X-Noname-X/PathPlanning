#include "rrt_star.h"

RRT_STAR::RRT_STAR(std::pair<float, float> mapsize)
{
	this->MapSize = mapsize;
}

bool RRT_STAR::PathSearch_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal)
{
	cv::Mat background(MapSize.first * ImageR, MapSize.second * ImageR, CV_8UC3, cv::Scalar(200, 200, 200));
	//�����ӻ�
	cv::circle(background, cv::Point(start.second * ImageR, start.first * ImageR),
		8, cv::Scalar(255, 0, 0), -1);
	//�յ���ӻ�
	cv::circle(background, cv::Point(goal.second * ImageR, goal.first * ImageR),
		8, cv::Scalar(0, 0, 255), -1);
	//�ϰ�����ӻ�
	for (int i = 0; i < BlockSquareList.size(); i++)
		cv::rectangle(background, cv::Point(BlockSquareList[i][1] * ImageR, BlockSquareList[i][0] * ImageR),
			cv::Point(BlockSquareList[i][3] * ImageR, BlockSquareList[i][2] * ImageR),
			cv::Scalar(128, 128, 128), -1);
	cv::imshow("RRT_STAR", background);
	cv::waitKey(5);
	this->StartNode.NodePos = start;
	this->List.push_back(this->StartNode); //������ڵ��б�
	float LastPathCost = 1000.0f; //�ȶ���һ���ܴ���ֵ������·������
	int iteration = 0; // �����Ĵ���
	while (iteration < 4500)
	{
		std::pair<float, float> SampleNode;
		Node* nearestPos;
		std::pair<float, float> newPos;
		while (true)
		{
			while (true)
			{
				//������ɲ�����
				SampleNode.first = this->GenerateSampleNode(0, this->MapSize.first);
				SampleNode.second = this->GenerateSampleNode(0, this->MapSize.second);
				if (!isInBlock(SampleNode) && SampleNode.first >= 0.0f && SampleNode.second >= 0.0f)
					break;
			}
			//�ҵ��������������ĵ�
			nearestPos = this->findNearestNode(SampleNode, 0.5f);
			//��ײ���
			if (!CheckCollisoin(SampleNode, nearestPos->NodePos))
				break;
		}
		float theta = std::atan2(SampleNode.first - nearestPos->NodePos.first, SampleNode.second - nearestPos->NodePos.second);
		//�����½ڵ�����
		newPos.first = nearestPos->NodePos.first + this->Step * sin(theta);
		newPos.second = nearestPos->NodePos.second + this->Step * cos(theta);
		//�ж��Ƿ�ΪĿ���
		if (sqrt(pow(goal.first - newPos.first, 2) + pow(goal.second - newPos.second, 2)) < Goal_R)
		{
			this->GoalNode.NodePos = newPos;
			this->GoalNode.Parents = nearestPos;
			this->GoalNode.Cost = nearestPos->Cost + sqrt(pow(this->GoalNode.NodePos.first - nearestPos->NodePos.first, 2) +
				pow(this->GoalNode.NodePos.second - nearestPos->NodePos.second, 2));
			std::cout << "��������:" << iteration << std::endl;
			Node tmp_node = this->GoalNode;
			if (LastPathCost > this->GoalNode.Cost)
			{
				this->PathNode.clear();  //���ԭ�ȵ�·��
				this->PathNode.push_back(&this->GoalNode);
				LastPathCost = this->GoalNode.Cost;
				std::cout << LastPathCost << std::endl;
				while (tmp_node.Parents != NULL) //���ĸ��ڵ�ָ��ָ��NULL
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
		//�½ڵ�����б�
		Node newNode;
		newNode.NodePos = newPos;
		newNode.Parents = nearestPos;
		//�Ƚ�����ڵ㶨��Ϊ�½ڵ�ĸ��ڵ㣬��������ۣ�����R��Χ���޽ڵ�ʱ���½ڵ�û�и��ڵ�ʹ���
		newNode.Cost = nearestPos->Cost + sqrt(pow(newNode.NodePos.first - nearestPos->NodePos.first, 2) +
			pow(newNode.NodePos.second - nearestPos->NodePos.second, 2));
		std::vector<Node*> nearNodeList = this->findNodeInR(newNode);
		this->ChooseParents(newNode, nearNodeList);
		this->List.push_back(newNode);
		this->Rewire(newNode, nearNodeList);
		iteration++;
		//����������ӻ�
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
	//��õ�ǰ�������нڵ�
	int node_num = this->List.size();
	//���½ڵ�ΪԲ�ģ���뾶
	float newNode_R = 50.0f * sqrt(log(node_num) / node_num);
	std::vector<Node*> nearNodeList;
	//�������нڵ�
	for (std::list<Node>::iterator it = this->List.begin(); it != this->List.end(); it++)
	{
		float dis = pow(newnode.NodePos.first - it->NodePos.first, 2) + pow(newnode.NodePos.second - it->NodePos.second, 2);
		if (dis < pow(newNode_R, 2))
			nearNodeList.push_back(&(*it)); //�洢�ڵ��ָ��
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
			if (!CheckCollisoin(nearnodelist[i]->NodePos, newnode.NodePos)) //�õ���newNode֮�����ϰ�
			{
				d = sqrt(pow(newnode.NodePos.first - nearnodelist[i]->NodePos.first, 2) +
					pow(newnode.NodePos.second - nearnodelist[i]->NodePos.second, 2));
			}
			else
				d = 1000.0f; //���ϰ���һ���ܴ����ֵ
			dList.push_back(nearnodelist[i]->Cost + d); //��֤dList��nearnodelist����һһ��Ӧ
		}
		// ʹ�� std::min_element �ҵ���СԪ�صĵ�����
		auto min_it = std::min_element(dList.begin(), dList.end());
		// ������СԪ�ص�����
		int min_index = std::distance(dList.begin(), min_it);
		//���øõ�Ϊ�½ڵ�ĸ��ڵ�
		newnode.Parents = nearnodelist[min_index];
		newnode.Cost = *min_it;
	}
}

void RRT_STAR::Rewire(Node& newnode, std::vector<Node*>& nearnodelist)
{
	for (int i = 0; i < nearnodelist.size(); i++)
	{
		float d = 0;
		if (!CheckCollisoin(nearnodelist[i]->NodePos, newnode.NodePos)) //�õ���newNode֮�����ϰ�
		{
			d = sqrt(pow(newnode.NodePos.first - nearnodelist[i]->NodePos.first, 2) +
				pow(newnode.NodePos.second - nearnodelist[i]->NodePos.second, 2));
		}
		else
			d = 1000.0f; //���ϰ���һ���ܴ����ֵ
		//��������
		float rewire_cost = newnode.Cost + d;
		if (nearnodelist[i]->Cost > rewire_cost)
		{
			nearnodelist[i]->Parents = &(this->List.back());
			nearnodelist[i]->Cost = rewire_cost;
		}
	}
}