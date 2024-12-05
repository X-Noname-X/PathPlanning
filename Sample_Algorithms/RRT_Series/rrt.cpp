#include "rrt.h"

RRT::RRT(std::pair<float, float> mapsize)
{
	this->MapSize = mapsize;
}

bool RRT::PathSearch(std::pair<float, float> start, std::pair<float, float> goal)
{
	cv::Mat background(MapSize.first * ImageR, MapSize.second * ImageR, CV_8UC3, cv::Scalar(200, 200, 200));//(y,x)
	//�����ӻ�
	cv::circle(background,cv::Point(start.second*ImageR,start.first*ImageR),
		8, cv::Scalar(255,0,0),-1); 
	//�յ���ӻ�
	cv::circle(background, cv::Point(goal.second * ImageR, goal.first * ImageR),
		8, cv::Scalar(0, 0, 255), -1); 
	//�ϰ�����ӻ�
	for(int i = 0; i < BlockSquareList.size(); i++)
		cv::rectangle(background, cv::Point(BlockSquareList[i][1]* ImageR, BlockSquareList[i][0]* ImageR),
			cv::Point(BlockSquareList[i][3]*ImageR, BlockSquareList[i][2]* ImageR),
			cv::Scalar(128, 128, 128), -1);
	cv::imshow("RRT", background);
	cv::waitKey(5);
	this->StartNode.NodePos = start;
	this->List.push_back(this->StartNode); //������ڵ��б�
	int iteration = 0; // �����Ĵ���
	while (iteration < 3000)
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
				if (!isInBlock(SampleNode))
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
		//����������ӻ�
		cv::line(background, cv::Point(newPos.second * ImageR, newPos.first * ImageR),
			cv::Point(nearestPos->NodePos.second * ImageR, nearestPos->NodePos.first * ImageR),
			cv::Scalar(0,128,0),2);
		cv::imshow("RRT", background);
		cv::waitKey(5);
		//�ж��Ƿ�ΪĿ���
		if (sqrt(pow(goal.first - newPos.first, 2) + pow(goal.second - newPos.second, 2)) < Goal_R)
		{
			this->GoalNode.NodePos = newPos;
			this->GoalNode.Parents = nearestPos;
			this->GoalNode.Cost = nearestPos->Cost + sqrt(pow(this->GoalNode.NodePos.first - nearestPos->NodePos.first, 2) +
				pow(this->GoalNode.NodePos.second - nearestPos->NodePos.second, 2));
			std::cout << "��������:" << iteration << std::endl;
			Node tmp_node = this->GoalNode;
			while (tmp_node.Parents != NULL) //���ĸ��ڵ�ָ��ָ��NULL
			{
				cv::line(background, cv::Point(tmp_node.NodePos.second * ImageR, tmp_node.NodePos.first * ImageR),
					cv::Point(tmp_node.Parents->NodePos.second * ImageR, tmp_node.Parents->NodePos.first * ImageR),
					cv::Scalar(0, 0, 255), 3);
				tmp_node = *(tmp_node.Parents);
			}
			this->ShowPath();
			std::cout << "----------·������----------" << std::endl;
			std::cout << this->GoalNode.Cost << std::endl;
			break;
		}
		//�½ڵ�����б�
		Node newNode;
		newNode.NodePos = newPos;
		newNode.Parents = nearestPos; //����ĵ���Ϊ�½ڵ�ĸ��ڵ�
		newNode.Cost = nearestPos->Cost + sqrt(pow(newNode.NodePos.first-nearestPos->NodePos.first,2) + 
			pow(newNode.NodePos.second - nearestPos->NodePos.second,2));
		this->List.push_back(newNode);
		iteration++;
	}
	cv::imshow("RRT", background);
	cv::waitKey(0);
	cv::destroyAllWindows();
	return true;
}
