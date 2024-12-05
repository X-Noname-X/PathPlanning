#include "rrt.h"

RRT::RRT(std::pair<float, float> mapsize)
{
	this->MapSize = mapsize;
}

bool RRT::PathSearch(std::pair<float, float> start, std::pair<float, float> goal)
{
	cv::Mat background(MapSize.first * ImageR, MapSize.second * ImageR, CV_8UC3, cv::Scalar(200, 200, 200));//(y,x)
	//起点可视化
	cv::circle(background,cv::Point(start.second*ImageR,start.first*ImageR),
		8, cv::Scalar(255,0,0),-1); 
	//终点可视化
	cv::circle(background, cv::Point(goal.second * ImageR, goal.first * ImageR),
		8, cv::Scalar(0, 0, 255), -1); 
	//障碍物可视化
	for(int i = 0; i < BlockSquareList.size(); i++)
		cv::rectangle(background, cv::Point(BlockSquareList[i][1]* ImageR, BlockSquareList[i][0]* ImageR),
			cv::Point(BlockSquareList[i][3]*ImageR, BlockSquareList[i][2]* ImageR),
			cv::Scalar(128, 128, 128), -1);
	cv::imshow("RRT", background);
	cv::waitKey(5);
	this->StartNode.NodePos = start;
	this->List.push_back(this->StartNode); //起点加入节点列表
	int iteration = 0; // 迭代的次数
	while (iteration < 3000)
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
				if (!isInBlock(SampleNode))
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
		//生长方向可视化
		cv::line(background, cv::Point(newPos.second * ImageR, newPos.first * ImageR),
			cv::Point(nearestPos->NodePos.second * ImageR, nearestPos->NodePos.first * ImageR),
			cv::Scalar(0,128,0),2);
		cv::imshow("RRT", background);
		cv::waitKey(5);
		//判断是否为目标点
		if (sqrt(pow(goal.first - newPos.first, 2) + pow(goal.second - newPos.second, 2)) < Goal_R)
		{
			this->GoalNode.NodePos = newPos;
			this->GoalNode.Parents = nearestPos;
			this->GoalNode.Cost = nearestPos->Cost + sqrt(pow(this->GoalNode.NodePos.first - nearestPos->NodePos.first, 2) +
				pow(this->GoalNode.NodePos.second - nearestPos->NodePos.second, 2));
			std::cout << "迭代次数:" << iteration << std::endl;
			Node tmp_node = this->GoalNode;
			while (tmp_node.Parents != NULL) //起点的父节点指针指向NULL
			{
				cv::line(background, cv::Point(tmp_node.NodePos.second * ImageR, tmp_node.NodePos.first * ImageR),
					cv::Point(tmp_node.Parents->NodePos.second * ImageR, tmp_node.Parents->NodePos.first * ImageR),
					cv::Scalar(0, 0, 255), 3);
				tmp_node = *(tmp_node.Parents);
			}
			this->ShowPath();
			std::cout << "----------路径长度----------" << std::endl;
			std::cout << this->GoalNode.Cost << std::endl;
			break;
		}
		//新节点加入列表
		Node newNode;
		newNode.NodePos = newPos;
		newNode.Parents = nearestPos; //最近的点作为新节点的父节点
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
