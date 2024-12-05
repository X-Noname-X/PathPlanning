#include "informed_rrt_star.h"

Informed_RRT_STAR::Informed_RRT_STAR(std::pair<float, float> mapsize) : RRT_STAR::RRT_STAR(mapsize)
{
	this->MapSize = mapsize;
}

const float PI = 3.1415926;
bool Informed_RRT_STAR::PathSearch_IN_RRTSTAR(std::pair<float, float> start, std::pair<float, float> goal)
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
	cv::imshow("Informed_RRT_STAR", background);
	cv::waitKey(5);
	this->StartNode.NodePos = start;
	this->List.push_back(this->StartNode); //起点加入节点列表
	int iteration = 0; // 迭代的次数
	float Cbest = 1000.0f;
	float Cmin = sqrt(pow(start.second - goal.second, 2) + pow(start.first - goal.first, 2)) - this->Goal_R;
	std::vector<std::vector<float>> Xcenter = { {(start.second + goal.second) / 2.0f},{(start.first + goal.first) / 2.0f},{0} };
	float e_theta = atan2((start.first - goal.first) / Cmin, (start.second - goal.second) / Cmin);
	//旋转矩阵
	std::vector<std::vector<float>> C = { {cos(e_theta),-sin(e_theta),0},{sin(e_theta),cos(e_theta),0},{0,0,1} };
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
				SampleNode = this->Informed_SampleNode(Cbest, Cmin, Xcenter, C);
				//要保证采样后的x和y坐标都大于0
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
			if (Cbest > this->GoalNode.Cost)
			{
				this->PathNode.clear();
				this->PathNode.push_back(&this->GoalNode);
				Cbest = this->GoalNode.Cost;
				std::cout << Cbest << std::endl;
				while (tmp_node.Parents != NULL) //起点的父节点指针指向NULL
				{
					this->PathNode.push_back(tmp_node.Parents);
					cv::line(background, cv::Point(tmp_node.NodePos.second * ImageR, tmp_node.NodePos.first * ImageR),
						cv::Point(tmp_node.Parents->NodePos.second * ImageR, tmp_node.Parents->NodePos.first * ImageR),
						cv::Scalar(0, 165, 255), 2);
					tmp_node = *(tmp_node.Parents);
				}
				float Csh = sqrt(pow(Cbest, 2) - pow(Cmin, 2));
				cv::ellipse(background, cv::Point(Xcenter[0][0] * ImageR, Xcenter[1][0] * ImageR),
					cv::Size(Cbest * ImageR / 2.0, Csh * ImageR / 2.0), e_theta * 180 / PI, 0.0, 360.0,
					cv::Scalar(0, 165, 255), 2);
				cv::imshow("Informed_RRT_STAR", background);
				cv::waitKey(3);
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
		cv::imshow("Informed_RRT_STAR", background);
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
		float Csh = sqrt(pow(Cbest, 2) - pow(Cmin, 2));
		cv::ellipse(background, cv::Point(Xcenter[0][0] * ImageR, Xcenter[1][0] * ImageR),
			cv::Size(Cbest * ImageR / 2.0, Csh * ImageR / 2.0), e_theta * 180 / PI, 0.0, 360.0,
			cv::Scalar(0, 0, 255), 2);
	}
	cv::imshow("Informed_RRT_STAR", background);
	cv::waitKey(0);
	cv::destroyAllWindows();
	return true;
}

//创建 h行l列的矩阵，并将初始各值设定为0
std::vector<std::vector<float>> creatmatrix(int h, int l)
{
	std::vector<std::vector<float>> v;
	for (int i = 0; i < h; i++)
	{
		std::vector<float>v1(l, 0);
		v.push_back(v1);
	}
	return v;
}

const float EPSILON = 1e-4;
//矩阵A+矩阵B=矩阵C，并返回
std::vector<std::vector<float>> plus(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B)
{
	int h = A.size();
	int l = A[0].size();
	std::vector<std::vector<float>> C;
	C = creatmatrix(h, l);
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < l; j++)
		{
			C[i][j] = A[i][j] + B[i][j];
			if (abs(C[i][j]) < EPSILON)
				C[i][j] = 0.0;
		}
	}
	return C;
}
//矩阵A*矩阵B=矩阵C，并返回
std::vector<std::vector<float>> multiply(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B)
{
	int A_h = A.size();
	int A_l = A[0].size();
	int B_h = B.size();
	int B_l = B[0].size();
	if (A_l != B_h)
	{
		std::cout << "两矩阵维数无法相乘" << std::endl;
		exit(0);
	}
	std::vector<std::vector<float>> C = creatmatrix(A_h, B_l);
	for (int i = 0; i < A_h; i++)
	{
		for (int j = 0; j < B_l; j++)
		{
			C[i][j] = 0;
			for (int k = 0; k < A_l; k++)
			{
				C[i][j] += A[i][k] * B[k][j];
			}
			if (abs(C[i][j]) < EPSILON)
				C[i][j] = 0.0;
		}
	}
	return C;
}

std::pair<float, float> Informed_RRT_STAR::Informed_SampleNode(float Cmax, float Cmin,
	std::vector<std::vector<float>> Xcenter, std::vector<std::vector<float>> C)
{
	std::pair<float, float> samplenode;
	if (Cmax < 1000.0f)
	{
		//将单位圆压缩为椭圆的矩阵
		std::vector<std::vector<float>> L = { {Cmax / 2.0f,0,0},
											{0,(float)sqrt((pow(Cmax,2) - pow(Cmin,2)) / 2.0),0},
											{0,0,(float)sqrt((pow(Cmax,2) - pow(Cmin,2)) / 2.0)} };
		std::vector<std::vector<float>> Xball = this->SampleUnitBall();
		std::vector<std::vector<float>> samplenodeList = plus(multiply(multiply(C, L), Xball), Xcenter);
		samplenode.first = samplenodeList[1][0];
		samplenode.second = samplenodeList[0][0];
	}
	else
	{
		samplenode.first = this->GenerateSampleNode(0, this->MapSize.first);
		samplenode.second = this->GenerateSampleNode(0, this->MapSize.second);
	}
	return samplenode;
}

std::vector<std::vector<float>> Informed_RRT_STAR::SampleUnitBall()
{
	float a = this->GenerateSampleNode(0, 1);
	float b = this->GenerateSampleNode(0, 1);
	//始终保证b大于等于a
	if (b < a)
	{
		float tmp = b;
		b = a;
		a = tmp;
	}
	return { {b * cos(2 * PI * a / b)},{b * sin(2 * PI * a / b)},{0} };
}