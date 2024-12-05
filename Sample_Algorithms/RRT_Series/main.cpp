#include "rrt.h"
#include "rrt_star.h"
#include "informed_rrt_star.h"
#define RRT_ 1
#define RRT_STAR_ 2
#define INFORMED_RRT_STAR_ 3

int main()
{
	int flag = INFORMED_RRT_STAR_;
	if (flag == RRT_)
	{
		RRT rrt({ 15.0f,20.0f });
		rrt.BlockSquareList = { {4.0,3.0,0,5.0},{5.0,1.0,3.0,2.0},{8,6,3,8},{9.0,2,6.0,4.5},{10,8,8.5,8.5} };
		rrt.PathSearch({ 1.0f,1.6f }, { 4.5f,9.5f });
	}
	else if (flag == RRT_STAR_)
	{
		RRT_STAR rrt_star({ 15.0f,20.0f }); //{y,x}
		rrt_star.BlockSquareList = { {4.0,3.0,0,5.0},{5.0,1.0,3.0,2.0},{8,6,3,8},{9.0,2,6.0,4.5},{10,8,8.5,8.5} };
		rrt_star.PathSearch_RRTSTAR({ 1.0f,1.6f }, { 12.5f,19.5f });
	}
	else if (flag == INFORMED_RRT_STAR_)
	{
		Informed_RRT_STAR In_rrt_star({ 15.0f,20.0f });
		In_rrt_star.BlockSquareList = { {4.0,3.0,0,5.0},{5.0,1.0,3.0,2.0},{8,6,3,8},{9.0,2,6.0,4.5},{10,8,8.5,8.5},
		{4.0,10.0,0,12.0},{5.0,8.0,3.0,9.0},{8,13,3,15},{9.0,9,6.0,11.5},{10,14,8.5,14.5},
		{10.0,3.0,6,5.0},{11.0,1.0,9.0,2.0},{14,6,9,8},{15.0,2,12.0,4.5},{15,8,14.5,8.5} };
		In_rrt_star.PathSearch_IN_RRTSTAR({ 5.5f,10.0f }, { 12.5f,19.5f });
	}
	system("pause");
	return 0;
}