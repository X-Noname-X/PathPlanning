#include "AStar.h"

int main()
{
	AStar astar({10,10}); //operater{}  
	astar.SetBlock({ {1,2},{1,3},{1,4},{1,5},{1,6},{1,7},
					 {7,1},{7,2},{7,3},{7,4},{7,5},{7,6},
					 {2,7},{3,7},{4,7},{6,7},{7,7},{8,7},{0,7},
					 {2,5},{3,5},{5,5},{6,5},{7,5}, 
					 {2,3},{3,3},{4,3},{5,3},{7,3}, });
	astar.ShowMap();
	bool path = astar.SearchPath({ 1,0 }, { 6,9 });
	if (path) astar.ShowPath();
	system("pause");
	return 0;
}