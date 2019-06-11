#include "path_planning_gr17.h"
#include "init_pos_gr17.h"
#include "opp_pos_gr17.h"
#include "useful_gr17.h"
#include <math.h>
#include <stdio.h>
using namespace std;
NAMESPACE_INIT(ctrlGr17);
#define MAX_Y 15
#define MAX_X 10

/* Planning based on the A star algorithm */


//x is in the horizontal direction ---- so second dimension 
// y is the the vertical direction |    so first dimension 
int MAP[15][10] = {
	{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
	{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
	{ 1, 1, 1, 0, 0, 0, 1, 1, 0, 0 },
	{ 1, 1, 1, 0, 0, 0, 0, 1, 0, 0 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 0, 0, 0, 0, 1, 1, 1, 0, 0, 0 },
	{ 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
	{ 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 },
	{ 0, 0, 0, 0, 1, 0, 1, 0, 0, 0 },
	{ 0, 0, 0, 0, 1, 1, 1, 0, 0, 0 },
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 },
	{ 1, 1, 1, 0, 0, 0, 0, 1, 0, 0 },
	{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
	{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 }
};

int node_index(new_row * open, int xval, int yval)
{
	int i = 0;
	while (open[i].xval != xval || open[i].yval != yval)
		i++;
	return i;
}
float distance(int x1, int y1, int x2, int y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
vector<new_row> expand_array(int node_x, int node_y, int gparent, int xTarget, int yTarget, vector<point> CLOSED)
{
	//This function takes a node and returns the expanded list
	//of successors, with the calculated fn values.
	//The criteria being none of the successors are on the CLOSED list
	vector<new_row> exp_array = {};
	exp_array.clear();
	bool flag;
	int exp_count = 0, k, j;
	int c1, c2 = CLOSED.size();
	int s_x, s_y;
	new_row a;
	for (k = 1; k > -2; k--) {
		for (j = 1; j > -2; j--) {
			if (k != j || k != 0) { //Test if node itself is not its successor
				s_x = node_x + k;
				s_y = node_y + j;
				if ((s_x >= 0 && s_x < MAX_X) && (s_y >= 0 && s_y < MAX_Y)) { //node within array bound
					flag = true;
					for (c1 = 0; c1 < c2; c1++) {
						if (s_x == CLOSED[c1].x && s_y == CLOSED[c1].y) flag = false;
					}
					if (MAP[s_y][node_x] == 1||MAP[node_y][s_x]==1) flag = false;
					if (flag == true) {
						a.xval = s_x;
						a.yval = s_y;
						a.gn = gparent + distance(node_x, node_y, s_x, s_y);
						a.hn = distance(s_x, s_y, xTarget, yTarget); //Euclidian
						a.fn = a.gn + a.hn;
						exp_array.push_back(a);

					}
				}
			}
		}
	}
	return exp_array;
}
int min(vector<new_row> temp_array)
{
	int i;
	int n = temp_array.size();
	new_row min = temp_array[0];
	int min_ind = 0;
	for (i = 1; i < n; i++)
	{
		if (temp_array[i].fn < min.fn)
		{
			min = temp_array[i];
			min_ind = i;
		}
	}
	return min_ind;
}

int min_fn(new_row *open, int open_count, int xTarget, int yTarget)
{
	int goal_index;
	int j;
	int i_min;
	vector<new_row> temp_array = {};
	temp_array.clear();
	vector<int> temp_array_j = {};
	temp_array_j.clear();
	int flag = 0;
	goal_index = 0;

	for (j = 0; j <= open_count; j++)
	{
		if (open[j].index == 1)
		{
			temp_array.push_back(open[j]);
			temp_array_j.push_back(j);
			if (open[j].xval == xTarget&&open[j].yval == yTarget)
			{
				flag = 1;
				goal_index = j;
			}
			
		}
	}
	if (flag == 1)
	{
		i_min = goal_index;
		return i_min;
	}
	if (temp_array.size() != 0)
	{
		int temp_min = min(temp_array);
		i_min = temp_array_j[temp_min];
	}
	else
		i_min = -1;
	return i_min;
}


/*! \brief initialize the path-planning algorithm (memory allocated)
*
* \param[in,out] path path-planning main structure
*/

PathPlanning* init_path_planning()
{
	PathPlanning *p;
	p = (PathPlanning*)malloc(sizeof(PathPlanning));
	return p;
}

PathPlanning* path_planning(CtrlStruct *cvs, float xSt, float ySt, float xTar, float yTar, int Sta)
{	
	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;
	int nb_opp;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	int Xopp, Yopp;

	if (Sta == 0)
	{
		Xopp = (int)floor((1 + opp_pos->x[0]) * 5);
		Yopp = (int)floor((1.5 - opp_pos->y[0]) * 5);
		MAP[Yopp][Xopp] = 1;
		if (Yopp != 14)
		{
			MAP[Yopp + 1][Xopp] = 1;
			if (Xopp != 9) MAP[Yopp + 1][Xopp + 1] = 1;
			if (Xopp != 0) MAP[Yopp + 1][Xopp - 1] = 1;
		}
		if (Yopp != 0)
		{
			MAP[Yopp - 1][Xopp] = 1;
			if (Xopp != 0) MAP[Yopp - 1][Xopp - 1] = 1;
		}

		if (Xopp != 0)	MAP[Yopp][Xopp - 1] = 1;
		if (Xopp != 9)
		{
			if (Yopp != 0) MAP[Yopp - 1][Xopp + 1] = 1;
			MAP[Yopp][Xopp + 1] = 1;
		}

		if (nb_opp == 2)
		{
			Xopp = (int)floor((1 + opp_pos->x[1]) * 5);
			Yopp = (int)floor((1.5 - opp_pos->y[1]) * 5);
			MAP[Yopp][Xopp] = 1;
			if (Yopp != 14)
			{
				MAP[Yopp + 1][Xopp] = 1;
				if (Xopp != 9) MAP[Yopp + 1][Xopp + 1] = 1;
				if (Xopp != 0) MAP[Yopp + 1][Xopp - 1] = 1;
			}
			if (Yopp != 0)
			{
				MAP[Yopp - 1][Xopp] = 1;
				if (Xopp != 0) MAP[Yopp - 1][Xopp - 1] = 1;
			}

			if (Xopp != 0)	MAP[Yopp][Xopp - 1] = 1;
			if (Xopp != 9)
			{
				if (Yopp != 0) MAP[Yopp - 1][Xopp + 1] = 1;
				MAP[Yopp][Xopp + 1] = 1;
			}
		}
	}

	//Transform to int to work in map;
	PathPlanning*pa = (PathPlanning*)malloc((sizeof(PathPlanning)));
	int xStart, xTarget, yStart, yTarget;
	xStart = (int)floor((1 + xSt) * 5);
	yStart = (int)floor((1.5 - ySt) * 5);
	xTarget = (int)floor((1 + xTar) * 5);
	yTarget= (int)floor((1.5 - yTar) * 5);

	MAP[yStart][xStart] = 2;
	
	int i;
	int j;
	int index_min_node;
	int size_closed;
	int xval;
	int yval;
	vector<point> closed;
	closed.clear();
	vector<point> path;
	path.clear();
	int inode;
	int parent_x;
	int parent_y;
	int flag;
	
	int k = 0;
	point temp;

	for (i = 0; i < MAX_X; i++)
	{
		for (j = 0; j < MAX_Y; j++)
		{
			if (MAP[j][i] == 1)
			{
				temp.x = i;
				temp.y = j;
				closed.push_back(temp);
				k++;
			}
		}
	}
	
	int closed_count = closed.size();
	int xNode = xStart;
	int yNode = yStart; // Robot position
	int open_count = 0;
	float path_cost = 0;
	float goal_distance = distance(xNode, yNode, xTarget, yTarget);

	new_row open[150];
	open[open_count].xval = xNode;
	open[open_count].yval = yNode;
	open[open_count].parent_xval = xNode;
	open[open_count].parent_yval = yNode;
	open[open_count].hn = path_cost;
	open[open_count].gn = goal_distance;
	open[open_count].fn = goal_distance;
	open[open_count].index = 0;
	closed_count++;
	
	point tem;
	tem.x = xNode;
	tem.y = yNode;
	closed.push_back(tem);
	int NoPath = 1;
	int loop = 0;

	//start algorithm
	while (!(xNode == xTarget && yNode == yTarget) && NoPath == 1)
	{
		vector<new_row> exp_array;
		exp_array.clear();
		exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, closed);

		int exp_count = exp_array.size();
		loop++;
		for (i = 0; i < exp_count; i++)
		{
			flag = 0;
			for (j = 0; j <= open_count; j++)
			{
				if (exp_array[i].xval == open[j].xval && exp_array[i].yval == open[j].yval)
				{
					if (open[j].fn >= exp_array[i].fn)
					{
						open[j].fn = exp_array[i].fn;
						open[j].parent_xval = xNode;
						open[j].parent_yval = yNode;
						open[j].hn = exp_array[i].hn;
						open[j].gn = exp_array[i].gn;
					}
					flag = 1;
				}
			}
			if (flag == 0)
			{

				open_count++;
				open[open_count].xval = exp_array[i].xval;
				open[open_count].yval = exp_array[i].yval;
				open[open_count].parent_xval = xNode;
				open[open_count].parent_yval = yNode;
				open[open_count].hn = exp_array[i].hn;
				open[open_count].gn = exp_array[i].gn;
				open[open_count].fn = exp_array[i].fn;
			}
		}
		index_min_node = min_fn(open, open_count, xTarget, yTarget);
		if (index_min_node != -1)
		{
			xNode = open[index_min_node].xval;
			yNode = open[index_min_node].yval;
			path_cost = open[index_min_node].gn;
			closed_count++;
			point tempo;
			tempo.x = xNode;
			tempo.y = yNode;
			closed.push_back(tempo);
			open[index_min_node].index = 0;
		}
		else
			NoPath = 0;
	}
	size_closed = closed.size();
	xval = closed[size_closed - 1].x;
	yval = closed[size_closed - 1].y;
	size_closed = 0;
	point t;
	t.x = xval;
	t.y = yval;
	path.insert(path.begin(),t);
	
	size_closed++;
	if ((xval == xTarget) && (yval == yTarget))
	{
		inode = 0;
		parent_x = open[node_index(open, xval, yval)].parent_xval;
		parent_y = open[node_index(open, xval, yval)].parent_yval;
		while (parent_x != xStart || parent_y != yStart)
		{
			point t;
			t.x = parent_x;
			t.y = parent_y;
			path.insert(path.begin(), t);
			inode = node_index(open, parent_x, parent_y);
			parent_x = open[inode].parent_xval;
			parent_y = open[inode].parent_yval;
			size_closed++;
		}
		
	}
	for (int i=0;i<path.size();i++)
	{
		MAP[int(path[i].y)][int(path[i].x)] = 8;
	}

	MAP[xStart][yStart] = 0;
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 10; j++)
			MAP[i][j] = 0;
	}
	MAP[0][7] = 1;
	MAP[1][7] = 1;
	MAP[2][7] = 1; MAP[2][0] = 1; MAP[2][1] = 1; MAP[2][2] = 1; MAP[2][6] = 1;
	MAP[3][0] = 1; MAP[3][1] = 1; MAP[3][2] = 1; MAP[3][7] = 1;
	MAP[5][4] = 1; MAP[5][5] = 1; MAP[5][6] = 1;
	MAP[6][4] = 1; MAP[6][6] = 1; 
	MAP[7][4] = 1; MAP[7][3] = 1; MAP[7][2] = 1;
	MAP[8][4] = 1; MAP[8][6] = 1;
	MAP[9][4] = 1; MAP[9][5] = 1; MAP[9][6] = 1; 
	MAP[11][0] = 1; MAP[11][1] = 1; MAP[11][2] = 1; MAP[11][3] = 1;
	MAP[12][7] = 1;
	MAP[13][7] = 1;
	MAP[14][7] = 1;
	MAP[12][0] = 1; MAP[12][1] = 1; MAP[12][2] = 1;

	for (int i = 0; i<path.size(); i++)
	{
		if (path[i].y == 0)
		path[i].y = 1.3;
		else if (path[i].y == 14)
		path[i].y = -1.3;
		else
		path[i].y = -path[i].y*0.2 + 1.4;
		if (path[i].x == 0)
		path[i].x = -0.8;
		else if (path[i].x == 9)
		path[i].x = 0.8;
		else
		path[i].x = path[i].x*0.2 - 0.9;
	}
	
	path.pop_back();
	if (NoPath!=0)
	{
		point t1;
		t1.x = xTar;
		t1.y = yTar;
		path.push_back(t1);
	}
	int n = path.size();
	
	point *p;
	p = new point[n];
	for (int i = 0; i < n; i++)
		p[i] = path[i];
	path.clear();
	pa->path = p;
	pa->length = n;
	return pa;
}


//void follow_path()
/*! \brief close the path-planning algorithm (memory released)
*
* \param[in,out] path path-planning main structure
*/
void free_path_planning(PathPlanning *path)
{
// ----- path-planning memory release start ----- //
	free(path);
// ----- path-planning memory release end ----- //

}
NAMESPACE_CLOSE();