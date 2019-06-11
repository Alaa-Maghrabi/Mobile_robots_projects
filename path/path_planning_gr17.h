 /*! 
 * \author Group 17
 * \file path_planning_gr17.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR17_H_
#define _PATH_PLANNING_GR17_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr17.h"
#include <vector>

using namespace std; 
NAMESPACE_INIT(ctrlGr17);

typedef struct {
	int index = 1;
	int xval;
	int yval;
	int parent_xval;
	int parent_yval;
	float hn;
	float gn;
	float fn;
} new_row;
typedef struct {
	float x;
	float y;
} point;


/// path-planning main structure 
struct PathPlanning
{
	point* path;
	int length;
} ;

PathPlanning* init_path_planning();
PathPlanning* path_planning(CtrlStruct *cvs,float xStart, float yStart, float xGoal, float yGoal, int Sta);
void free_path_planning(PathPlanning *path);
NAMESPACE_CLOSE();

#endif
