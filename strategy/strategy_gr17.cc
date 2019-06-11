#include "strategy_gr17.h"
#include "path_planning_gr17.h"
#include "speed_regulation_gr17.h"
#include "path_regulation_gr17.h"
#include "init_pos_gr17.h"
#include "opp_pos_gr17.h"
#include "odometry_gr17.h"
#include <math.h>
NAMESPACE_INIT(ctrlGr17);
using namespace std;
/*! \brief intitialize the strategy structure
*
* \return strategy structure initialized
*/
int sta = 1;
int reference = 0;
int restart_path = 0;
bool  fol = false;
double BASE_A[4]{ -1,-0.9,-0.5,-1.5 };
double BASE_B[4]{ -1,0.9,-0.5,1.5 };

Strategy* init_strategy()
{
	Strategy *strat;
	strat = (Strategy*)malloc(sizeof(Strategy));
	strat->prev = (OpponentsPosition*)malloc(sizeof(OpponentsPosition));
	for (int i = 0; i<2; i++)
	{
		strat->prev->x[i] = 0.0;
		strat->prev->y[i] = 0.0;
	}
	return strat;
}
int ref = 0;
int changed(int nbtargets)
{
	if (nbtargets == 1 && ref == 0)
	{
		ref = 1;
		return 1;
	}
	else if (nbtargets == 0 && ref == 1)
		ref = 0;
	return 0;
}

int recog_base(CtrlStruct *cvs)
{
	int inbase = 0;
	if (cvs->team_id == TEAM_A)
		inbase = (cvs->rob_pos->x > BASE_A[0] && cvs->rob_pos->y < BASE_A[1] && cvs->rob_pos->x<BASE_A[2] && cvs->rob_pos->y>BASE_A[3]);
	else
		inbase = (cvs->rob_pos->x > BASE_B[0] && cvs->rob_pos->y > BASE_B[1] && cvs->rob_pos->x<BASE_B[2] && cvs->rob_pos->y<BASE_B[3]);

	return inbase;
}

int targettaken(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;
	strat = cvs->strat;
	inputs = cvs->inputs;
	double *xopp_prev = strat->prev->x;
	double *yopp_prev = strat->prev->y;
	double *xopp = cvs->opp_pos->x;
	double *yopp = cvs->opp_pos->y;
	double t_flag;
	int detected = -1;
	if (inputs->t - strat->timestamp >= 1.25)
	{
		for (int i = 0; i < cvs->opp_pos->nb_opp; i++)
			if (abs(xopp[i] - xopp_prev[i]) < 0.2 && abs(yopp[i] - yopp_prev[i]) < 0.2)
			{
				for (int j = 0; j < cvs->strat->targets_size; j++)
				{
					if (abs(xopp[i] - cvs->strat->targets[j].target.x) < 0.2 && abs(yopp[i] - cvs->strat->targets[j].target.y) < 0.2)
					{
						detected = j;
						break;
					}
				}
			}
		for (int i = 0; i < cvs->opp_pos->nb_opp; i++)
		{
			xopp_prev[i] = xopp[i];
			yopp_prev[i] = yopp[i];
		}
		strat->timestamp = inputs->t;
	}
	return detected;
}

/*! \brief release the strategy structure (memory released)
*
* \param[out] strat strategy structure to release
*/
void free_strategy(Strategy *strat)
{
	free(strat);
}
/*! \brief startegy during the game
*
* \param[in,out] cvs controller main structure
*/
float distancee(float x1, float x2, float y1, float y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

double cost_fn(CtrlStruct *cvs, int i)
{
	double cost = 0;
	cost = 1 / (cvs->strat->targets[i].cost / 3 + 1 / distancee(cvs->rob_pos->x, cvs->strat->targets[i].target.x, cvs->rob_pos->y, cvs->strat->targets[i].target.y));
	return cost;
}

int min(Target* temp_array, int n)
{
	int i;
	Target min = temp_array[0];
	int min_ind = 0;
	for (i = 1; i < n; i++)
	{
		if (temp_array[i].cost < min.cost)
		{
			min = temp_array[i];
			min_ind = i;
		}
	}
	return min_ind;
}

void remove_target(CtrlStruct* cvs, int index)
{
	Target temp;
	temp = cvs->strat->targets[cvs->strat->targets_size - 1];
	cvs->strat->targets[cvs->strat->targets_size - 1] = cvs->strat->targets[index];
	cvs->strat->targets[index] = temp;
	cvs->strat->targets_size--;
}

void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;
	bool arrived;
	bool flag;//indicates if we paln the new path 
			  // variables initialization
	strat = cvs->strat;
	inputs = cvs->inputs;
	double *xopp_prev = strat->prev->x;
	double *yopp_prev = strat->prev->y;
	double *xopp = cvs->opp_pos->x;
	double *yopp = cvs->opp_pos->y;
	double t_flag;
	int detected;
	int Start_path = strat->Start_path;
	//Target temp;
	float cost;
	
	//int strat->index = strat->strat->index;
	int index_taken = targettaken(cvs);
	if (index_taken != -1)
	{
		remove_target(cvs, index_taken);
	}

	switch (strat->main_state)
	{
	case GAME_STATE_A:

		if (check_opp_front(cvs))
		{
			speed_regulation(cvs, 0, 0);
			strat->main_state = GAME_STATE_G;
			strat->timestuck = inputs->t;
			break;
		}

		else if (strat->targets_size == 0)
		{
			strat->main_state = GAME_STATE_D; break;
	    }

		else if ((strat->targets[strat->targets_size].target.x == cvs->path->path[cvs->path->length - 1].x &&
			strat->targets[strat->targets_size].target.y == cvs->path->path[cvs->path->length - 1].y))
		{
			strat->main_state = GAME_STATE_B;
			break;
			
		}

		else if (cvs->inputs->target_detected && fol && !recog_base(cvs))
		{
			strat->main_state = GAME_STATE_C;
			break;
		}

		else 
		{
			fol = follow_path(cvs, cvs->path, restart_path);
			restart_path = 0;
		}
		
		if (cvs->inputs->target_detected == 0 && fol && strat->targets_size!=0)
		{
			for (int i = 0; i < strat->targets_size; i++)
				if (strat->targets[i].target.x == cvs->path->path[cvs->path->length - 1].x &&
					strat->targets[i].target.y == cvs->path->path[cvs->path->length - 1].y)
				{
					remove_target(cvs, i);
					strat->main_state = GAME_STATE_B;
					break;
				}
		}

		break;

	case GAME_STATE_B:
		if (cvs->strat->targets_size == 0)
		{
			strat->main_state = GAME_STATE_D;
			break;
		}

		speed_regulation(cvs, 0.0, 0.0);
		cvs->outputs->flag_release = 0;
		for (int i = 0; i<cvs->strat->targets_size; i++)
			cvs->strat->targets[i].cost = cost_fn(cvs, i); // might have to loop over
		strat->index = min(cvs->strat->targets, cvs->strat->targets_size);
		cvs->path = path_planning(cvs, cvs->rob_pos->x, cvs->rob_pos->y, cvs->strat->targets[strat->index].target.x, cvs->strat->targets[strat->index].target.y, sta);
		restart_path = 1;
		strat->main_state = GAME_STATE_A;
		break;

	case GAME_STATE_C:
		speed_regulation(cvs, 0.0, 0.0); // wait to get target
										 
		if (changed(cvs->inputs->nb_targets) && cvs->strat->targets_size>1) // beware when there is only one available target 
		{	// nb_targets would stay at 1 and we would delete targets in vain
			remove_target(cvs, strat->index);
			cvs->strat->main_state = GAME_STATE_B;
		}
		else if (changed(cvs->inputs->nb_targets) && cvs->strat->targets_size == 1)
		{

			remove_target(cvs, strat->index);
			cvs->strat->main_state = GAME_STATE_D;
		}
		else if (cvs->inputs->nb_targets == 2)
		{
			remove_target(cvs, strat->index);
			strat->main_state = GAME_STATE_D;
					}
		break;

	case GAME_STATE_D:
		if (cvs->team_id == TEAM_A)
			cvs->path = path_planning(cvs, cvs->rob_pos->x, cvs->rob_pos->y, -0.75, -1.2, sta);
		else cvs->path = path_planning(cvs, cvs->rob_pos->x, cvs->rob_pos->y, -0.75, 1.2, sta);
		restart_path = 1;
		if (cvs->path->length != 0)
			strat->main_state = GAME_STATE_E;
		else
		{
			strat->timestuck = inputs->t;
			strat->main_state = GAME_STATE_G;
		}
		sta = 1;
		//strat->main_state = GAME_STATE_E;
		break;

	case GAME_STATE_E: // game finishe
			if (check_opp_front(cvs))
		{
			speed_regulation(cvs, 0, 0);
			sta = 0;
			strat->timestuck= inputs->t;
			strat->main_state = GAME_STATE_G;
			break;
		}	
		follow_path(cvs, cvs->path, restart_path);
		restart_path = 0;

		if ((cvs->team_id == TEAM_A) && (cvs->rob_pos->x + 0.7 < 0.01 && cvs->rob_pos->y + 1.2 < 0.01) || (cvs->team_id == TEAM_B) && (cvs->rob_pos->x + 0.7 < 0.01 && cvs->rob_pos->y - 1.2 < 0.01))
		{
			cvs->outputs->flag_release = 1;
			if (cvs->strat->targets_size == 0)
				strat->main_state = GAME_STATE_F;
			else
				strat->main_state = GAME_STATE_B;
		}
		break;

	case GAME_STATE_F:
		speed_regulation(cvs, 0, 0);
		return;
		break;

	case GAME_STATE_G:
		if (inputs->t - strat->timestuck <= 1)
		{
			speed_regulation(cvs, -10, 10);
		}

		else
		{
			if (cvs->inputs->nb_targets == 2)
			{
				speed_regulation(cvs, 0, 0);
				strat->main_state = GAME_STATE_D;
				sta = 0;
			}
			else
			{
				speed_regulation(cvs, 0.0, 0.0);
				for (int i = 0; i < strat->targets_size; i++)
					if (strat->targets[i].target.x == cvs->path->path[cvs->path->length - 1].x &&
						strat->targets[i].target.y == cvs->path->path[cvs->path->length - 1].y)
					{
						strat->targets[i].cost = 10;
						break;
					}
					else
						cvs->strat->targets[i].cost = cost_fn(cvs, i);
				;
				strat->index = min(cvs->strat->targets, cvs->strat->targets_size);
				cvs->path = path_planning(cvs, cvs->rob_pos->x, cvs->rob_pos->y, cvs->strat->targets[strat->index].target.x, cvs->strat->targets[strat->index].target.y, 0);
				restart_path = 1;
				
				if (cvs->path->length != 0)
					strat->main_state = GAME_STATE_A;
				else
				{
					strat->timestuck = inputs->t;
				}
			}
		}
		break;
	default:
		exit(EXIT_FAILURE);
	}
}
NAMESPACE_CLOSE();

