/*! 
 * \author Group 17
 * \file strategy_gr17.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR17_H_
#define _STRATEGY_GR17_H_

#include "CtrlStruct_gr17.h"
#include "path_planning_gr17.h"
#include "opp_pos_gr17.h"
NAMESPACE_INIT(ctrlGr17);

typedef struct Target
{
	point target;
	double cost;

} Target;
/// strategy main structure
typedef struct Strategy
{
	int main_state; ///< main state of the strategy
	//point *targets;
	Target *targets;
	int targets_size;
	double timestamp;
	int index;
	OpponentsPosition *prev;
	int Start_path;
	point prev_rob;
	double timestuck;
} Strategy;

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_A, GAME_STATE_B, GAME_STATE_C, GAME_STATE_D, GAME_STATE_E,GAME_STATE_F,GAME_STATE_G, Stuck_STATE
};

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif
