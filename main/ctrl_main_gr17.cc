/*! 
 * \author Group 17
 * \file controller_main_gr17.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr17.h"
#include "namespace_ctrl.h"
#include "init_pos_gr17.h"
#include "odometry_gr17.h"
#include "opp_pos_gr17.h"
#include "speed_regulation_gr17.h"
#include "calibration_gr17.h"
#include "triangulation_gr17.h"
#include "strategy_gr17.h"
#include "path_planning_gr17.h"
#include "path_regulation_gr17.h"
#include "kalman_gr17.h"
#include "set_output.h"

NAMESPACE_INIT(ctrlGr17);

/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
 //targets
Target targets[8] = {
	{ { -0.8,0 }, 3 },
	{ {0.7, 0.6},1 },
	{ {0.7,-0.6},1},
	{ {-0.4,-0.6},1 },
	{ {-0.4,0.6},1 },
	{ {0.25,-1.25 }, 2 },
	{ {0.25,1.25 }, 2 },
	{ {0.1,0}, 2 }
};

//int sta=1;
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
	cvs->robot_id = inputs->robot_id;

	// robot team
	switch (inputs->robot_id)
	{
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_R: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;
		case ROBOT_W: cvs->team_id = TEAM_B; break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", inputs->robot_id);
			exit(EXIT_FAILURE);
	}

	// number of opponents
	cvs->nb_opp = inputs->nb_opponents;
	// robot initial position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;

	cvs->strat->targets = targets;

	cvs->strat->Start_path = 1;

}

/*! \brief controller loop (called every time-step)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	//int restart_path=0;
	int staaa = 1;
	// variables declaration
	double t;
	CtrlIn *inputs;
	CtrlOut *outputs;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	// time
	t = inputs->t;

	// update the robot odometry
	update_odometry(cvs);
	// tower control
	outputs->tower_command = 50.0;
	// triangulation
	triangulation(cvs);

	opponents_tower(cvs);

	int restart_path = 1;
	 
	switch (cvs->main_state)
	{
		// calibration
		case CALIB_STATE:
			calibration(cvs);
			if (t>-5)
			{
				cvs->main_state = WAIT_INIT_STATE;
				cvs->path = path_planning(cvs, cvs->rob_pos->x, cvs->rob_pos->y,cvs->strat->targets[0].target.x, cvs->strat->targets[0].target.y,staaa);
			}
			break;

		// wait before match beginning
		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0, 0);
			if (t > 0.0)
			{
				cvs->strat->timestamp = 0;
				cvs->strat->timestuck = 0;
				cvs->main_state = RUN_STATE;
				cvs->strat->main_state = GAME_STATE_A;
				cvs->strat->index = 0;
				cvs->strat->targets_size = 8;
			}
			break;

		// during game
		case RUN_STATE:
			
			main_strategy(cvs);
			
			if (t > 89.0) // 1 second safety
			{
				cvs->main_state = STOP_END_STATE;
			}
			break;

		// stop at the end of the game
		case STOP_END_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			outputs->flag_release = 1;
			break;

		case NB_MAIN_STATES:
			exit(EXIT_FAILURE);
			break;
	
		default:
			exit(EXIT_FAILURE);
	}
}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

NAMESPACE_CLOSE();
