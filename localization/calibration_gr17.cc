#include "calibration_gr17.h"
#include "speed_regulation_gr17.h"
#include "odometry_gr17.h"
#include "useful_gr17.h"
#include "init_pos_gr17.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr17);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians
// calibration states
enum { CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_STATE_D, CALIB_STATE_E, CALIB_STATE_F, CALIB_FINISH, CALIB_GOINIT, CALIB_STOP };

/*! \brief calibration of the robot to calibrate its position
*
* \param[in,out] cvs controller main structure
*
* This FSM can be adapted, depending on the map and on the robots initial position.
*/
int i = 1;
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	int team_id;
	double t;
	double time;
	//int i=1;
	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib = cvs->calib;

	rob_pos = cvs->rob_pos;

	t = inputs->t;
	team_id = cvs->team_id;

	// finite state machine (FSM)
	switch (calib->flag)
	{
	case CALIB_START: // start calibration
		speed_regulation(cvs, 0.0, 0.0);

		calib->flag = CALIB_STATE_A; // directly go to state 1
		calib->t_flag = t;

		break;

	case CALIB_STATE_A: // state 1: go back until you hit the wall 
		speed_regulation(cvs, -10.0, -10.0);
		// go to state B after it hits the wall 
		if (inputs->u_switch[0] == 1 || inputs->u_switch[1] == 1)
		{
			calib->flag = CALIB_STATE_B;
			calib->t_flag = t;
		}
		break;

	case CALIB_STATE_B: // state B
		speed_regulation(cvs, -7.0, -7.0); //go back 

		if (inputs->u_switch[0] == 1)
		{
			speed_regulation(cvs, 0.0, 0.0);
			calib->flag = CALIB_STATE_C; //right switch on 
			calib->t_flag = t;
		}
		else if (inputs->u_switch[1] == 1) //left switch is on 
		{
			calib->t_flag = t; speed_regulation(cvs, 0.0, 0.0); calib->flag = CALIB_STATE_D;
		}
		break;

	case CALIB_STATE_C: // state C
		speed_regulation(cvs, 0.0, -7.0);


		if (inputs->u_switch[1] == 1) {
			speed_regulation(cvs, -2.0, -2.0);
			if (t - calib->t_flag > 0.25)
			{
				calib->flag = CALIB_FINISH;
				calib->t_flag = t;
			}
		}
		break;
	case CALIB_STATE_D: // state D
		speed_regulation(cvs, -7.0, 0.0);
		if (inputs->u_switch[0] == 1) {
			speed_regulation(cvs, -2.0, -2.0);
			if (t - calib->t_flag>0.25)
			{
				calib->flag = CALIB_FINISH;
				calib->t_flag = t;
			}
		}
		break;
	case CALIB_STATE_E: // state E
		i = 0;
		speed_regulation(cvs, 10, 10.0);//go forward 
		if (t - calib->t_flag > 0.75)
		{
			speed_regulation(cvs, 0.0, 0.0);
			calib->flag = CALIB_STATE_F;
		}
		break;
	case CALIB_STATE_F:
		speed_regulation(cvs, 5, -5.0);//turn
		if (t - calib->t_flag>2)
		{
			speed_regulation(cvs, 0.0, 0.0);
			calib->flag = CALIB_STATE_A;
			calib->t_flag = t;
		}
		break;
	case CALIB_FINISH: // wait before the match is starting
		speed_regulation(cvs, 0.0, 0.0);
		if (i == 0)
		{
			calib->flag = CALIB_GOINIT;
			if (cvs->team_id == TEAM_A)
				rob_pos->x = (562.0 + 60.0 - 62.0) / 1000.0;
			else rob_pos->x = 1 - 0.06;

		}
		if (i == 1 && t - calib->t_flag>0.25)
		{
			calib->flag = CALIB_STATE_E;
			calib->t_flag = t;

			if (cvs->team_id == TEAM_A)
			{
				rob_pos->theta = -90;
				rob_pos->y = (1562.0 - 60.0 - 62.0) / 1000.0;
			}
			else
			{
				rob_pos->y = (-1562.0 + 60.0 + 62.0) / 1000.0;
				rob_pos->theta = 90;
			}

		}

		break;
	case CALIB_GOINIT:
		if (t - calib->t_flag>1)
		{
			speed_regulation(cvs, -5.0, 5.0);
			if (t - calib->t_flag > 2.23)
			{
				calib->flag = CALIB_STOP;
				calib->t_flag = t;
			}
		}
		else
			speed_regulation(cvs, 5.0, 5.0);

		break;

	case CALIB_STOP:
		speed_regulation(cvs, 0.0, 0.0);
		break;
	default:
		printf("Error: unknown state : %d !\n", calib->flag);
		exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
