#include "odometry_gr17.h"
#include "useful_gr17.h"
#include "init_pos_gr17.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr17);
#define Radius (0.030)
#define b (0.225)
#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;
	double dS, dtheta, dx, dy;
	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

	// ----- odometry computation start ----- //
	dS = dt*Radius*((r_sp) + (l_sp))/ 2.0;
	dtheta= dt*Radius*((r_sp)-(l_sp)) / b;
	dx = dS*cos(rob_pos->theta*DEG_TO_RAD + dtheta/2.0);
	dy = dS*sin(rob_pos->theta*DEG_TO_RAD + dtheta/2.0);
	rob_pos->x = rob_pos->x + dx;
	rob_pos->y = rob_pos->y + dy;
	rob_pos->theta = limit_angle(rob_pos->theta*DEG_TO_RAD + dtheta)*RAD_TO_DEG;

	// ----- odometry computation end ----- //

	// last update time
	rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
