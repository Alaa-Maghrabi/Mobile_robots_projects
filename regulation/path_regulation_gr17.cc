
#include "path_regulation_gr17.h"
#include "useful_gr17.h"
#include "init_pos_gr17.h"
#include "speed_regulation_gr17.h"
#include "math.h"
#include <vector>
#include "path_planning_gr17.h"
#include "opp_pos_gr17.h"

using namespace std;

NAMESPACE_INIT(ctrlGr17);

#define	EPS_DISTANCE		0.04
#define	EPS_DISTANCE_R		0.07
#define EPS_DISTANCE_D		0.005
#define	EPS_DEG				1
#define RAD_DEG				180/M_PI
#define WHEEL_RADIUS		0.03
#define R_ROBOT				0.1125


short int step_path = 0, iter;

double prev_dist = 0, Ie_dist = 0, prev_dtheta = 0, Ie_dtheta = 0;


void move(CtrlStruct *cvs, double w, double v)
{
	double speed_r, speed_l, speed_rw, speed_lw, k, k_th;

	speed_r = v - w*R_ROBOT; 
	speed_l = v + w*R_ROBOT; 
	speed_lw = speed_l / WHEEL_RADIUS / 2.5;
	speed_rw = speed_r / WHEEL_RADIUS / 2.5;
	speed_regulation(cvs, speed_rw, speed_lw);

}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
/*! \brief follow a given path
*
* \param[in,out] cvs controller main structur
controller main structure
*/

bool  follow_path(CtrlStruct *cvs, PathPlanning* pa, int restart_path)
{
	double dtheta, w, v, lf, dist, k, k_dth, k_b, dt;
	double de_dist, de_dtheta;
	point *inter = pa->path;
	int size = pa->length;
	RobotPosition *rob_pos;
	rob_pos = cvs->rob_pos;

	lf = distance(rob_pos->x, rob_pos->y, inter[size - 1].x, inter[size - 1].y);

	if (restart_path == 1)
	{
		step_path = 0;
	}

	if (step_path == size)
	{
		v = 0;
		w = 0;
		step_path = 0;
		speed_regulation(cvs, 0, 0);
		return true;
	}


	if (rob_pos->x == inter[step_path].x) {
		if (inter[step_path].y > rob_pos->y) dtheta = 90 - rob_pos->theta;
		if (inter[step_path].y < rob_pos->y) dtheta = -90 - rob_pos->theta;
	}
	else dtheta = RAD_DEG*atanl((inter[step_path].y - rob_pos->y) / (inter[step_path].x - rob_pos->x)) - rob_pos->theta;//Find difference between the theta and the robot and the theta between the two positions 


	if ((atan((inter[step_path].y - rob_pos->y) / (inter[step_path].x - rob_pos->x)) > 0) && (((inter[step_path].y - rob_pos->y) < 0) || ((inter[step_path].x - rob_pos->x) < 0)))
		dtheta = dtheta - 180;
	if ((atan((inter[step_path].y - rob_pos->y) / (inter[step_path].x - rob_pos->x)) < 0) && (((inter[step_path].y - rob_pos->y) > 0) || ((inter[step_path].x - rob_pos->x) < 0)))
		dtheta = dtheta + 180;
	if (dtheta > 180) dtheta = dtheta - 2 * 180;
	if (dtheta < -180) dtheta = dtheta + 2 * 180;//dtheta has to stay between -180 and 180

	dist = distance(rob_pos->x, rob_pos->y, inter[step_path].x, inter[step_path].y);


	float KPv = 5; //PID proportional gain constant 
	float KDv = 20;//PID derivative gain constant

	if (dist > 0.11) KPv = 3;

	de_dist = dist - prev_dist;
	prev_dist = dist;

	double KPw = -0.05; //PID proportional gain constant 
	double KDw = -0.30; //PID derivative gain constant

	if (abs(dtheta) > 45) KPw = KPw * 20;
	else if (abs(dtheta) > 5) KPw = KPw * 10;
	de_dtheta = dtheta - prev_dtheta;
	prev_dtheta = dtheta;
	Ie_dtheta = Ie_dtheta + dtheta;

	if (lf < EPS_DISTANCE_D)
	{
		v = 0;
		w = 0;
		de_dist = 0;
		de_dtheta = 0;
		speed_regulation(cvs, 0, 0);
		if (step_path == size - 1) step_path++;
		return true;
	}
	else
	{
		if (step_path < size - 1)
		{
			if ((abs(dtheta) > 30) || ((step_path == 0) && (abs(dtheta) > 3))) v = 0;
			else v = (KPv*dist + KDv*de_dist) * 1 + 1;
			w = KPw*dtheta + KDw*de_dtheta;

			if (dist < EPS_DISTANCE)
			{
				step_path++;
				de_dist = 0;
				de_dtheta = 0;
			}
			move(cvs, w, v);
		}
		else
		{
			if (dist < EPS_DISTANCE_D)
			{
				v = 0;
				w = 0;
				de_dist = 0;
				de_dtheta = 0;
				step_path++;
				return true;
			}
			else if (dist < EPS_DISTANCE_R)
			{
				if (abs(dtheta)<EPS_DEG)	speed_regulation(cvs, 4, 4);
				else if (dtheta>5) speed_regulation(cvs, 5, -5);
				else if (dtheta>0) speed_regulation(cvs, 1, -1);
				else if (dtheta<-5) speed_regulation(cvs, -5, 5);
				else speed_regulation(cvs, -1, 1);
			}
			else
			{
				v = (KPv*dist + KDv*de_dist);
				w = KPw*dtheta + KDw*de_dtheta;
				move(cvs, w, v);
			}
		}
	}

	return false;

}
NAMESPACE_CLOSE();


/*! \brief follow a given path
*
* \param[in,out] cvs controller main structure
*/