#include "opp_pos_gr17.h"
#include "init_pos_gr17.h"
#include "useful_gr17.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr17);

/*! \brief compute the opponents position using the tower
*
* \param[in,out] cvs controller main structure
*/
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;
	

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	rise_1 = limit_angle2(rise_1 + 2 * M_PI);
	fall_1 = limit_angle2(fall_1 + 2 * M_PI);

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_1 - 1;
		fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_1 - 1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
		rise_2= limit_angle2(rise_2 + 2 * M_PI);
		fall_2 = limit_angle2(fall_2 + 2 * M_PI);
	}

	// ----- opponents position computation start ----- //
	double new_x_opp, new_y_opp, new_x_opp1, new_y_opp1;
	single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &new_x_opp, &new_y_opp );
	opp_pos->x[0] = first_order_filter(new_x_opp, opp_pos->x[0],1,400);
	opp_pos->y[0] = first_order_filter(new_y_opp, opp_pos->y[0],1,500);
	if (nb_opp == 2)
	{
		single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &new_x_opp1, &new_y_opp1);
		opp_pos->x[1] = first_order_filter(new_x_opp1, opp_pos->x[1], delta_t, 300 * delta_t);
		opp_pos->y[1] = first_order_filter(new_y_opp1, opp_pos->y[1], delta_t, 500 * delta_t);
	}
	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
*
* \param[in] last_rise last rise relative angle [rad]
* \param[in] last_fall last fall relative angle [rad]
* \param[in] rob_x robot x position [m]
* \param[in] rob_y robot y position [m]
* \param[in] rob_theta robot orientation [rad]
* \param[out] new_x_opp new known x opponent position
* \param[out] new_y_opp new known y opponent position
* \return 1 if computation successful, 0 otherwise
*/
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{	
	double thetao = last_rise + last_fall; //theta opponent
	double thetar = last_fall - last_rise; 
	double xx, yy;
	xx = limit_range(0.040 *cos(thetao/2.0) / sin(thetar / 2.0),-2,2);
	yy = limit_range(0.040 *sin(thetao/2.0) / sin(thetar / 2.0),-2,2);

	*new_x_opp = rob_x + (xx+0.083)* cos(rob_theta*M_PI/180) - (yy) * sin((rob_theta*M_PI/180.0));
	*new_y_opp = rob_y + (xx+0.083)* sin(rob_theta*M_PI / 180) + (yy) * cos(rob_theta*M_PI / 180);
	
	
	return 1;
}

/*! \brief check if there is an opponent in front of the robot
*
* \param[in] cvs controller main structure
* \return 1 if opponent robot in front of the current robot
*/
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;

	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	double distance, angle;
	// no opponent
	if (!nb_opp)
	{
		return 0;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	for (i = 0; i<nb_opp; i++)
	{
		// ----- opponents check computation start ----- //

		distance = sqrt(opp_pos->x[i]-rob_pos->x)*((opp_pos->x[i] - rob_pos->x)) + ((opp_pos->y[i] - rob_pos->y))*((opp_pos->y[i] - rob_pos->y));
		angle = atan2((opp_pos->y[i] - rob_pos->y), (opp_pos->x[i] - rob_pos->x));
		angle = limit_angle2(angle + 2 * M_PI);
		if (distance < 0.5 && fabs(angle - rob_pos->theta*M_PI / 180) < M_PI_4)
		{
			return 1;
		}

		// ----- opponents check computation end ----- //
	}

	return 0;
}

NAMESPACE_CLOSE();
