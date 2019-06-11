#include "speed_regulation_gr17.h"
#include "useful_gr17.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr17);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;
	double error_r, error_l;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //
	error_r= r_sp_ref - r_sp;
	error_l= l_sp_ref - l_sp;

	sp_reg->int_error_r = sp_reg->int_error_r+ limit_range(error_r,-0.2,0.2);
	sp_reg->int_error_l = sp_reg->int_error_l+limit_range(error_l, -0.2, 0.2);
	
	//Kp and Ki
	double kp = 20;
	double ki=0.9;
	// wheel commands
	outputs->wheel_commands[R_ID] =
		limit_range((100/50)*(kp*error_r + sp_reg->int_error_r * ki),-100,100);
	outputs->wheel_commands[L_ID] = 
		limit_range((100/50)*(kp*error_l + sp_reg->int_error_l * ki),-100,100);

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
