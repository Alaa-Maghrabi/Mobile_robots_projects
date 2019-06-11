#include "kalman_gr17.h"
#include "odometry_gr17.h"
#include "triangulation_gr17.h"
#include "useful_gr17.h"

NAMESPACE_INIT(ctrlGr17);
#define Radius (0.030)
#define b (0.225)
#define DEG_TO_RAD (M_PI/180.0)
#define RAD_TO_DEG (180.0/M_PI)
#define TIME_STEP 0.001

/*! \brief follow a given path
*
* \param[in,out] cvs controller main structure
*/

// multiply two matrices m1 and m2 and putting the result in m3
// row and col are the values of the dimensions of matrices being multiplied
void mul(double m1[3][3], double m2[3][3], double m3[3][3], double row, double col)
{
	int i, j, k;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			for (k = 0; k < row; k++)
			{
				m3[i][j] = m3[i][j] + (m1[i][k] * m2[k][j]);
			}
		}
	}
}

// Invert a matrix m1 and putting it in minv
void inver(double m1[3][3], double minv[3][3])
{

	// computes the inverse of a matrix m
	double det;
	det = m1[0][0] * (m1[1][1] * m1[2][2] - m1[2][1] * m1[1][2]) -
		m1[0][1] * (m1[1][0] * m1[2][2] - m1[1][2] * m1[2][0]) +
		m1[0][2] * (m1[1][0] * m1[2][1] - m1[1][1] * m1[2][0]);

	double invdet;
	invdet = 1 / det;

	//struct matrix minv; // inverse of matrix m

	minv[0][0] = (m1[1][1] * m1[2][2] - m1[2][1] * m1[1][2]) * invdet;
	minv[0][1] = (m1[0][2] * m1[2][1] - m1[0][1] * m1[2][2]) * invdet;
	minv[0][2] = (m1[0][1] * m1[1][2] - m1[0][2] * m1[1][1]) * invdet;
	minv[1][0] = (m1[1][2] * m1[2][0] - m1[1][0] * m1[2][2]) * invdet;
	minv[1][1] = (m1[0][0] * m1[2][2] - m1[0][2] * m1[2][0]) * invdet;
	minv[1][2] = (m1[1][0] * m1[0][2] - m1[0][0] * m1[1][2]) * invdet;
	minv[2][0] = (m1[1][0] * m1[2][1] - m1[2][0] * m1[1][1]) * invdet;
	minv[2][1] = (m1[2][0] * m1[0][1] - m1[0][0] * m1[2][1]) * invdet;
	minv[2][2] = (m1[0][0] * m1[1][1] - m1[1][0] * m1[0][1]) * invdet;


}

// Transpose a matrix m1 and putting the result in m2
void trans(double m1[3][3], double m2[3][3])
{
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j <= 3; j++) {
			m2[i][j] = m1[j][i];
		}
	}
}

// add two matrices m1 and m2, putting the sum in m3
void add_ma(double m1[3][3], double m2[3][3], double m3[3][3])
{
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			m3[i][j] = m1[i][j] + m2[i][j];
		}
	}
}

// multiply a matrix m1 by a scalar and putting the result in m2
void mul_ct(double m1[3][3], double m2[3][3], double a)
{
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			m2[i][j] = m1[i][j] * a;
		}
	}
}

void kalman(CtrlStruct *cvs)
{
	// variable declaration
	//double meas_x, meas_y, meas_theta;
	double dS, dtheta, dx, dy, dt, dSr, dSl;
	double r_sp, l_sp;
	//double k_r = 0.0015, k_l = 0.0001, k_d = 0.0000;
	double k_r = 5, k_l = 2, k_d = 0.01;
	double err_x, err_y, err_theta;
	double J_delta[3][3];
	double J_p[3][3] = { { 0,0,0 },{ 0, 0, 0 },{ 0, 0, 0 } }; // R prediction error
	static double final_eps[3][3] = { { 1,0,0 },{ 0, 1, 0 },{ 0, 0, 1 } };
	double pos_fin[3][3];

	RobotPosition *rob_pos;
	RobotPosition *triang_pos;
	KalmanStruc kal_pos;
	CtrlIn *inputs;

	// variables initialization
	rob_pos = cvs->rob_pos;
	triang_pos = cvs->triang_pos;
	inputs = cvs->inputs;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed


	double err[3][3] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };    //error difference triang and odometry
	err[0][0] = -rob_pos->x + triang_pos->x;
	err[1][0] = -rob_pos->y + triang_pos->y;
	err[2][0] = -rob_pos->theta*M_PI / 180 + triang_pos->theta;

	// time
	dt = TIME_STEP;

	dSr = dt*Radius*(r_sp);
	dSl = dt*Radius*(l_sp);
	dS = dt*Radius*((r_sp)+(l_sp)) / 2.0;
	dtheta = dt*Radius*((r_sp)-(l_sp)) / b;

	double pos_ma[3][3] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };    //matrix position odometry
	pos_ma[0][0] = rob_pos->x;
	pos_ma[1][0] = rob_pos->y;
	pos_ma[2][0] = rob_pos->theta;

	double covar[3][3] = { { k_r*fabs(dSr),0,0 },{ 0,k_l*fabs(dSl),0 },{ 0,0,0 } }; //R prediction error
	double covar1[3][3] = { { k_d,0,0 },{ 0,k_d,0 },{ 0,0,k_d } }; //Q prediction error

	J_delta[0][0] = 0.5*(cos(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0)) - 0.5*dS / b*sin(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);
	J_delta[0][1] = 0.5*(cos(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0)) + 0.5*dS / b*sin(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);
	J_delta[1][0] = 0.5*(sin(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0)) + 0.5*dS / b*cos(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);
	J_delta[1][1] = 0.5*(sin(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0)) - 0.5*dS / b*cos(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);
	J_delta[2][0] = 0;
	J_delta[2][1] = 0;

	J_p[0][2] = -dS*sin(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);
	J_p[1][2] = dS*cos(rob_pos->theta*DEG_TO_RAD + dtheta / 2.0);

	double eps_delta[3][3];
	double J_delta_t[3][3];
	//Jdelta*epsilonedelta*Jdelta_transpose
	mul(J_delta, covar, eps_delta, 3, 2);
	trans(J_delta, J_delta_t);
	mul(eps_delta, J_delta_t, eps_delta, 3, 3);

	double eps_f[3][3];
	double J_p_t[3][3];
	double inter1[3][3];
	//J_p*epsilone*J_p_transpose
	mul(J_p, final_eps, inter1, 3, 3);
	trans(J_p, J_p_t);
	mul(inter1, J_p_t, eps_f, 3, 3);

	// add these last two to get the final Sigma'
	add_ma(eps_f, eps_delta, final_eps);

	double final_eps_inv[3][3];
	double K[3][3];
	double inter2[3][3];
	// Calculate kalman gain
	add_ma(final_eps, covar1, inter2);
	inver(inter2, final_eps_inv);
	mul(inter2, final_eps, K, 3, 3);

	double final_est[3][3];
	double inter[3][3];
	double new_eps[3][3];
	//calculate the final estimation
	mul(K, err, inter, 3, 1);
	add_ma(pos_ma, inter, pos_fin);

	//calculate the final epsilone
	mul(K, final_eps, inter, 3, 3);
	mul_ct(inter, inter, -1);
	add_ma(final_eps, inter, new_eps);


	mul_ct(new_eps, final_eps, 1);

	triang_pos->x = pos_fin[0][0];
	triang_pos->y = pos_fin[1][0];
	triang_pos->theta = pos_fin[2][0];
	
}

NAMESPACE_CLOSE();

