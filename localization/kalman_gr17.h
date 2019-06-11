/*! 
 * \author Group 17
 * \file kalman_gr17.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR17_H_
#define _KALMAN_GR17_H_

#include "CtrlStruct_gr17.h"
#include "init_pos_gr17.h"

NAMESPACE_INIT(ctrlGr17);

/// Kalman main structure
struct KalmanStruc
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
