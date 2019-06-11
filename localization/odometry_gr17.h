/*! 
 * \author Group 17
 * \file odometry_gr17.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR17_H_
#define _ODOMETRY_GR17_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr17.h"

NAMESPACE_INIT(ctrlGr17);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
