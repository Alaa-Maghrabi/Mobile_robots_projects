/*! 
 * \author Group 17
 * \file calibration_gr17.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR17_H_
#define _CALIBRATION_GR17_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr17.h"

NAMESPACE_INIT(ctrlGr17);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
