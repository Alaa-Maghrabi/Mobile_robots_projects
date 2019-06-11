/*! 
 * \author Group 17
 * \file path_regulation_gr17.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR17_H_
#define _PATH_REGULATION_GR17_H_

#include "CtrlStruct_gr17.h"
#include "path_planning_gr17.h"

NAMESPACE_INIT(ctrlGr17);

bool follow_path(CtrlStruct *cvs, PathPlanning *pa,int restart_path);

NAMESPACE_CLOSE();

#endif
