/*! 
 * \author Group 17
 * \file controller_main_gr17.h
 * \brief main header of the controller
 */

#ifndef _CTRL_MAIN_GR17_H_
#define _CTRL_MAIN_GR17_H_

#include "CtrlStruct_gr17.h"
#include "namespace_ctrl.h"
#include <stdlib.h>

NAMESPACE_INIT(ctrlGr17);

void controller_init(CtrlStruct *cvs);
void controller_loop(CtrlStruct *cvs);
void controller_finish(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
