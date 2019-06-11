/*! 
 * \author Group 17
 * \file mcl_gr17.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR17_H_
#define _MCL_GR17_H_

#include "CtrlStruct_gr17.h"
#include "init_pos_gr17.h"

NAMESPACE_INIT(ctrlGr17);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
