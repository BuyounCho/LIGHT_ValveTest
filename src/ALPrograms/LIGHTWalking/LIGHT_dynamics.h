/*============================================================================
 *
 *  <light hopping dynamics>
 *
 * Input : Robot States (q,dq,ddq, Robot model... something like that)
 * Output : Object Matrix(A&b in 1/2|Ax-b|^2) or Constant Matrix (A&b in Ax=b)
 *          for dynamics level (containing force term)
 *
 *                -  Buyoun,Cho 2018.05.08
 *
=============================================================================*/

#ifndef LIGHT_HOPPING_DYNAMICS
#define LIGHT_HOPPING_DYNAMICS

#include "LIGHT_robotmodel.h"

extern LIGHTWholeBody LIGHT;


#endif // LIGHT_HOPPING_DYNAMICS

