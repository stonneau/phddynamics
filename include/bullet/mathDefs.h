/**
* \file mathDefs.h
* \brief math functions.
* \author Steve T.
* \version 0.1
* \date 11/05/2013
*
*/


#ifndef _MATH_DEFS
#define _MATH_DEFS

#include <math.h>
#include "btBulletDynamicsCommon.h"

const btScalar Pi = btScalar(acos(-1.0));

const btScalar RadiansToDegrees = btScalar(180.0/Pi);
const btScalar DegreesToRadians = btScalar(Pi/180);

#define RADIAN(X)	((X)*DegreesToRadians)

static btScalar GetSignedAngle(const btScalar& cosinus, const btScalar& sinus)
{	
	int sign = (sinus > 0) ? 1 : -1;
	return sign * acos(cosinus);
}

#endif //_MATH_DEFS