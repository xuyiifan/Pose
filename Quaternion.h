#ifndef _QUATERNION_H
#define _QUATERNION_H

typedef float real;
typedef double lreal;

void Quaternion_ToRPY(const real q[4], real rpy[3]);

#endif