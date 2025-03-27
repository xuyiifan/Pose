#include "Quaternion.h"

#include <tgmath.h>

void Quaternion_ToRPY(const real q[4], real rpy[3])
{
    rpy[0] = atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    rpy[1] = asin(2 * (q[0] * q[2] - q[1] * q[3]));
    rpy[2] = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}