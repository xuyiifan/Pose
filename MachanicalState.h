#ifndef _MACHANICAL_STATE_H
#define _MACHANICAL_STATE_H

typedef float real;
typedef double lreal;

typedef struct {
	real pitch, yaw;
	real w, w_ground, w_noise;
	real v[2], v_noise[2][2];
} MachanicalState;

void MachanicalState_Apply(real dt, const real (*g)[3], const real (*g_noise)[3][3],
                           const MachanicalState *mach,
                           real (*last_mach_v)[2][1], real (*last_mach_v_noise)[2][2],
                           real (*g_true)[3], real (*g_true_noise)[3][3], real (*mach_w_direction)[3]);

#endif