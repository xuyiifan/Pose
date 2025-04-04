#ifndef _MACHANICAL_STATE_H
#define _MACHANICAL_STATE_H

typedef float real;
typedef double lreal;

typedef struct {
	const real offset[3];
	real last_v_offset_pitched[3][1];
	real yaw, pitch, w_yaw, w_pitch;
	real w_ground, w_noise;
	real last_v[2][1], last_v_noise[2][2];
	real v[2], v_noise[2][2];
} MachanicalState;

void MachanicalState_JustifyG(real dt, MachanicalState *mach,
                              const real (*g)[3], const real (*g_noise)[3][3],
                              real (*g_true)[3], real (*g_true_noise)[3][3], real (*w_direction_)[3]);

#endif