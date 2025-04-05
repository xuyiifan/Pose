#ifndef _MACHANICAL_STATE_H
#define _MACHANICAL_STATE_H

typedef float real;
typedef double lreal;

typedef struct {
	const real offset[3];
	real last_v_offset_pitched[3][1];
	real last_v_offset_pitched_noise[3][3];
	real yaw;
	const real yaw_noise;
	real pitch;
	const real pitch_noise;
	real w_yaw;
	const real w_yaw_noise;
	real w_pitch;
	const real w_pitch_noise;
	real w_ground;
	real w_ground_noise;
	real last_v[2][1];
	real last_v_noise[2][2];
	real v[2];
	real v_noise[2][2];
} MachanicalState;

void MachanicalState_JustifyG(real dt, MachanicalState *mach,
                              const real (*g)[3], const real (*g_noise)[3][3],
                              real (*g_true)[3], real (*g_true_noise)[3][3], real (*w_direction)[3]);

#endif