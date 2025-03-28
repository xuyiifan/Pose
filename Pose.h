#ifndef _POSE_H
#define _POSE_H

typedef float real;
typedef double lreal;

void Pose_Update(real (*q)[4], real (*q_noise)[3][3],
                 real (*w_bias)[3], real (*w_bias_noise)[3][3], const real (*w_bias_progress_noise)[3][3],
                 const real (*g)[3], const real (*g_noise)[3][3],
                 const real (*w)[3], const real (*w_noise)[3][3],
                 real directed_w, real directed_w_noise, const real (*direction)[3],
                 real dt);

#endif