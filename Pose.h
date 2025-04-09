#ifndef _POSE_H
#define _POSE_H

typedef float real;
typedef double lreal;

void Pose_Update(real (*q)[4], real (*w_bias)[3], const real (*w_bias_progress_noise)[3][3],
                 real (*q_and_w_bias_noise)[6][6],
                 const real (*w)[3], const real (*w_noise)[3][3],
                 const real (*direction)[3], const real (*direction_noise)[3][3],
                 const real (*g)[3], real directed_w,
                 const real (*g_and_directed_w_noise)[4][4],
                 real dt);

#endif