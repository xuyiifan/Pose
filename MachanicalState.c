#include "MachanicalState.h"

#include "Matrix.h"

void MachanicalState_Apply(real dt, const real (*g_)[3], const real (*g_noise)[3][3],
                           const MachanicalState *mach,
                           real (*last_mach_v)[2][1], real (*last_mach_v_noise)[2][2],
                           real (*g_true_)[3], real (*g_true_noise)[3][3], real (*mach_w_direction_)[3])
{
    static const real rotate_jacobian[2][2] = {
        {0, -1},
        {1,  0}};
    
    const real (*mach_v)[2][1] = (typeof(mach_v))&mach->v;
    const real (*g)[3][1] = (typeof(g))g_;
    real (*g_true)[3][1] = (typeof(g_true))g_true_;
    real (*mach_w_direction)[3][1] = (typeof(mach_w_direction))mach_w_direction_;
    
    real d_ground_angle = mach->w_ground * dt;
    real d_ground_angle_noise = mach->w_noise * dt * dt;
    real sin_d_ground_angle = sin(d_ground_angle), cos_d_ground_angle = cos(d_ground_angle);
    
    float d_ground_angle_rotate[2][2] = {
        {cos_d_ground_angle, sin_d_ground_angle},
        {-sin_d_ground_angle,  cos_d_ground_angle}};
    _Mvl9(_Mst(*last_mach_v, = ,d_ground_angle_rotate $ *last_mach_v));
    float last_mach_v_derivative[2][1];
    _Mvl3(_Mst(last_mach_v_derivative, = ,rotate_jacobian $ *last_mach_v));
    _Mvl9(_Mst(*last_mach_v_noise,
        = d_ground_angle_noise*,rotate_jacobian $ _Mgt(*last_mach_v_noise $T rotate_jacobian),
        + d_ground_angle_noise*,last_mach_v_derivative $T last_mach_v_derivative,
        + ,d_ground_angle_rotate $ _Mgt(*last_mach_v_noise $T d_ground_angle_rotate)));
    
    real mach_dv[2][1], mach_dv_noise[2][2];
    _Mvl3(_Mst(mach_dv, = ,*mach_v, - ,*last_mach_v));
    _Mvl3(_Mst(mach_dv_noise, = ,mach->v_noise, + ,*last_mach_v_noise));
    
    _Mvl3(_Mst(*last_mach_v, = ,*mach_v));
    _Mvl3(_Mst(*last_mach_v_noise, = ,mach->v_noise));
    
    real inv_dt = 1 / dt, inv2_dt = inv_dt * inv_dt, sin_yaw = sin(mach->yaw), cos_yaw = cos(mach->yaw);
    
    real yaw_rotate[2][2] = {
        { cos_yaw, sin_yaw},
        {-sin_yaw, cos_yaw}};
    
    real sin_pitch = sin(mach->pitch), cos_pitch = cos(mach->pitch);
    
    real pitch_rotate[3][3] = {
        { cos_pitch, 0, -sin_pitch},
        {         0, 1,          0},
        { sin_pitch, 0,  cos_pitch}};
    
    real yaw_pitch_rotate[3][2];
    _Mvl3(_Mst(yaw_pitch_rotate, = ,_Mpt(pitch_rotate, 0,0, 3,2) $ yaw_rotate));
    
    _Mvl3(_Mst(*g_true, = ,*g, - inv_dt*,yaw_pitch_rotate $ mach_dv));
    _Mvl9(_Mst(*g_true_noise, = ,*g_noise, + inv2_dt*,yaw_pitch_rotate $ _Mgt(mach_dv_noise $T yaw_pitch_rotate)));
    _Mvl3(_Mst(*mach_w_direction, = ,_Mpt(pitch_rotate, 0,2, 3,1)));
}