#define NDEBUG
#include "MachanicalState.h"

#include "Matrix.h"

static void make_rotate_matrix(real angle, real (*matrix)[2][2])
{
    real sin_angle = sin(angle), cos_angle = cos(angle);
    
    _Mvl3(_Mst(*matrix, = ,((real[2][2]){
        {cos_angle, -sin_angle},
        {sin_angle,  cos_angle}
    })));
}

static void make_pitch_rotate_matrix(real angle, real (*matrix)[3][3])
{
    real sin_angle = sin(angle), cos_angle = cos(angle);
    
    _Mvl3(_Mst(*matrix, = ,((real[3][3]){
        { cos_angle, 0, sin_angle},
        {         0, 1,         0},
        {-sin_angle, 0, cos_angle}
    })));
}

static void make_v_offset(real w_yaw, real w_pitch, const real offset[3][1], real (*v_offset)[3][1])
{
    _Mvl3(_Mst(*v_offset, = ,((real[3][1]){
        {-offset[1][0] * w_yaw + offset[2][0] * w_pitch},
        { offset[0][0] * w_yaw                         },
        {                       -offset[0][0] * w_pitch}
    })));
}

void MachanicalState_JustifyG(real dt, MachanicalState *mach,
                              const real (*g_)[3], const real (*g_noise)[3][3],
                              real (*g_true_)[3], real (*g_true_noise)[3][3], real (*w_direction_)[3])
{
    const real (*offset)[3][1] = (typeof(offset))&mach->offset;
    const real (*v)[2][1] = (typeof(v))&mach->v;
    const real (*g)[3][1] = (typeof(g))g_;
    real (*g_true)[3][1] = (typeof(g_true))g_true_;
    real (*w_direction)[3][1] = (typeof(w_direction))w_direction_;
    
    real d_ground_angle = mach->w_ground * dt, d_ground_angle_rotate[2][2];
    make_rotate_matrix(-d_ground_angle, &d_ground_angle_rotate);
    _Mvl9(_Mst(mach->last_v, = ,_Mgt(d_ground_angle_rotate $ mach->last_v)));

    real d_ground_angle_rotate_derivative[2][2], last_v_derivative[2][1], d_ground_angle_noise = mach->w_noise * dt * dt;
    make_rotate_matrix(-d_ground_angle - 1.5707963f, &d_ground_angle_rotate_derivative);
    _Mvl3(_Mst(last_v_derivative, = ,d_ground_angle_rotate_derivative $ mach->last_v));
    _Mvl9(_Mst(mach->last_v_noise,
        = d_ground_angle_noise*,last_v_derivative $T last_v_derivative,
        + ,d_ground_angle_rotate $ _Mgt(mach->last_v_noise $T d_ground_angle_rotate)));
    
    real dv[2][1], dv_noise[2][2];
    _Mvl3(_Mst(dv, = ,*v, - ,mach->last_v));
    _Mvl3(_Mst(dv_noise, = ,mach->v_noise, + ,mach->last_v_noise));
    
    _Mvl3(_Mst(mach->last_v, = ,*v));
    _Mvl3(_Mst(mach->last_v_noise, = ,mach->v_noise));
    // dv without offset
    
    real yaw_rotate[2][2], pitch_rotate[3][3];
    make_rotate_matrix(-mach->yaw, &yaw_rotate);
    make_pitch_rotate_matrix(-mach->pitch, &pitch_rotate);
    real yaw_pitch_rotate[3][2];
    _Mvl3(_Mst(yaw_pitch_rotate, = ,_Mpt(pitch_rotate, 0,0, 3,2) $ yaw_rotate));

    real offset_pitched[3][1], v_offset_pitched[3][1], w_yaw_with_offset = mach->w_yaw + mach->w_ground;
    _Mvl3(_Mst(offset_pitched, = ,pitch_rotate T$ *offset));
    make_v_offset(w_yaw_with_offset, mach->w_pitch, offset_pitched, &v_offset_pitched);
    
    real d_yaw_with_offset = w_yaw_with_offset * dt, d_yaw_with_offset_rotate[2][2];
    make_rotate_matrix(-d_yaw_with_offset, &d_yaw_with_offset_rotate);
    _Mvl9(_Mst(_Mpt(mach->last_v_offset_pitched, 0,0, 2,1),
        = ,_Mgt(d_yaw_with_offset_rotate $ _Mpt(mach->last_v_offset_pitched, 0,0, 2,1))));
    
    real d_yaw_with_offset_rotate_derivative[2][2], last_v_offset_pitched_derivative[2][1], d_yaw_with_offset_noise = d_ground_angle_noise;
    make_rotate_matrix(-d_yaw_with_offset - 1.5707963f, &d_yaw_with_offset_rotate_derivative);
    _Mvl3(_Mst(last_v_offset_pitched_derivative, = ,d_yaw_with_offset_rotate_derivative $ _Mpt(mach->last_v_offset_pitched, 0,0, 2,1)));
    real dv_offset_pitched_noise[3][3] = {};
    _Mvl3(_Mst(_Mpt(dv_offset_pitched_noise, 0,0, 2,2),
        = d_yaw_with_offset_noise*,last_v_offset_pitched_derivative $T last_v_offset_pitched_derivative));
    
    real dv_offset_pitched[3][1];
    _Mvl3(_Mst(dv_offset_pitched, = ,v_offset_pitched, - ,mach->last_v_offset_pitched));

    _Mvl3(_Mst(mach->last_v_offset_pitched, = ,v_offset_pitched));
    // dv with offset
    
    real inv_dt = 1 / dt, inv2_dt = inv_dt * inv_dt;
    
    _Mvl9(_Mst(*g_true, = ,*g,
        - inv_dt*,yaw_pitch_rotate $ dv,
        - inv_dt*,pitch_rotate $ dv_offset_pitched));
    _Mvl9(_Mst(*g_true_noise, = ,*g_noise,
        + inv2_dt*,yaw_pitch_rotate $ _Mgt(dv_noise $T yaw_pitch_rotate),
        + inv2_dt*,pitch_rotate $ _Mgt(dv_offset_pitched_noise $T pitch_rotate)));
    _Mvl3(_Mst(*w_direction, = ,_Mpt(pitch_rotate, 0,2, 3,1)));
}