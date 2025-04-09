#define NDEBUG
#include "MachanicalState.h"

#include "Matrix.h"

#define HPI 1.5707963f

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

void MachanicalState_Resolve(MachanicalState *mach,
                             const real (*g_)[3], const real (*g_noise)[3][3],
                             real (*g_true_)[3], real *w_mach,
                             real (*g_true_and_w_mach_noise)[4][4],
                             real (*w_mach_direction_)[3], real (*w_mach_direction_noise)[3][3],
                             real dt)
{
    const real (*offset)[3][1] = (typeof(offset))&mach->offset;
    const real (*v)[2][1] = (typeof(v))&mach->v;
    const real (*g)[3][1] = (typeof(g))g_;
    real (*g_true)[3][1] = (typeof(g_true))g_true_;
    real (*w_mach_direction)[3][1] = (typeof(w_mach_direction))w_mach_direction_;
    real d2t = dt * dt;
    
    real d_angle_ground = mach->w_ground * dt, d_angle_ground_noise = mach->w_ground_noise * d2t;
    real d_angle_ground_rotate[2][2], d_angle_ground_rotate_derivative[2][2];
    make_rotate_matrix(-d_angle_ground, &d_angle_ground_rotate);
    make_rotate_matrix(-d_angle_ground - HPI, &d_angle_ground_rotate_derivative);
    _Mvl9(_Mst(mach->last_v, = ,_Mgt(d_angle_ground_rotate $ mach->last_v)));
    real last_v_derivative[2][1];
    _Mvl3(_Mst(last_v_derivative, = ,d_angle_ground_rotate_derivative $ mach->last_v));
    _Mvl9(_Mst(mach->last_v_noise,
        = d_angle_ground_noise*,last_v_derivative $T last_v_derivative,
        + ,d_angle_ground_rotate $ _Mgt(mach->last_v_noise $T d_angle_ground_rotate)));
    
    real dv[2][1], dv_noise[2][2];
    _Mvl3(_Mst(dv, = ,*v, - ,mach->last_v));
    _Mvl3(_Mst(dv_noise, = ,mach->v_noise, + ,mach->last_v_noise));
    
    _Mvl3(_Mst(mach->last_v, = ,*v));
    _Mvl3(_Mst(mach->last_v_noise, = ,mach->v_noise));
    // dv without offset
    
    real yaw_rotate[2][2], yaw_rotate_derivative[2][2];
    make_rotate_matrix(-mach->yaw, &yaw_rotate);
    make_rotate_matrix(-mach->yaw - HPI, &yaw_rotate_derivative);
    real pitch_rotate[3][3], pitch_rotate_derivative[3][3];
    make_pitch_rotate_matrix(-mach->pitch, &pitch_rotate);
    make_pitch_rotate_matrix(-mach->pitch - HPI, &pitch_rotate_derivative);
    pitch_rotate_derivative[1][1] = 0;
    real yaw_pitch_rotate[3][2], yaw_pitch_rotate_derivative_to_yaw[3][2], yaw_pitch_rotate_derivative_to_pitch[3][2];
    _Mvl3(_Mst(yaw_pitch_rotate, = ,_Mpt(pitch_rotate, 0,0, 3,2) $ yaw_rotate));
    _Mvl3(_Mst(yaw_pitch_rotate_derivative_to_yaw, = ,_Mpt(pitch_rotate, 0,0, 3,2) $ yaw_rotate_derivative));
    _Mvl3(_Mst(yaw_pitch_rotate_derivative_to_pitch, = ,_Mpt(pitch_rotate_derivative, 0,0, 3,2) $ yaw_rotate));

    real offset_pitched[3][1], offset_pitched_derivative[3][1];
    _Mvl3(_Mst(offset_pitched, = ,pitch_rotate T$ *offset));
    _Mvl3(_Mst(offset_pitched_derivative, = ,pitch_rotate_derivative T$ *offset));
    real w_yaw_with_offset = mach->w_yaw + mach->w_ground, w_yaw_with_offset_noise = mach->w_yaw_noise + mach->w_ground_noise;
    real v_offset_pitched[3][1], v_offset_pitched_derivative_to_w_yaw[3][1],
         v_offset_pitched_derivative_to_w_pitch[3][1], v_offset_pitched_derivative_to_pitch[3][1];
    make_v_offset(w_yaw_with_offset, mach->w_pitch, offset_pitched, &v_offset_pitched);
    make_v_offset(1, 0, offset_pitched, &v_offset_pitched_derivative_to_w_yaw);
    make_v_offset(0, 1, offset_pitched, &v_offset_pitched_derivative_to_w_pitch);
    make_v_offset(w_yaw_with_offset, mach->w_pitch, offset_pitched_derivative, &v_offset_pitched_derivative_to_pitch);
    real v_offset_pitched_noise[3][3];
    _Mvl9(_Mst(v_offset_pitched_noise,
        = w_yaw_with_offset_noise*,v_offset_pitched_derivative_to_w_yaw $T v_offset_pitched_derivative_to_w_yaw,
        + mach->w_pitch_noise*,v_offset_pitched_derivative_to_w_pitch $T v_offset_pitched_derivative_to_w_pitch,
        + mach->pitch_noise*,v_offset_pitched_derivative_to_pitch $T v_offset_pitched_derivative_to_pitch));
    
    real d_yaw_with_offset = w_yaw_with_offset * dt, d_yaw_with_offset_noise = w_yaw_with_offset_noise * d2t;
    real d_yaw_with_offset_rotate2d[2][2], d_yaw_with_offset_rotate2d_derivative[2][2];
    make_rotate_matrix(-d_yaw_with_offset, &d_yaw_with_offset_rotate2d);
    make_rotate_matrix(-d_yaw_with_offset - HPI, &d_yaw_with_offset_rotate2d_derivative);
    _Mvl9(_Mst(_Mpt(mach->last_v_offset_pitched, 0,0, 2,1),
        = ,_Mgt(d_yaw_with_offset_rotate2d $ _Mpt(mach->last_v_offset_pitched, 0,0, 2,1))));
    real d_yaw_with_offset_rotate[3][3] = {}, d_yaw_with_offset_rotate_derivative[3][3] = {};
    _Mvl3(_Mst(_Mpt(d_yaw_with_offset_rotate, 0,0, 2,2), = ,d_yaw_with_offset_rotate2d));
    d_yaw_with_offset_rotate[2][2] = 1;
    _Mvl3(_Mst(_Mpt(d_yaw_with_offset_rotate_derivative, 0,0, 2,2), = ,d_yaw_with_offset_rotate2d_derivative));
    real last_v_offset_pitched_derivative[3][1];
    _Mvl3(_Mst(last_v_offset_pitched_derivative, = ,d_yaw_with_offset_rotate_derivative $ mach->last_v_offset_pitched));
    _Mvl9(_Mst(mach->last_v_offset_pitched_noise,
        = ,d_yaw_with_offset_rotate $ _Mgt(mach->last_v_offset_pitched_noise $T d_yaw_with_offset_rotate),
        + d_yaw_with_offset_noise*,last_v_offset_pitched_derivative $T last_v_offset_pitched_derivative));

    real dv_offset_pitched[3][1], dv_offset_pitched_noise[3][3];
    _Mvl3(_Mst(dv_offset_pitched, = ,v_offset_pitched, - ,mach->last_v_offset_pitched));
    _Mvl3(_Mst(dv_offset_pitched_noise, = ,mach->last_v_offset_pitched_noise, + ,v_offset_pitched_noise));

    _Mvl3(_Mst(mach->last_v_offset_pitched, = ,v_offset_pitched));
    _Mvl3(_Mst(mach->last_v_offset_pitched_noise, = ,v_offset_pitched_noise));
    // dv from offset
    
    real inv_dt = 1 / dt, inv2_dt = inv_dt * inv_dt;
    
    _Mvl9(_Mst(*g_true, = ,*g,
        - inv_dt*,yaw_pitch_rotate $ dv,
        - inv_dt*,pitch_rotate $ dv_offset_pitched));
    real dv_derivative_to_yaw[3][1], dv_derivative_to_pitch[3][1], dv_offset_pitched_derivative[3][1];
    _Mvl3(_Mst(dv_derivative_to_yaw, = ,yaw_pitch_rotate_derivative_to_yaw $ dv));
    _Mvl3(_Mst(dv_derivative_to_pitch, = ,yaw_pitch_rotate_derivative_to_pitch $ dv));
    _Mvl3(_Mst(dv_offset_pitched_derivative, = ,pitch_rotate_derivative $ dv_offset_pitched));
    real inv2_dt_w_yaw_noise = inv2_dt * w_yaw_with_offset_noise, inv2_dt_w_pitch_noise = inv2_dt * mach->w_pitch_noise;
    _Mvl9(_Mst(_Mpt(*g_true_and_w_mach_noise, 0,0, 3,3), = ,*g_noise,
        + inv2_dt*,yaw_pitch_rotate $ _Mgt(dv_noise $T yaw_pitch_rotate),
        + inv2_dt_w_yaw_noise*,dv_derivative_to_yaw $T dv_derivative_to_yaw,
        + inv2_dt_w_pitch_noise*,dv_derivative_to_pitch $T dv_derivative_to_pitch,
        + inv2_dt*,pitch_rotate $ _Mgt(dv_offset_pitched_noise $T pitch_rotate),
        + inv2_dt_w_pitch_noise*,dv_offset_pitched_derivative $T dv_offset_pitched_derivative));
    
    *w_mach = mach->w_yaw + mach->w_ground;
    (*g_true_and_w_mach_noise)[3][3] = mach->w_yaw_noise + mach->w_ground_noise;

    real g_true_and_w_mach_cov[3][1];
    real inv_dt_w_ground_noise = inv_dt * mach->w_ground_noise, inv_dt_w_yaw_with_offset_noise = inv_dt_w_ground_noise + inv_dt * mach->w_yaw_noise;
    _Mvl9(_Mst(g_true_and_w_mach_cov,
        = -inv_dt_w_ground_noise*,yaw_pitch_rotate $ last_v_derivative,
        + inv_dt_w_yaw_with_offset_noise*,pitch_rotate $ _Mgt(v_offset_pitched_derivative_to_w_yaw, - ,last_v_offset_pitched_derivative)));
    _Mvl3(_Mst(_Mpt(*g_true_and_w_mach_noise, 0,3, 3,1), = ,g_true_and_w_mach_cov));
    _Mvl3(_Mst(_Mpt(*g_true_and_w_mach_noise, 3,0, 1,3), = ,g_true_and_w_mach_cov $T$));

    _Mvl3(_Mst(*w_mach_direction, = ,_Mpt(pitch_rotate, 0,2, 3,1)));
    _Mvl3(_Mst(*w_mach_direction_noise, = mach->pitch_noise*,_Mpt(pitch_rotate_derivative, 0,2, 3,1) $T _Mpt(pitch_rotate_derivative, 0,2, 3,1)));
}