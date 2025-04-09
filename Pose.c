#define NDEBUG
#include "Pose.h"

#include "Matrix.h"

static void make_omg(real coe, real w[3], real (*omg)[4][4])
{
    _Mvl3(_Mst(*omg, = coe*,((real[4][4]){
        {0, -w[0], -w[1], -w[2]},
        {w[0], 0, w[2], -w[1]},
        {w[1], -w[2], 0, w[0]},
        {w[2], w[1], -w[0], 0}
    })));
}

static void make_O(real coe, real q[4], real (*O)[3][3])
{
    _Mvl3(_Mst(*O, = coe*,((real[3][3]){
        {-q[1], -q[2], -q[3]},
        {q[0], -q[3], q[2]},
        {q[3], q[0], -q[1]}
    })));
}

static void make_predict_norm_g(real q[4], real (*predict_norm_g)[3][1])
{
    _Mvl3(_Mst(*predict_norm_g, = ,((real[3][1]){
        {2 * (q[1] * q[3] - q[0] * q[2])},
        {2 * (q[2] * q[3] + q[0] * q[1])},
        {1 - 2 * (q[1] * q[1] + q[2] * q[2])}
    })));
}

static void make_G(real coe, real q[4], real (*G)[3][3])
{
    _Mvl3(_Mst(*G, = coe*,((real[3][3]){
        {-q[2], q[3], -q[0]},
        {q[1], q[0], q[3]},
        {q[0], -q[1], -q[2]}
    })));
}

void Pose_Update(real (*q_)[4], real (*w_bias_)[3], const real (*w_bias_progress_noise)[3][3],
                 real (*q_and_w_bias_noise)[6][6],
                 const real (*w_)[3], const real (*w_noise)[3][3],
                 const real (*direction_)[3], const real (*direction_noise)[3][3],
                 const real (*g_)[3], real directed_w,
                 const real (*g_and_directed_w_noise)[4][4],
                 real dt)
{
    real (*q)[4][1] = (typeof(q))q_;
    real (*w_bias)[3][1] = (typeof(w_bias))w_bias_;
    const real (*g)[3][1] = (typeof(g))g_;
    const real (*w)[3][1] = (typeof(w))w_;
    const real (*direction)[3][1] = (typeof(direction))direction_;

    real unbiased_w[3][1];
    _Mvl3(_Mst(unbiased_w, = ,*w, - ,*w_bias));

    real omg[4][4], half_dt = dt / 2;
    make_omg(half_dt, (real*)unbiased_w, &omg);

    _Mvl9(_Mst(*q, += ,_Mgt(omg $ *q)));
    // x = f(x)

    real O[3][3];
    make_O(half_dt, *q_, &O);
    
    real F[12][12] = {};
    _Mvl3(_Mgr(F, +1));
    _Mvl3(_Mst(_Mpt(F, 0,0, 3,3), += ,_Mpt(omg, 0,0, 3,3)));
    _Mvl3(_Mst(_Mpt(F, 0,3, 3,3), -= ,O));
    _Mvl3(_Mst(_Mpt(F, 0,6, 3,3), += ,O));
    // F
    
    real P[12][12] = {};
    _Mvl3(_Mst(_Mpt(P, 0,0, 6,6), = ,*q_and_w_bias_noise));
    _Mvl3(_Mst(_Mpt(P, 6,6, 3,3), = ,*w_noise));
    _Mvl3(_Mst(_Mpt(P, 9,9, 3,3), = ,*direction_noise));
    // P

    _Mvl9(_Mst(P, = ,F $ _Mgt(P $T F)));
    _Mvl3(_Mst(_Mpt(P, 3,3, 3,3), += dt*,*w_bias_progress_noise));
    // P = FPF^T + noise;

    real predict_norm_g[3][1];
    make_predict_norm_g(*q_, &predict_norm_g);

    real predict_directed_w[1][1];
    _Mvl3(_Mst(predict_directed_w, = ,*direction T$ unbiased_w));
    // h(x)

    real g2[1][1];
    _Mvl3(_Mst(g2, = ,*g T$ *g));
    real inv_g2 = 1 / g2[0][0], inv_g = sqrt(inv_g2);
    
    real e[4][1];
    _Mvl3(_Mst(_Mpt(e, 0,0, 3,1), = inv_g*,*g, - ,predict_norm_g));
    e[3][0] = directed_w - predict_directed_w[0][0];
    // e = z - h(x)

    real G[3][3];
    make_G(2, *q_, &G);
    
    real H[4][12] = {};
    _Mvl3(_Mst(_Mpt(H, 0,0, 3,3), = ,G));
    _Mvl3(_Mst(_Mpt(H, 3,3, 1,3), = -,*direction $T$));
    _Mvl3(_Mst(_Mpt(H, 3,6, 1,3), = ,*direction $T$));
    _Mvl3(_Mst(_Mpt(H, 3,9, 1,3), = ,unbiased_w $T$));
    // H

    real HP[4][12];
    _Mvl3(_Mst(HP, = ,H $ P));
    
    real K[12][4];
    _Mvl9(_Mdv(_Mst(K, = ,HP $T$) $I _Mgt(HP $T H, + ,*g_and_directed_w_noise)));
    // K = PH^T / (HPH^T + R) = (HP)^T / (HPH^T + R)

    _Mvl3(_Mst(_Mpt(*q, 0,0, 3,1), += ,_Mpt(K, 0,0, 3,4) $ e));
    _Mvl3(_Mst(*w_bias, += ,_Mpt(K, 3,0, 3,4) $ e));
    // x += K(z - h(x))

    _Mvl9(_Mst(P, -= ,K $ HP));
    _Mvl9(_Mst(*q_and_w_bias_noise, = 0.5*,_Mpt(P, 0,0, 6,6), + 0.5*,_Mpt(P, 0,0, 6,6) $T$));
    // P -= KHP

    real q2[1][1];
    _Mvl3(_Mst(q2, = ,*q T$ *q));
    real inv_q = sqrt(1 / q2[0][0]);
    _Mvl3(_Mst(*q, = inv_q*,*q));
    #define mul(x) x *= inv_q
    _Mvl3(_Mcl(mul, _Mpt(*q_and_w_bias_noise, 0,0, 3,6)));
    _Mvl3(_Mcl(mul, _Mpt(*q_and_w_bias_noise, 0,0, 6,3)));
}