#include "Noise.h"

#include "Matrix.h"

void Noise_MeasureStart(size_t n, lreal (*noise)[n][n], lreal (*progress_noise)[n][n])
{
    #define set0(x) x = 0
    _Mvl3(_Mcl(set0, *noise));
    _Mvl3(_Mcl(set0, *progress_noise));
}

void Noise_MeasureUpdate(size_t i, size_t n, const lreal (*sample_)[n],
                         lreal prev_sample[2][n][1], lreal (*noise)[n][n], lreal (*progress_noise)[n][n])
{
    const lreal (*sample)[n][1] = (typeof(sample))sample_;

    if (i >= 2)
    {
        lreal diff[n][1];
        _Mvl9(_Mst(diff, = ,*sample, - ,prev_sample[i - 1 & 1]));
        _Mvl3(_Mst(*noise, += ,diff $T diff));
        _Mvl9(_Mst(diff, = ,*sample, - ,prev_sample[i - 2 & 1]));
        _Mvl3(_Mst(*progress_noise, += ,diff $T diff));
    }

    _Mvl3(_Mst(prev_sample[i & 1], = ,*sample));
}

void Noise_MeasureFinish(size_t m, size_t n, lreal (*noise)[n][n], lreal (*progress_noise)[n][n], lreal dt)
{
    lreal inv = 1.0 / (m - 2), ninv = inv / 2, pinv = inv / dt;

    #define lmbd(a, b) \
        b -= a, \
        a -= b, \
        a *= ninv, \
        b *= pinv
    
    _Mvl3(_Mcl(lmbd, *noise, *progress_noise));
}