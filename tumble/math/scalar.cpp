// sinf, cosf, atanf, atan2f and acosf from musl 1.2.6 (MIT, see MUSL_COPYRIGHT), from FreeBSD's msun:
//
// ====================================================
// Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
//
// Developed at SunPro, a Sun Microsystems, Inc. business.
// Permission to use, copy, modify, and distribute this
// software is freely granted, provided that this notice
// is preserved.
// ====================================================
//
// Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
// Debugged and optimized by Bruce D. Evans.
//
// ported to C++ as written, musl's names kept so each function reads against its source; the argument reduction is
// cut to the one input word and the precision that float arguments use.

#include "tumble/math/scalar.h"

TUMBLE_NAMESPACE_BEGIN

namespace scalar {

namespace {

uint32_t asuint(float f) { return __builtin_bit_cast(uint32_t, f); }
float asfloat(uint32_t i) { return __builtin_bit_cast(float, i); }
double asdouble(uint64_t i) { return __builtin_bit_cast(double, i); }

double scalbn(double x, int n) {
    double y = x;
    if (n > 1023) {
        y *= 0x1p1023;
        n -= 1023;
        if (n > 1023) {
            y *= 0x1p1023;
            n -= 1023;
            if (n > 1023) n = 1023;
        }
    } else if (n < -1022) {
        // make sure final n < -53 to avoid double rounding in the subnormal range
        y *= 0x1p-1022 * 0x1p53;
        n += 1022 - 53;
        if (n < -1022) {
            y *= 0x1p-1022 * 0x1p53;
            n += 1022 - 53;
            if (n < -1022) n = -1022;
        }
    }
    return y * asdouble((uint64_t)(0x3ff + n) << 52);
}

// |sin(x)/x - s(x)| < 2**-37.5 (~[-4.89e-12, 4.824e-12]).
constexpr double S1 = -0x15555554cbac77.0p-55;
constexpr double S2 = 0x111110896efbb2.0p-59;
constexpr double S3 = -0x1a00f9e2cae774.0p-65;
constexpr double S4 = 0x16cd878c3b46a7.0p-71;

float sindf(double x) {
    const double z = x * x;
    const double w = z * z;
    const double r = S3 + z * S4;
    const double s = z * x;
    return (float)((x + s * (S1 + z * S2)) + s * w * r);
}

// |cos(x) - c(x)| < 2**-34.1 (~[-5.37e-11, 5.295e-11]).
constexpr double C0 = -0x1ffffffd0c5e81.0p-54;
constexpr double C1 = 0x155553e1053a42.0p-57;
constexpr double C2 = -0x16c087e80f1e27.0p-62;
constexpr double C3 = 0x199342e0ee5069.0p-68;

float cosdf(double x) {
    const double z = x * x;
    const double w = z * z;
    const double r = C2 + z * C3;
    return (float)(((1.0 + z * C0) + w * C1) + (w * z) * r);
}

// 2/pi in 24-bit words; a float argument reaches no further than these
const int32_t ipio2[] = {
    0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62,
    0x95993C, 0x439041, 0xFE5163, 0xABDEBB, 0xC561B7, 0x246E3A,
    0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
    0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41,
    0x3991D6, 0x398353, 0x39F49C, 0x845F8B, 0xBDF928, 0x3B1FF8,
    0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
    0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5,
    0xF17B3D, 0x0739F7, 0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08,
    0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
    0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880,
    0x4D7327, 0x310606, 0x1556CA, 0x73A8C9, 0x60E27B, 0xC08C6B,
};

const double PIo2[] = {
    1.57079625129699707031e+00, 7.54978941586159635335e-08, 5.39030252995776476554e-15, 3.28200341580791294123e-22,
    1.27065575308067607349e-29, 1.22933308981111328932e-36, 2.73370053816464559624e-44, 2.16741683877804819444e-51,
};

/** __rem_pio2_large with one input word and precision 0: x is in [2^23, 2^24) scaled by 2^e0 */
int rem_pio2_large(double x, double *y, int e0) {
    int32_t jz, jv, jk, carry, n, iq[20], i, j, k, m, q0, ih;
    double z, fw, f[20], fq[20], q[20];

    jk = 3;
    const int32_t jp = jk;

    jv = (e0 - 3) / 24;
    if (jv < 0) jv = 0;
    q0 = e0 - 24 * (jv + 1);

    j = jv;
    m = jk;
    for (i = 0; i <= m; i++, j++) f[i] = j < 0 ? 0.0 : (double)ipio2[j];

    for (i = 0; i <= jk; i++) {
        fw = 0.0;
        fw += x * f[i];
        q[i] = fw;
    }

    jz = jk;
recompute:
    // distill q[] into iq[] reversingly
    for (i = 0, j = jz, z = q[jz]; j > 0; i++, j--) {
        fw = (double)(int32_t)(0x1p-24 * z);
        iq[i] = (int32_t)(z - 0x1p24 * fw);
        z = q[j - 1] + fw;
    }

    z = scalbn(z, q0);
    z -= 8.0 * __builtin_floor(z * 0.125);
    n = (int32_t)z;
    z -= (double)n;
    ih = 0;
    if (q0 > 0) {
        i = iq[jz - 1] >> (24 - q0);
        n += i;
        iq[jz - 1] -= i << (24 - q0);
        ih = iq[jz - 1] >> (23 - q0);
    } else if (q0 == 0) {
        ih = iq[jz - 1] >> 23;
    } else if (z >= 0.5) {
        ih = 2;
    }

    if (ih > 0) {
        n += 1;
        carry = 0;
        for (i = 0; i < jz; i++) {
            j = iq[i];
            if (carry == 0) {
                if (j != 0) {
                    carry = 1;
                    iq[i] = 0x1000000 - j;
                }
            } else {
                iq[i] = 0xffffff - j;
            }
        }
        if (q0 > 0) {
            switch (q0) {
                case 1: iq[jz - 1] &= 0x7fffff; break;
                case 2: iq[jz - 1] &= 0x3fffff; break;
            }
        }
        if (ih == 2) {
            z = 1.0 - z;
            if (carry != 0) z -= scalbn(1.0, q0);
        }
    }

    if (z == 0.0) {
        j = 0;
        for (i = jz - 1; i >= jk; i--) j |= iq[i];
        if (j == 0) {
            for (k = 1; iq[jk - k] == 0; k++) {}
            for (i = jz + 1; i <= jz + k; i++) {
                f[i] = (double)ipio2[jv + i];
                fw = 0.0;
                fw += x * f[i];
                q[i] = fw;
            }
            jz += k;
            goto recompute;
        }
    }

    // chop off zero terms
    if (z == 0.0) {
        jz -= 1;
        q0 -= 24;
        while (iq[jz] == 0) {
            jz--;
            q0 -= 24;
        }
    } else {
        z = scalbn(z, -q0);
        if (z >= 0x1p24) {
            fw = (double)(int32_t)(0x1p-24 * z);
            iq[jz] = (int32_t)(z - 0x1p24 * fw);
            jz += 1;
            q0 += 24;
            iq[jz] = (int32_t)fw;
        } else {
            iq[jz] = (int32_t)z;
        }
    }

    fw = scalbn(1.0, q0);
    for (i = jz; i >= 0; i--) {
        q[i] = fw * (double)iq[i];
        fw *= 0x1p-24;
    }

    for (i = jz; i >= 0; i--) {
        for (fw = 0.0, k = 0; k <= jp && k <= jz - i; k++) fw += PIo2[k] * q[i + k];
        fq[jz - i] = fw;
    }

    fw = 0.0;
    for (i = jz; i >= 0; i--) fw += fq[i];
    *y = ih == 0 ? fw : -fw;
    return n & 7;
}

// invpio2: 53 bits of 2/pi; pio2_1: first 25 bits of pi/2; pio2_1t: pi/2 - pio2_1
constexpr double toint = 1.5 / __DBL_EPSILON__;
constexpr double pio4 = 0x1.921fb6p-1;
constexpr double invpio2 = 6.36619772367581382433e-01;
constexpr double pio2_1 = 1.57079631090164184570e+00;
constexpr double pio2_1t = 1.58932547735281966916e-08;

int rem_pio2f(float x, double *y) {
    uint32_t bits = asuint(x);
    const uint32_t ix = bits & 0x7fffffff;
    // 25+53 bit pi is good enough for medium size
    if (ix < 0x4dc90fdb) {
        double fn = (double)x * invpio2 + toint - toint;
        int n = (int32_t)fn;
        *y = x - fn * pio2_1 - fn * pio2_1t;
        if (*y < -pio4) {
            n--;
            fn--;
            *y = x - fn * pio2_1 - fn * pio2_1t;
        } else if (*y > pio4) {
            n++;
            fn++;
            *y = x - fn * pio2_1 - fn * pio2_1t;
        }
        return n;
    }
    if (ix >= 0x7f800000) {
        *y = x - x;
        return 0;
    }
    // scale x into [2^23, 2^24-1]
    const int sign = bits >> 31;
    const int e0 = (int)(ix >> 23) - (0x7f + 23);
    bits = ix - ((uint32_t)e0 << 23);
    double ty;
    const int n = rem_pio2_large(asfloat(bits), &ty, e0);
    if (sign) {
        *y = -ty;
        return -n;
    }
    *y = ty;
    return n;
}

// small multiples of pi/2 rounded to double precision
constexpr double pio2_x1 = 1 * 1.57079632679489661923;
constexpr double pio2_x2 = 2 * 1.57079632679489661923;
constexpr double pio2_x3 = 3 * 1.57079632679489661923;
constexpr double pio2_x4 = 4 * 1.57079632679489661923;

const float atanhi[] = {
    4.6364760399e-01,
    7.8539812565e-01,
    9.8279368877e-01,
    1.5707962513e+00,
};

const float atanlo[] = {
    5.0121582440e-09,
    3.7748947079e-08,
    3.4473217170e-08,
    7.5497894159e-08,
};

const float aT[] = {
    3.3333328366e-01, -1.9999158382e-01, 1.4253635705e-01, -1.0648017377e-01, 6.1687607318e-02,
};

constexpr float pi_f = 3.1415927410e+00;
constexpr float pi_lo = -8.7422776573e-08;

constexpr float pio2_hi = 1.5707962513e+00;
constexpr float pio2_lo = 7.5497894159e-08;
constexpr float pS0 = 1.6666586697e-01;
constexpr float pS1 = -4.2743422091e-02;
constexpr float pS2 = -8.6563630030e-03;
constexpr float qS1 = -7.0662963390e-01;

float R(float z) {
    const float p = z * (pS0 + z * (pS1 + z * pS2));
    const float q = 1.0f + z * qS1;
    return p / q;
}

}  // namespace

float sin(float x) {
    double y;
    uint32_t ix = asuint(x);
    const int sign = ix >> 31;
    ix &= 0x7fffffff;

    if (ix <= 0x3f490fda) {
        if (ix < 0x39800000) return x;
        return sindf(x);
    }
    if (ix <= 0x407b53d1) {
        if (ix <= 0x4016cbe3) {
            if (sign) return -cosdf(x + pio2_x1);
            return cosdf(x - pio2_x1);
        }
        return sindf(sign ? -(x + pio2_x2) : -(x - pio2_x2));
    }
    if (ix <= 0x40e231d5) {
        if (ix <= 0x40afeddf) {
            if (sign) return cosdf(x + pio2_x3);
            return -cosdf(x - pio2_x3);
        }
        return sindf(sign ? x + pio2_x4 : x - pio2_x4);
    }

    // sin(Inf or NaN) is NaN
    if (ix >= 0x7f800000) return x - x;

    const int n = rem_pio2f(x, &y);
    switch (n & 3) {
        case 0: return sindf(y);
        case 1: return cosdf(y);
        case 2: return sindf(-y);
        default: return -cosdf(y);
    }
}

float cos(float x) {
    double y;
    uint32_t ix = asuint(x);
    const unsigned sign = ix >> 31;
    ix &= 0x7fffffff;

    if (ix <= 0x3f490fda) {
        if (ix < 0x39800000) return 1.0f;
        return cosdf(x);
    }
    if (ix <= 0x407b53d1) {
        if (ix > 0x4016cbe3) return -cosdf(sign ? x + pio2_x2 : x - pio2_x2);
        if (sign) return sindf(x + pio2_x1);
        return sindf(pio2_x1 - x);
    }
    if (ix <= 0x40e231d5) {
        if (ix > 0x40afeddf) return cosdf(sign ? x + pio2_x4 : x - pio2_x4);
        if (sign) return sindf(-x - pio2_x3);
        return sindf(x - pio2_x3);
    }

    // cos(Inf or NaN) is NaN
    if (ix >= 0x7f800000) return x - x;

    const unsigned n = (unsigned)rem_pio2f(x, &y);
    switch (n & 3) {
        case 0: return cosdf(y);
        case 1: return sindf(-y);
        case 2: return -cosdf(y);
        default: return sindf(y);
    }
}

float atan(float x) {
    float w, s1, s2, z;
    int id;

    uint32_t ix = asuint(x);
    const uint32_t sign = ix >> 31;
    ix &= 0x7fffffff;
    if (ix >= 0x4c800000) {
        if (x != x) return x;
        z = atanhi[3] + 0x1p-120f;
        return sign ? -z : z;
    }
    if (ix < 0x3ee00000) {
        if (ix < 0x39800000) return x;
        id = -1;
    } else {
        x = __builtin_fabsf(x);
        if (ix < 0x3f980000) {
            if (ix < 0x3f300000) {
                id = 0;
                x = (2.0f * x - 1.0f) / (2.0f + x);
            } else {
                id = 1;
                x = (x - 1.0f) / (x + 1.0f);
            }
        } else {
            if (ix < 0x401c0000) {
                id = 2;
                x = (x - 1.5f) / (1.0f + 1.5f * x);
            } else {
                id = 3;
                x = -1.0f / x;
            }
        }
    }
    z = x * x;
    w = z * z;
    // break sum from i=0 to 10 aT[i]z**(i+1) into odd and even poly
    s1 = z * (aT[0] + w * (aT[2] + w * aT[4]));
    s2 = w * (aT[1] + w * aT[3]);
    if (id < 0) return x - x * (s1 + s2);
    z = atanhi[id] - ((x * (s1 + s2) - atanlo[id]) - x);
    return sign ? -z : z;
}

float atan2(float y, float x) {
    float z;

    if (x != x || y != y) return x + y;
    uint32_t ix = asuint(x);
    uint32_t iy = asuint(y);
    if (ix == 0x3f800000) return atan(y);
    const uint32_t m = ((iy >> 31) & 1) | ((ix >> 30) & 2);  // 2*sign(x)+sign(y)
    ix &= 0x7fffffff;
    iy &= 0x7fffffff;

    if (iy == 0) {
        switch (m) {
            case 0:
            case 1: return y;
            case 2: return pi_f;
            case 3: return -pi_f;
        }
    }
    if (ix == 0) return m & 1 ? -pi_f / 2 : pi_f / 2;
    if (ix == 0x7f800000) {
        if (iy == 0x7f800000) {
            switch (m) {
                case 0: return pi_f / 4;
                case 1: return -pi_f / 4;
                case 2: return 3 * pi_f / 4;
                case 3: return -3 * pi_f / 4;
            }
        } else {
            switch (m) {
                case 0: return 0.0f;
                case 1: return -0.0f;
                case 2: return pi_f;
                case 3: return -pi_f;
            }
        }
    }
    // |y/x| > 0x1p26
    if (ix + (26 << 23) < iy || iy == 0x7f800000) return m & 1 ? -pi_f / 2 : pi_f / 2;

    // z = atan(|y/x|) with correct underflow
    if ((m & 2) && iy + (26 << 23) < ix) z = 0.0f;
    else z = atan(__builtin_fabsf(y / x));
    switch (m) {
        case 0: return z;
        case 1: return -z;
        case 2: return pi_f - (z - pi_lo);
        default: return (z - pi_lo) - pi_f;
    }
}

float acos(float x) {
    float z, w, s, c, df;

    uint32_t hx = asuint(x);
    const uint32_t ix = hx & 0x7fffffff;
    // |x| >= 1 or nan
    if (ix >= 0x3f800000) {
        if (ix == 0x3f800000) {
            if (hx >> 31) return 2 * pio2_hi + 0x1p-120f;
            return 0;
        }
        return 0 / (x - x);
    }
    // |x| < 0.5
    if (ix < 0x3f000000) {
        if (ix <= 0x32800000) return pio2_hi + 0x1p-120f;
        return pio2_hi - (x - (pio2_lo - x * R(x * x)));
    }
    // x < -0.5
    if (hx >> 31) {
        z = (1 + x) * 0.5f;
        s = __builtin_sqrtf(z);
        w = R(z) * s - pio2_lo;
        return 2 * (pio2_hi - (s + w));
    }
    // x > 0.5
    z = (1 - x) * 0.5f;
    s = __builtin_sqrtf(z);
    hx = asuint(s);
    df = asfloat(hx & 0xfffff000);
    c = (z - df * df) / (s + df);
    w = R(z) * s + c;
    return 2 * (df + w);
}

}  // namespace scalar

TUMBLE_NAMESPACE_END
