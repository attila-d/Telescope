#pragma once

#include "debug.h"

inline static double dRange(double x, double a, double b)
{
    double res = x;
    if (x >= a)
    {
        if (x > b)
        {
            double n = 1 + (x - b) / (b - a);
            int nn = n;
            res -= nn * (b - a);
        }
    }
    else
    {
        double n = 1 + (a - x) / (b - a);
        res += ((int)n) * (b - a);
    }
    return res;
}

inline static long lRange(long x, long a, long b)
{
    // DEBUG3("Range x=", x, DEC);
    // DEBUG3("[", a, DEC);
    // DEBUG3(",", b, DEC);
    // DEBUG("]");
    long res = x;
    if (x >= a)
    {
        if (x > b)
        {
            long n = 1 + (x - b) / (b - a);
            // DEBUG3("Range+ n=", n, DEC);
            res -= ((int)n) * (b - a);
        }
    }
    else
    {
        long n = 1 + (a - x) / (b - a);
        // DEBUG3("Range- n=", n, DEC);
        res += ((int)n) * (b - a);
    }
    // DEBUG3LN("Res=", res, DEC);
    return res;
}

inline double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
