/*
 * math_utils.h
 * Common math helpers.
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <math.h>

namespace math_utils
{
    template <typename T>
    inline T clamp (T value, T min_value, T max_value)
    {
        if (min_value > value)
        {
            return min_value;
        }
        if (max_value < value)
        {
            return max_value;
        }
        return value;
    }
}

#endif /* MATH_UTILS_H */
