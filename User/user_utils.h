#ifndef __USER_UTILS_H__
#define __USER_UTILS_H__

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(val, low, high)  ( MAX( (low), MIN( (val), (high) ) ) )

#endif  // __USER_UTILS_H__
