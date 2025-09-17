#ifndef __MATH_UTIL_H__
#define __MATH_UTIL_H__

//#define USE_HWACC_BAM_MATH			// by default it's disabled

#ifdef CONFIG_CURIE
#ifdef USE_HWACC_BAM_MATH
#include "bamath.h"
#define cos(x) cos_bam(x)
#define sin(x) sin_bam(x)
#define sqrt(x) sqrt_bam(x)
#endif
#endif

#endif
