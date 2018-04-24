#pragma once

#define PIf       3.14159265358f
#define HALF_PI_F 1.57079632679f
#define IPIf      0.31830988618f //inverse of Pi
#define I180      0.00555555555f
#define PI180f    0.01745329251f
#define d180PIf   57.2957795131f

#ifndef SQUARE
  #define SQUARE(x) ((x)*(x))
#endif

//These were defined in a lib file?
#ifndef MIN
  #define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
  #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef ABS
  #define ABS(x) ((x) > 0 ? (x) : -(x))
#endif

#ifndef CONSTRAIN
  #define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

#ifndef CHANGERANGE
  #define CHANGERANGE(oldValue,oldMax,oldMin,newMax,newMin) ((((oldValue - oldMin) * (newMax - newMin)) / (oldMax - oldMin)) + newMin)
#endif

#ifndef DegreesToRadians
#define DegreesToRadians(degrees)(degrees * PI180f)
#endif
#ifndef RadiansToDegrees
#define RadiansToDegrees(radians)(radians * d180PIf)
#endif