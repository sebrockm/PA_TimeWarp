#ifndef _TYPES_H_
#define _TYPES_H_

#include "cuda_macro.h"

#include <math_constants.h>
#include <numeric>
//#include <cmath>


typedef unsigned char	u8;
typedef unsigned short	u16;
typedef unsigned int	u32;
typedef unsigned long long	u64;

typedef signed char		s8;
typedef signed short	s16;
typedef signed int		s32;
typedef signed long long	s64;

typedef float	f32;
typedef double	f64;


#ifdef __CUDA_ARCH__
#define INFINITY CUDART_INF_F;
#define EPSILON .0000001f;
#else
const f32 INFINITY = std::numeric_limits<f32>::infinity();
const f32 EPSILON = .0000001f;
#endif

template <class T>
CUDA_CALLABLE_MEMBER inline T fabs(const T& a){
	return a < 0 ? -a : a;
}


template <class T>
CUDA_CALLABLE_MEMBER inline bool fNearlyEqual(const T& a, const T& b){
	return a == b;
}

template <>
CUDA_CALLABLE_MEMBER inline bool fNearlyEqual(const f32& a, const f32& b){
#ifdef __CUDACC__
	return a == b;
#else
	return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * std::numeric_limits<f32>::epsilon());
#endif
}

template <>
CUDA_CALLABLE_MEMBER inline bool fNearlyEqual(const f64& a, const f64& b){
#ifdef __CUDACC__
	return a == b;
#else
	return fabs(a - b) <= ( (fabs(a) > fabs(b) ? fabs(b) : fabs(a)) * std::numeric_limits<f64>::epsilon());
#endif
}

CUDA_CALLABLE_MEMBER inline bool fEqual(f64 a, f64 b){
	return fabs(a - b) <= EPSILON;
}

CUDA_CALLABLE_MEMBER inline bool fLess(f64 a, f64 b){
	return a - b <= -EPSILON;
}

CUDA_CALLABLE_MEMBER inline bool fGreater(f64 a, f64 b){
	return a - b >= EPSILON;
}

CUDA_CALLABLE_MEMBER inline bool fnEqual(f64 a, f64 b){
	return fLess(a, b) || fGreater(a, b);
}

CUDA_CALLABLE_MEMBER inline bool fLessEq(f64 a, f64 b){
	return a - b <= EPSILON;
}

CUDA_CALLABLE_MEMBER inline bool fGreaterEq(f64 a, f64 b){
	return a - b >= -EPSILON;
}

CUDA_CALLABLE_MEMBER inline bool equalSign(f64 f1, f64 f2){
	return f1 * f2 >= 0.;
}

inline f32 mod(f32 a, f32 b){
	 return a - b * std::floor(a/b);
}


const f32 PI = 3.14159265358979323846264338327950288419716939937510f;
const f32 SQRT2 = 1.4142135623730950488016887242097f;
const f32 SQRT3 = 1.7320508075688772935274463415059f;


#endif