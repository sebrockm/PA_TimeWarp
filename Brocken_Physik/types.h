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
#define INFINITY CUDART_INF_F
#define EPSILON .000001f
#else
const f32 INFINITY = std::numeric_limits<f32>::infinity();
const f32 EPSILON = .000001f;
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

#define GENAU

template <class T>
CUDA_CALLABLE_MEMBER inline bool fEqual(T a, T b){
#ifdef GENAU
	return a == b;
#else
	return fabs(a - b) <= EPSILON;
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool fLess(T a, T b){
#ifdef GENAU
	return a < b;
#else
	return a - b <= -EPSILON;
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool fGreater(T a, T b){
#ifdef GENAU
	return a > b;
#else
	return a - b >= EPSILON;
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool fnEqual(T a, T b){
#ifdef GENAU
	return a != b;
#else
	return fLess(a, b) || fGreater(a, b);
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool fLessEq(T a, T b){
#ifdef GENAU
	return a <= b;
#else
	return a - b <= EPSILON;
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool fGreaterEq(T a, T b){
#ifdef GENAU
	return a >= b;
#else
	return a - b >= -EPSILON;
#endif
}

template <class T>
CUDA_CALLABLE_MEMBER inline bool equalSign(T f1, T f2){
	return f1 * f2 >= 0.;
}

template <class T>
inline T mod(T a, T b){
	 return a - b * std::floor(a/b);
}


const f32 PI = 3.14159265358979323846264338327950288419716939937510f;
const f32 SQRT2 = 1.4142135623730950488016887242097f;
const f32 SQRT3 = 1.7320508075688772935274463415059f;


#endif