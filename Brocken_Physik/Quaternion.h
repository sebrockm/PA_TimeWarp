#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "Vector.h"
#include "Matrix.h"
#include "cuda_macro.h"
#include <cmath>

template <class T, u32 dim>
class Vector;

template <class T, u32 dim>
class Matrix;


template <class T>
class Quaternion {
private:
	Vector<T, 4> im;

public:
	CUDA_CALLABLE_MEMBER Quaternion():im(){}

	CUDA_CALLABLE_MEMBER Quaternion(const Quaternion& other):im(other.im){}

	CUDA_CALLABLE_MEMBER Quaternion(const T& re, const Vector<T, 3> im):im(re,im[0],im[1],im[2]){}

	CUDA_CALLABLE_MEMBER Quaternion(const T& re, const T& im1, const T& im2, const T& im3)
		:im(re,im1,im2,im3){}

	CUDA_CALLABLE_MEMBER Quaternion(const Vector<T, 4> v):im(v){}

	CUDA_CALLABLE_MEMBER Quaternion<T> operator * (const Quaternion<T>& other) const {
		return Quaternion(	im[0]*other.im[0] - im[1]*other.im[1] - im[2]*other.im[2] - im[3]*other.im[3], 
							im[0]*other.im[1] + im[1]*other.im[0] + im[2]*other.im[3] - im[3]*other.im[2],
							im[0]*other.im[2] + im[2]*other.im[0] + im[3]*other.im[1] - im[1]*other.im[3],
							im[0]*other.im[3] + im[3]*other.im[0] + im[1]*other.im[2] - im[2]*other.im[1]);
	}

	CUDA_CALLABLE_MEMBER Vector<T, 3> operator * (const Vector<T, 3> point) const {
		return (*this * Quaternion<T>(0, point) * getConjugate()).getImag();
	}

	CUDA_CALLABLE_MEMBER T getReal() const {
		return im[0];
	}

	CUDA_CALLABLE_MEMBER const Vector<T, 3> getImag() const {
		return Vector<T, 3>(im[1], im[2], im[3]);
	}

	CUDA_CALLABLE_MEMBER Quaternion<T> getConjugate() const {
		return Quaternion<T>(im[0], -im[1], -im[2], -im[3]);
	}

	CUDA_CALLABLE_MEMBER T lengthSqr() const {
		return im.lengthSqr();
	}

	CUDA_CALLABLE_MEMBER T length() const {
		return sqrt(lengthSqr());
	}

	CUDA_CALLABLE_MEMBER Matrix<T, 4> getMatrix4() const {
		T x2 = im[1] * im[1];
		T y2 = im[2] * im[2];
		T z2 = im[3] * im[3];
		T xy = im[1] * im[2];
		T xz = im[1] * im[3];
		T yz = im[2] * im[3];
		T wx = im[0] * im[1];
		T wy = im[0] * im[2];
		T wz = im[0] * im[3];
 
		return Matrix<T, 4>(1 - 2*(y2+z2),	2*(xy-wz),		2*(xz+wy),		0,
							2*(xy+wz),		1 - 2*(x2+z2),	2*(yz-wx),		0,
							2*(xz-wy),		2*(yz+wx),		1 - 2*(x2+y2),	0,
							0,				0,				0,				1);
	}

	CUDA_CALLABLE_MEMBER Matrix<T, 3> getMatrix3() const {
		T x2 = im[1] * im[1];
		T y2 = im[2] * im[2];
		T z2 = im[3] * im[3];
		T xy = im[1] * im[2];
		T xz = im[1] * im[3];
		T yz = im[2] * im[3];
		T wx = im[0] * im[1];
		T wy = im[0] * im[2];
		T wz = im[0] * im[3];
 
		return Matrix<T, 3>(1 - 2*(y2+z2),	2*(xy-wz),		2*(xz+wy),
							2*(xy+wz),		1 - 2*(x2+z2),	2*(yz-wx),
							2*(xz-wy),		2*(yz+wx),		1 - 2*(x2+y2) );
	}
};


typedef Quaternion<f32> Quaternionf;

template <class T>
CUDA_CALLABLE_MEMBER Quaternion<T> createRotationQuaternion(f32 phi, const Vector<T, 3>& axis){
	return Quaternion<T>(cos(phi/2), sin(phi/2)*axis);
}

#endif