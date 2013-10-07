#ifndef _VECTOR_H_
#define _VECTOR_H_

#include "types.h"
#include "Array.h"
#include "Quaternion.h"
#include "cuda_macro.h"
#include <cuda_runtime.h>

template <class T>
class Quaternion;


template <class T, u32 dim>
class Vector{
public:
	class DimError{};

private:
	Array<T, dim> x;

public:
	//CUDA_CALLABLE_MEMBER Vector():x(){}

	CUDA_CALLABLE_MEMBER ~Vector(){};

	CUDA_CALLABLE_MEMBER Vector(const T& val = T()){
		for(int i=0;i<dim;i++)
			x[i] = val;
	}

	CUDA_CALLABLE_MEMBER Vector(const T& x, const T& y){
#ifndef __CUDACC__
		if(dim != 2) throw DimError();
#endif		
		this->x[0] = x;
		this->x[1] = y;
	}

	CUDA_CALLABLE_MEMBER Vector(const T& x, const T& y, const T& z){
#ifndef __CUDACC__
		if(dim != 3) throw DimError();
#endif
		this->x[0] = x;
		this->x[1] = y;
		this->x[2] = z;
	}

	CUDA_CALLABLE_MEMBER Vector(const T& x, const T& y, const T& z, const T& w){
#ifndef __CUDACC__
		if(dim != 4) throw DimError();
#endif
		this->x[0] = x;
		this->x[1] = y;
		this->x[2] = z;
		this->x[3] = w;
	}

	CUDA_CALLABLE_MEMBER Vector(const T* arr):x(arr){}

	CUDA_CALLABLE_MEMBER Vector(const Array<T, dim>& arr):x(arr){}

	CUDA_CALLABLE_MEMBER Vector(const Vector<T, dim>& other):x(other.x){}

	CUDA_CALLABLE_MEMBER Vector(const Vector<T, dim-1>& other, const T& s){
		for(u32 i = 0; i < dim-1; i++)
			x[i] = other[i];
		x[dim-1] = s;
	}

	CUDA_CALLABLE_MEMBER operator Vector<T, dim-1> () const {
		Vector<T, dim-1> erg;
		for(u32 i = 0; i < dim-1; i++)
			erg[i] = (*this)[i];

		return erg;
	}

	CUDA_CALLABLE_MEMBER T& operator [] (int i){
		return x[i];
	}

	CUDA_CALLABLE_MEMBER const T& operator [] (int i) const {
		return x[i];
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator = (const Vector<T, dim>& other){
		if(this != &other) 
			x = other.x;
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator += (const Vector<T, dim>& other){
		for(int i=0;i<dim;i++)
			x[i] += other.x[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator -= (const Vector<T, dim>& other){
		for(int i=0;i<dim;i++)
			x[i] -= other.x[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator *= (const T& scalar){
		for(int i=0;i<dim;i++)
			x[i] *= scalar;
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator /= (const T& scalar){
		for(int i=0;i<dim;i++)
			x[i] /= scalar;
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim>& operator %= (const T& scalar){
		for(int i=0;i<dim;i++)
			x[i] = mod(x[i], scalar);
		return *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator + (const Vector<T, dim>& rhs) const {
		Vector<T, dim> res(*this);
		return res += rhs;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator - (const Vector<T, dim>& rhs) const {
		Vector<T, dim> res(*this);
		return res -= rhs;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator * (const T& scalar) const {
		Vector<T, dim> res(*this);
		return res *= scalar;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator / (const T& scalar) const {
		Vector<T, dim> res(*this);
		return res /= scalar;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator % (const T& scalar) const {
		Vector<T, dim> res(*this);
		return res %= scalar;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator - () const {
		Vector<T, dim> res(*this);
		return res *= -1;
	}

	CUDA_CALLABLE_MEMBER bool operator == (const Vector<T, dim>& other) const {
		for(int i=0;i<dim;i++)
			if(fnEqual(x[i], other.x[i]))
				return false;
		return	true;
	}

	CUDA_CALLABLE_MEMBER bool operator != (const Vector<T, dim>& other) const {
		for(int i=0;i<dim;i++)
			if(fnEqual(x[i], other.x[i]))
				return true;
		return false;
	}

	CUDA_CALLABLE_MEMBER operator void* () const {
		for(int i=0;i<dim;i++)
			if(x[i] != 0)
				return (void*)this;
		return 0;
	}

	template <class S>
	CUDA_CALLABLE_MEMBER operator Vector<S, dim> () const {
		Vector<S, dim> erg;
		for(int i=0;i<dim;i++)
			erg[i] = (S)x[i];
		return erg;
	}

	CUDA_CALLABLE_MEMBER T operator * (const Vector<T, dim>& rhs) const {
		T sum = 0;
		for(int i=0;i<dim;i++)
			sum += x[i]*rhs.x[i];
		return sum;
	}

	CUDA_CALLABLE_MEMBER T dotProduct(const Vector<T, dim>& rhs) const {
		return *this * rhs;
	}

	CUDA_CALLABLE_MEMBER Vector<T, 3> crossProduct(const Vector<T, 3>& rhs) const {
#ifndef __CUDACC__
		if(dim != 3) throw DimError();
#endif

		return Vector<T, 3>(	(*this)[1]*rhs[2] - (*this)[2]*rhs[1],
								(*this)[2]*rhs[0] - (*this)[0]*rhs[2],
								(*this)[0]*rhs[1] - (*this)[1]*rhs[0]	);
	}

	CUDA_CALLABLE_MEMBER T lengthSqr() const {
		return *this * *this;
	}

	CUDA_CALLABLE_MEMBER T length() const {
		return sqrt(lengthSqr());
	}

	/**
		Teilt den Vektor in Parallel- und Normalteil bzgl. eines anderen Vektors.
		@param ref Referenzvektor
		@return Parallelteil von *this zu ref
	 */
	CUDA_CALLABLE_MEMBER Vector<T, dim> getParallelPartTo(const Vector<T, dim>& ref) const {
		return (*this * ref) * ref / ref.lengthSqr();
	}

	/**
		Genau wie @see getParallelPartTo(), aber es wird ausgenutzt, dass der
		Referenzvektor die Länge 1 hat.
		@param ref Referenzvektor der Länge 1
		@return Parallelteil von *this zu ref
	*/
	CUDA_CALLABLE_MEMBER Vector<T, dim> getParallelPartToNormal(const Vector<T, dim>& ref) const {
		return (*this * ref) * ref;
	}

	CUDA_CALLABLE_MEMBER Vector<T, 3> rotate(f32 phi, const  Vector<T, 3>& axis) const {
#ifndef __CUDACC__
		if(dim != 3) throw DimError();
#endif
		Quaternion<T> q(cos(phi/2), sin(phi/2)*axis);
		return q * *this;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> getNormalized() const {
		if(!*this)
			return *this;

#ifndef __CUDA_ARCH__		
		return *this / length();
#else
		return *this * rsqrtf(lengthSqr());
#endif
	}

	CUDA_CALLABLE_MEMBER u32 size(){
		return dim;
	}
};



template <class T, u32 dim>
CUDA_CALLABLE_MEMBER Vector<T, dim> operator * (const T& scalar, const Vector<T, dim>& vector){
	return vector * scalar;
}

template <class T, u32 dim>
CUDA_CALLABLE_MEMBER T dotProduct(const Vector<T, dim>& lhs, const Vector<T, dim>& rhs){
	return lhs.dotProduct(rhs);
}

template <class T>
CUDA_CALLABLE_MEMBER Vector<T, 3> crossProduct(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs){
	return lhs.crossProduct(rhs);
}


#ifdef __CUDACC__
template <u32 dim>
__inline__ __device__ Vector<f32, dim> atomicAdd(Vector<f32, dim>* addr, const Vector<f32, dim>& value){
	Vector<f32, dim> ret;
	for(u32 i=0; i<dim; i++)
		ret[i] = atomicAdd(&(*addr)[i], value[i]);
	return ret;
}
#endif


typedef Vector<f32, 3> Vector3f;
typedef Vector<f32, 4> Vector4f;
typedef Vector<f64, 3> Vector3d;
typedef Vector<f64, 4> Vector4d;




#endif