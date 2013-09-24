#ifndef _MATRIX_H_
#define _MATRIX_H_


#include "Array.h"
#include "Vector.h"
#include "cuda_macro.h"

#include <iostream>

template <class T, u32 dim>
class Vector;


template <class T, u32 dim>
class Matrix{
public:
	class DimError{};

private:
	Array<T, dim*dim> data;

public:
	CUDA_CALLABLE_MEMBER Matrix():data(){}

	CUDA_CALLABLE_MEMBER Matrix(const Matrix& other):data(other.data){}

	CUDA_CALLABLE_MEMBER Matrix(const T* arr):data(arr){}

	CUDA_CALLABLE_MEMBER Matrix(const Array<T, dim*dim>& arr):data(arr){}

	CUDA_CALLABLE_MEMBER Matrix(	const T& a00, const T& a01, const T& a02,
			const T& a10, const T& a11, const T& a12,
			const T& a20, const T& a21, const T& a22	){
#ifndef __CUDACC__
		if(dim != 3) throw DimError();
#endif
		data[0] = a00;
		data[1] = a01;
		data[2] = a02;
		data[3] = a10;
		data[4] = a11;
		data[5] = a12;
		data[6] = a20;
		data[7] = a21;
		data[8] = a22;
	}

	CUDA_CALLABLE_MEMBER Matrix(	const T& a00, const T& a01, const T& a02, const T& a03,
			const T& a10, const T& a11, const T& a12, const T& a13,
			const T& a20, const T& a21, const T& a22, const T& a23,
			const T& a30, const T& a31, const T& a32, const T& a33	){
#ifndef __CUDACC__
		if(dim != 4) throw DimError();
#endif
		data[ 0] = a00;
		data[ 1] = a01;
		data[ 2] = a02;
		data[ 3] = a03;
		data[ 4] = a10;
		data[ 5] = a11;
		data[ 6] = a12;
		data[ 7] = a13;
		data[ 8] = a20;
		data[ 9] = a21;
		data[10] = a22;
		data[11] = a23;
		data[12] = a30;
		data[13] = a31;
		data[14] = a32;
		data[15] = a33;
	}

	CUDA_CALLABLE_MEMBER Matrix& operator = (const Matrix& other){
		if(this != &other){
			data = other.data;
		}
	
		return *this;
	}

	CUDA_CALLABLE_MEMBER T* operator [] (int i){
		return data.begin() + dim*i;
	}

	CUDA_CALLABLE_MEMBER const T* operator [] (int i) const {
		return data.begin() + dim*i;
	}

	CUDA_CALLABLE_MEMBER T& operator () (int i, int j){
		return data[dim*i + j];
	}

	CUDA_CALLABLE_MEMBER const T& operator () (int i, int j) const {
		return data[dim*i + j];
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim>& operator += (const Matrix<T, dim>& other){
		for(int i=0; i<size(); i++)
			data[i] += other.data[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim>& operator -= (const Matrix<T, dim>& other){
		for(int i=0; i<size(); i++)
			data[i] -= other.data[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim>& operator *= (const T& scalar){
		for(int i=0; i<size(); i++)
			data[i] *= scalar;
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim>& operator /= (const T& scalar){
		for(int i=0; i<size(); i++)
			data[i] /= scalar;
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim> operator + (const Matrix<T, dim>& other) const {
		Matrix<T, dim> erg;
		for(u32 i=0; i<size(); i++)
			erg.data[i] = data[i] + other.data[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim> operator - (const Matrix<T, dim>& other) const {
		Matrix<T, dim> erg;
		for(u32 i=0; i<size(); i++)
			erg.data[i] = data[i] - other.data[i];
		return *this;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim> operator * (const Matrix<T, dim>& other) const {
		Matrix<T, dim> erg;
		for(int i=0; i<dim; i++)
			for(int j=0; j<dim; j++)
				for(int k=0; k<dim; k++)
					erg[i][j] += (*this)[i][k] * other[k][j];
		return erg;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim> operator * (const T& scalar) const {
		Matrix<T, dim> erg = *this;
		return erg *= scalar;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim>& operator *= (const Matrix<T, dim>& other){
		return *this = *this * other;
	}

	CUDA_CALLABLE_MEMBER Vector<T, dim> operator * (const Vector<T, dim>& v) const {
		Vector<T, dim> erg;
		for(int i=0;i<dim;i++)
			for(int j=0;j<dim;j++)
				erg[i] += (*this)[i][j] * v[j];
		return erg;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, dim> transpose() const {
		Matrix<T, dim> erg;
		for(int j=0; j<dim; j++)
			for(int i=0; i<dim; i++)
				erg[j][i] = (*this)[i][j];

		return erg;
	}

	CUDA_CALLABLE_MEMBER T det() const {
		return	+(*this)[0][0] * (*this)[1][1] * (*this)[2][2] 
				+(*this)[0][1] * (*this)[1][2] * (*this)[2][0] 
				+(*this)[0][2] * (*this)[1][0] * (*this)[2][1] 
				-(*this)[0][2] * (*this)[1][1] * (*this)[2][0] 
				-(*this)[0][1] * (*this)[1][0] * (*this)[2][2] 
				-(*this)[0][0] * (*this)[1][2] * (*this)[2][1] ;
	}

	CUDA_CALLABLE_MEMBER Matrix<T, 3> inverse() const {
#ifndef __CUDACC__
		if(dim != 3) throw DimError();
#endif

		T det = this->det();

		return Matrix<T, 3>(+((*this)[1][1]*(*this)[2][2] - (*this)[1][2]*(*this)[2][1]) / det,
							-((*this)[0][1]*(*this)[2][2] - (*this)[0][2]*(*this)[2][1]) / det,
							+((*this)[0][1]*(*this)[1][2] - (*this)[0][2]*(*this)[1][1]) / det,
							-((*this)[1][0]*(*this)[2][2] - (*this)[1][2]*(*this)[2][0]) / det,
							+((*this)[0][0]*(*this)[2][2] - (*this)[0][2]*(*this)[2][0]) / det,
							-((*this)[0][0]*(*this)[1][2] - (*this)[0][2]*(*this)[1][0]) / det,
							+((*this)[1][0]*(*this)[2][1] - (*this)[1][1]*(*this)[2][0]) / det,
							-((*this)[0][0]*(*this)[2][1] - (*this)[0][1]*(*this)[2][0]) / det,
							+((*this)[0][0]*(*this)[1][1] - (*this)[0][1]*(*this)[1][0]) / det	);
	}

	CUDA_CALLABLE_MEMBER u32 size() const {
		return data.size();
	}

	CUDA_CALLABLE_MEMBER u32 dimension() const {
		return dim;
	}
};


template <class T, u32 dim>
CUDA_CALLABLE_MEMBER Vector<T, dim> operator * (const Vector<T, dim>& v, const Matrix<T, dim>& m){
	Vector<T, dim> erg;
		for(int i=0;i<dim;i++)
			for(int j=0;j<dim;j++)
				erg[i] += v[j] * m[j][i];
		return erg;
}

template <class T, u32 dim>
CUDA_CALLABLE_MEMBER Matrix<T, dim> operator * (const T& scalar, const Matrix<T, dim>& m) {
	return m * scalar;
}


typedef Matrix<f32, 3> Matrix3f;
typedef Matrix<f32, 4> Matrix4f;


inline CUDA_CALLABLE_MEMBER Matrix4f createScalarMatrix(f32 sx, f32 sy, f32 sz){
	return Matrix4f(sx,	0,	0,	0,
					0,	sy,	0,	0,
					0,	0,	sz, 0,
					0,	0,	0,	1);
}

template <class T>
inline CUDA_CALLABLE_MEMBER Matrix<T, 4> createTranslationMatrix(const Vector<T, 3>& dist){
	return Matrix4f(1,	0,	0,	dist[0],
					0,	1,	0,	dist[1],
					0,	0,	1,	dist[2],
					0,	0,	0,	1		);
}

inline CUDA_CALLABLE_MEMBER Matrix4f createRotationXMatrix(f32 phi){
	return Matrix4f(	1,	0,			0,			0,
						0,	cos(phi),	-sin(phi),	0,
						0,	sin(phi),	cos(phi),	0,
						0,	0,			0,			1);
}

inline CUDA_CALLABLE_MEMBER Matrix4f createRotationYMatrix(f32 phi){
	return Matrix4f(	cos(phi),	0,	sin(phi),	0,
						0,			1,	0,			0,
						-sin(phi),	0,	cos(phi),	0,
						0,			0,	0,			1);
}

inline CUDA_CALLABLE_MEMBER Matrix4f createRotationZMatrix(f32 phi){
	return Matrix4f(	cos(phi),	-sin(phi),	0, 0,
						sin(phi),	cos(phi),	0, 0,
						0,			0,			1, 0,
						0,			0,			0, 1);
}


const Matrix4f IDENTITY4F = Matrix4f(	1,0,0,0,
										0,1,0,0,
										0,0,1,0,
										0,0,0,1);


template <class T, u32 dim>
std::ostream& operator << (std::ostream& st, const Matrix<T, dim>& m){
	for(u32 i=0; i<dim; i++){
		for(u32 j=0; j<dim; j++){
			st << m[i][j]  << "\t";
		}
		st << std::endl;
	}
	return st;
}


#endif