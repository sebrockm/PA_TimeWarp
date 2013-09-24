#ifndef _ARRAY_H_
#define _ARRAY_H_

#include "cuda_macro.h"
#include "types.h"

template <class T, u32 Size>
class Array{
public:
	T data[Size];

	typedef T* iterator;
	typedef const T* const_iterator;


	//CUDA_CALLABLE_MEMBER Array(){
	//	for(u32 i = 0; i < Size; i++)
	//		data[i] = T();
	//}

	//template <u32 Size2>
	//CUDA_CALLABLE_MEMBER Array(const Array<T, Size2>& other){
	//	u32 min = Size < Size2 ? Size : Size2;
	//	for(int i=0; i<min; i++)
	//		data[i] = other.data[i];
	//	for(int i=min; i<Size; i++) //ggf. Rest mit 0 füllen
	//		data[i] = T();
	//}

	//CUDA_CALLABLE_MEMBER Array(const T* arr){
	//	for(int i=0; i<Size; i++)
	//		data[i] = arr[i];
	//}

	//template <u32 Size2>
	//CUDA_CALLABLE_MEMBER Array<T, Size>& operator = (const Array<T, Size2>& other){
	//	if(this != &other){
	//		u32 min = Size < Size2 ? Size : Size2;
	//		for(int i=0; i<min; i++)
	//			data[i] = other.data[i];
	//		for(int i=min; i<Size; i++) //ggf. Rest mit 0 füllen
	//			data[i] = T();
	//	}

	//	return *this;
	//}

	CUDA_CALLABLE_MEMBER T& operator [] (u32 i){
		return data[i];
	}

	CUDA_CALLABLE_MEMBER const T& operator [] (u32 i) const {
		return data[i];
	}

	CUDA_CALLABLE_MEMBER u32 size() const {
		return Size;
	}

	CUDA_CALLABLE_MEMBER iterator begin(){
		return &data[0];
	}

	CUDA_CALLABLE_MEMBER const_iterator begin() const {
		return &data[0];
	}

	CUDA_CALLABLE_MEMBER iterator end(){
		return &data[Size];
	}

	CUDA_CALLABLE_MEMBER const_iterator end() const {
		return &data[Size];
	}
};



#endif