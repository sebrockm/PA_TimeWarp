#ifndef _HEAP_H_
#define _HEAP_H_


#include "Array.h"
#include "cuda_macro.h"
#include "types.h"


template <class T, u32 Size>
class Heap{
private:
	Array<T, Size> ar;
	u32 count;

public:
	CUDA_CALLABLE_MEMBER Heap():ar(), count(0){}

	CUDA_CALLABLE_MEMBER u32 length() const {
		return count;
	}

	CUDA_CALLABLE_MEMBER bool empty() const {
		return count == 0;
	}

	CUDA_CALLABLE_MEMBER bool full() const {
		return count == Size;
	}

	CUDA_CALLABLE_MEMBER void insert(const T& t){
		u32 id = count++;
		ar[id] = t;
		while(id > 0){
			u32 father = (id-1)/2;
			if(ar[id] < ar[father]){
				T tmp = ar[id];
				ar[id] = ar[father];
				ar[father] = tmp;
				id = father;
			}
			else{
				break;
			}
		}
	}

	CUDA_CALLABLE_MEMBER const T& top() const {
		return ar[0];
	}

	CUDA_CALLABLE_MEMBER T peek(){
		T erg = ar[0];
		ar[0] = ar[--count];
		u32 id = 0;
		u32 left = 1;
		while(left < count){//solange linker Sohn vorhanden
			if(ar[left] < ar[id] || left+1 < count && ar[left+1] < ar[id]){
				u32 tausch = left;
				if(left+1 < count && ar[left+1] < ar[left]){
					tausch++;
				}
				T tmp = ar[id];
				ar[id] = ar[tausch];
				ar[tausch] = tmp;
				id = tausch;
				left = id*2 + 1;
			}
			else{
				break;
			}		
		}
		return erg;
	}

};


#endif