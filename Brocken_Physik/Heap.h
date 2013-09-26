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
	Heap():ar(), count(0){}

	u32 length() const {
		return count;
	}

	void insert(const T& t){
		u32 id = length++;
		ar[id] = t;
		while(id > 0){
			u32 father = (id-1)/2;
			if(ar[id].timestamp < ar[father].timestamp){
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

	const T& top() const {
		return ar[0];
	}

	T peek(){
		count--;
		T erg = ar[0];
		ar[0] = ar[count-1];
		u32 id = 0;
		u32 left = 1;
		while(id < count && (ar[id].timestamp > ar[left].timestamp || ar[id].timestamp > ar[left+1].timestamp)){
			u32 tausch = left;
			if(ar[left].timestamp > ar[right].timestamp){
				tausch++;
			}
			T tmp = ar[id];
			ar[id] = ar[tausch];
			ar[tausch] = ar[id];
			id = tausch;
		}
		return erg;
	}

};


#endif