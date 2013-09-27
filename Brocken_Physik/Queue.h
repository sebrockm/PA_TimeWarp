#ifndef _Queue_H_
#define _Queue_H_

#include "Array.h"
#include "cuda_macro.h"
#include "types.h"


const u32 QL = 20;


template <class Body, u32 Size>
class Queue{
private:
	Array<Body, Size> ar;
	u32 head;//Index vom ersten belegten
	u32 count;

public:
	Queue():head(0), count(0), ar() {}

	CUDA_CALLABLE_MEMBER const Body& front() const {
		return ar[head];
	}

	CUDA_CALLABLE_MEMBER Body& front() {
		return ar[head];
	}

	CUDA_CALLABLE_MEMBER const Body& back() const {
		return ar[(head+count-1)%Size];
	}

	CUDA_CALLABLE_MEMBER Body& back() {
		return ar[(head+count-1)%Size];
	}

	CUDA_CALLABLE_MEMBER Body peek() {
		u32 tmp = head;
		head = (head+1) % Size;
		count--;
		return ar[tmp];
	}

	CUDA_CALLABLE_MEMBER void insert(const Body& b){
		ar[(head+count)%Size] = b;
		count++;
	}

	CUDA_CALLABLE_MEMBER bool empty() const {
		return count == 0;
	}

	CUDA_CALLABLE_MEMBER bool full() const {
		return count == Size;
	}

	CUDA_CALLABLE_MEMBER u32 length() const {
		return count;
	}

	CUDA_CALLABLE_MEMBER u32 size() const {
		return Size;
	}

	CUDA_CALLABLE_MEMBER const Body& operator [] (u32 pos) const {
		return ar[(head+pos)%Size];
	}

	CUDA_CALLABLE_MEMBER Body& operator [] (u32 pos) {
		return ar[(head+pos)%Size];
	}

	// Index des naechst kleineren als b finden
	// falls es keinen kleineren als b gibt, wird -1 zurueckgegeben
	CUDA_CALLABLE_MEMBER int searchFirstBefore(const Body& b) const {
		int first = 0;
		int last = count - 1;

		while(first < last){
			u32 middle = (first + last) / 2;

			if((*this)[middle] < b){ //in hinterer Haelfte weitersuchen, middle kann nicht das gesuchte sein
				first = middle + 1;
			}
			else{ //in vorderer Haelfte weitersuchen, middle koennte das gesuchte sein
				last = middle;
			}
		}

		return first - 1;
	}

	CUDA_CALLABLE_MEMBER int searchFirstBefore(f32 t) const {
		int first = 0;
		int last = count - 1;

		while(first < last){
			u32 middle = (first + last) / 2;

			if((*this)[middle].timestamp < t){ //in hinterer Haelfte weitersuchen, middle kann nicht das gesuchte sein
				first = middle + 1;
			}
			else{ //in vorderer Haelfte weitersuchen, middle koennte das gesuchte sein
				last = middle;
			}
		}

		return first - 1;
	}

	CUDA_CALLABLE_MEMBER void deleteAllGreaterThan(f32 t) {
		Body b;
		b.timestamp = t;
		count = searchFirstBefore(b) + 1;
	}

	CUDA_CALLABLE_MEMBER void deleteAllAfterEq(u32 pos) {
		count = pos;
	}
};



#endif