#ifndef _Queue_H_
#define _Queue_H_

#include "Array.h"
#include "cuda_macro.h"
#include "types.h"


const u32 QL = 100;


template <class Body, u32 Size>
class Queue{
private:
	Array<Body, Size> ar;
	u32 head;//Index vom ersten belegten
	u32 count;

public:
	Queue():head(0), count(0), ar() {}

	bool operator < (const Queue<Body, Size>& other) const {
		return length() < other.length();
	}

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

	CUDA_CALLABLE_MEMBER Body peekFront() {
		u32 tmp = head;
		head = (head+1) % Size;
		count--;
		return ar[tmp];
	}

	CUDA_CALLABLE_MEMBER Body peekBack() {
		u32 tmp = (head + count) % Size;
		count--;
		return ar[tmp];
	}

	CUDA_CALLABLE_MEMBER void insertBack(const Body& b){
#ifdef DOPRINT
			if(count >= Size) printf("q voll\n");
#endif
		ar[(head+count)%Size] = b;
		count++;
	}

	CUDA_CALLABLE_MEMBER void insertFront(const Body& b){
#ifdef DOPRINT
		if(count >= Size) printf("q voll\n");
#endif
		head = (head + Size - 1) % Size;
		ar[head] = b;
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

	CUDA_CALLABLE_MEMBER void removeAll(){
		count = 0;
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
	CUDA_CALLABLE_MEMBER int searchFirstBeforeEq(const Body& b) const {
		int first = 0;
		//int last = count - 1;

		//while(first < last){
		//	u32 middle = (first + last) / 2;

		//	if((*this)[middle] < b){ //in hinterer Haelfte weitersuchen, middle kann nicht das gesuchte sein
		//		first = middle + 1;
		//	}
		//	else{ //in vorderer Haelfte weitersuchen, middle koennte das gesuchte sein
		//		last = middle;
		//	}
		//}

		//return first - 1;
		while(first < count && (*this)[first] < b) first++;
		return first;
	}

	CUDA_CALLABLE_MEMBER int searchFirstBeforeEq(f32 t) const {
		int first = 0;
		//int last = count - 1;

		//while(first < last){
		//	u32 middle = (first + last) / 2;

		//	if((*this)[middle].timestamp < t){ //in hinterer Haelfte weitersuchen, middle kann nicht das gesuchte sein
		//		first = middle + 1;
		//	}
		//	else{ //in vorderer Haelfte weitersuchen, middle koennte das gesuchte sein
		//		last = middle;
		//	}
		//}

		//return first - 1;
		while(first < count && (*this)[first].timestamp < t) first++;
		return first;
	}

	CUDA_CALLABLE_MEMBER void deleteAllGreaterThan(f32 t) {
		count = searchFirstBeforeEq(t);
	}

	CUDA_CALLABLE_MEMBER void deleteAllAfterEq(u32 pos) {
		count = pos;
	}
};



#endif