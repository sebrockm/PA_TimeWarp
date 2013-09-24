#ifndef _STATEQUEUE_H_
#define _STATEQUEUE_H_

#include "Array.h"
#include "cuda_macro.h"
#include "types.h"

template <class Body, u32 Size>
class StateQueue{
private:
	Array<Body, Size> ar;
	u32 head;//Index vom ersten belegten
	u32 count;

public:
	StateQueue():head(0), count(0), ar() {}

	CUDA_CALLABLE_MEMBER const Body& top() const {
		return ar[head];
	}

	CUDA_CALLABLE_MEMBER Body top() {
		return ar[head];
	}

	CUDA_CALLABLE_MEMBER Body pop() {
		u32 tmp = head;
		head = (head+1) % Size;
		count--;
		return ar[tmp];
	}

	CUDA_CALLABLE_MEMBER void push(const Body& b){
		count++;
		ar[(head+count)%Size] = b;
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

	CUDA_CALLABLE_MEMBER Body get(u32 pos) const {
		return ar[(head+pos)%Size];
	}

	// naechst groesseres als t finden
	// falls es keinen groesseren als t gibt, wird Size zurueckgegeben
	CUDA_CALLABLE_MEMBER u32 searchNext(f32 t) const {		
		u32 first = head;
		u32 last = (head + count - 1) % Size;

		if(ar[last].timestamp < t) // gibt es ueberhaupt einen groesseren?
			return Size;

		while(last != first){
			u32 middle = first + last + (first>last) * Size;

			middle = (middle / 2) % Size;
			if(t < ar[middle].timestamp){ //in vorderer Haelfte weitersuchen, middle koennte das gesuchte sein
				last = middle;
			}
			else{ //in hinterer Haelfte weitersuchen, middle kann nicht das gesuchte sein
				first = (middle+1) % Size;
			}
		}

		return first;
	}

};



#endif