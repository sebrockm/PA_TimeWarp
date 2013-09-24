#ifndef _MYEXCEPTIONS_H_
#define _MYEXCEPTIONS_H_


#include <cuda.h>
#include <cuda_runtime.h>
#include <stdexcept>

using std::runtime_error;


class cuda_exception : public runtime_error {
public:
	explicit cuda_exception(cudaError err):runtime_error(cudaGetErrorString(err)){}
};


inline void throwCudaError(cudaError err){
#if 0
	if(err != cudaSuccess)
		throw cuda_exception(err);
#endif
}



#endif