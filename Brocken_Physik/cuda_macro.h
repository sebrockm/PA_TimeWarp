#ifndef _CUDA_MACRO_H_
#define _CUDA_MACRO_H_



#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__ 
#define CUDA_DEVICE __device__
#else
#define CUDA_CALLABLE_MEMBER
#define CUDA_CONST_VAR const
#define CUDA_DEVICE
#endif 


#ifdef __CUDA_ARCH__
#define CUDA_CONST_VAR __device__ const
#else
#define CUDA_CONST_VAR const
#endif



#endif