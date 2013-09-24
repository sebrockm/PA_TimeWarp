#include "Matrix.h"
#include "Vector.h"


inline CUDA_CALLABLE_MEMBER Matrix4f createScalarMatrix(f32 sx, f32 sy, f32 sz){
	return Matrix4f(sx,	0,	0,	0,
					0,	sy,	0,	0,
					0,	0,	sz, 0,
					0,	0,	0,	1);
}

inline CUDA_CALLABLE_MEMBER Matrix4f createTranslationMatrix(const Vector3f& dist){
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
