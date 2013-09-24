#ifndef _CAMERA_H_
#define _CAMERA_H_


#include "Vector.h"
#include "Matrix.h"



class Camera{
public:
	Vector3f pos;
	Vector3f upDir, sideDir, viewDir;
	const Vector3f UPDIR, SIDEDIR, VIEWDIR;
	f32 phi, theta;
	Matrix4f frustum;

	static Matrix4f createFrustum(f32 l, f32 r, f32 b, f32 t, f32 n, f32 f){
		return Matrix4f(2*n/(r-l),	0.f,		(r+l)/(r-l),	0.f,
						0.f,		2*n/(t-b),	(t+b)/(t-b),	0.f,
						0.f,		0.f,		-(f+n)/(f-n),	-2*n*f/(f-n),
						0.f,		0.f,		-1.0f,			0.f			);
	}

	static Matrix4f createView(const Vector3f& eye, const Vector3f& viewDir, const Vector3f& up){
		Vector3f nViewDir = viewDir.getNormalized();
		Vector3f nSideDir = crossProduct(viewDir, up).getNormalized();
		Vector3f nUpDir = crossProduct(nSideDir, nViewDir);

		return Matrix4f(nSideDir[0],	nSideDir[1],	nSideDir[2],	-eye*nSideDir,
						nUpDir[0],		nUpDir[1],		nUpDir[2],		-eye*nUpDir,
						-nViewDir[0],	-nViewDir[1],	-nViewDir[2],	eye*nViewDir,
						0.f,			0.f,			0.f,			1.f				);
	}

	static const f32 FRUSTNEAR;
	static const f32 FRUSTFAR;

public:
	Camera(): pos(0,0,0), UPDIR(0,1,0), upDir(0,1,0), SIDEDIR(-1,0,0), sideDir(-1,0,0), VIEWDIR(0,0,-1), viewDir(0,0,-1), phi(0), theta(0)
	{}

	void move(f32 fb, f32 lr, f32 ud);

	void rotate(f32 dPhi, f32 dTheta);

	void changeViewport(int w, int h);

	Matrix4f getViewProj();
};



#endif