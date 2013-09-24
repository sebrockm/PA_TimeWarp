#include "Camera.h"


void Camera::move(f32 fb, f32 lr, f32 ud){
	pos += fb*viewDir + lr*sideDir + ud*upDir;
}

void Camera::rotate(f32 dPhi, f32 dTheta){
	phi += dPhi;
		
	theta += dTheta;
	if(theta > PI/2){
		theta = PI/2;
	}
	if(theta < -PI/2){
		theta = -PI/2;
	}

	Matrix4f rot = createRotationYMatrix(phi) * createRotationXMatrix(theta);

	sideDir = -Vector3f(rot[0][0], rot[1][0], rot[2][0]);
	upDir = Vector3f(rot[0][1], rot[1][1], rot[2][1]);
	viewDir = -Vector3f(rot[0][2], rot[1][2], rot[2][2]);
}

void Camera::changeViewport(int w, int h){
	float top = (float)h/w * FRUSTNEAR/2;
	frustum = createFrustum(-FRUSTNEAR/2, FRUSTNEAR/2, -top, top, FRUSTNEAR, FRUSTFAR);
}

Matrix4f Camera::getViewProj(){
	return frustum * createView(pos, viewDir, upDir);
}


const f32 Camera::FRUSTNEAR = .01f;
const f32 Camera::FRUSTFAR = 1000.f;