#include "LineSegment.h"
#include "Plane.h"
#include "Vector.h"



LineSegment::LineSegment():a(), b(), dir() {}

LineSegment::LineSegment(const Vector3f& start, const Vector3f& dest):a(start), b(dest), dir(b-a) {}

Vector3f LineSegment::getIntersectionPoint(const Plane& p) const {
		Vector3f n = p.n;
		f32 d = p.d;
		f32 t = (d - n*a) / (n*dir);
		if(0 <= t && t <= 1){
			return a + t*dir;
		}
		return Vector3f(INFINITY, INFINITY, INFINITY);
	}