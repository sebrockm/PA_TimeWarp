#ifndef _LINESEGMENT_H_
#define _LINESEGMENT_H_

#include "Vector.h"
#include "Plane.h"


class Plane;




class LineSegment{
public:
	Vector3f a, b, dir;

	LineSegment();

	LineSegment(const Vector3f& start, const Vector3f& dest);

	Vector3f getIntersectionPoint(const Plane& p) const;
};




#endif