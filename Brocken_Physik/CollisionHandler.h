#ifndef _COLLISIONHANDLER_H_
#define _COLLISIONHANDLER_H_


#include "Sphere.h"
#include "Plane.h"
#include "Cuboid.h"
#include "FixCuboid.h"
#include "material.h"
#include "cuda_macro.h"





class CollisionHandler{
public:

	CUDA_CALLABLE_MEMBER void operator () (
		const Cuboid& c1, const Cuboid& c2, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f v1Orth = c1.v.getParallelPartToNormal(n);

		//Parallelteil
		Vector3f v1Par = c1.v - v1Orth;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omega1Orth = c1.omega.getParallelPartToNormal(n);

		//und der parallele Teil
		Vector3f omega1Par = c1.omega - omega1Orth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c1.k][c2.k];
		f32 mue = staticFricCoef[c1.k][c2.k];

		dPos += correctPosition(c1, Plane(n, pt));

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f r1 = pt - c1.x;
		Vector3f r2 = pt - c2.x;

		Matrix3f invTheta1 = c1.getInverseInertia();
		Matrix3f invTheta2 = c2.getInverseInertia();

		f32 v = n * (c1.v + crossProduct(c1.omega, r1) - c2.v - crossProduct(c2.omega, r2));
		f32 t1 = n * crossProduct(invTheta1*crossProduct(r1, n), r1);
		f32 t2 = n * crossProduct(invTheta2*crossProduct(r2, n), r2);

		//Kraftstoß (Impulsänderung)
		Vector3f F = n * ((-1-res) * v / (1/c1.m + 1/c2.m + t1 + t2));

		//neuer Orthogonalteil der Geschwindigkeit
		v1Orth += F/c1.m;

		//neuer Parallelteil der Winkelgeschwindigkeit
		omega1Par += invTheta1 * crossProduct(r1, F);

		//neue Winkelgeschwindigkeit
		dRot += omega1Par + (1-mue)*omega1Orth - c1.omega;

		//abhängig von Reibung Geschwindigkeit aus Winkelgeschwindigkeit bestimmen
		v1Par = mue * crossProduct(r1, omega1Par);

		//neue Geschwindigkeit
		dSpeed += v1Orth	+ v1Par - v1Par.getParallelPartToNormal(n) - c1.v;

	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Cuboid& c, const Plane& p, 
		const Vector3f& pt,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{

		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f vOrth = c.v.getParallelPartToNormal(p.n);

		//Parallelteil
		Vector3f vPar = c.v - vOrth;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omegaOrth = c.omega.getParallelPartToNormal(p.n);

		//und der parallele Teil
		Vector3f omegaPar = c.omega - omegaOrth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c.k][p.k];
		f32 mue = staticFricCoef[c.k][p.k];
		
		//Quader und Ebene sollen sich nicht schneiden
		dPos += correctPosition(c, p);

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f r = pt - c.x;

		Matrix3f invTheta = c.getInverseInertia();

		f32 v = p.n * (c.v + crossProduct(c.omega, r));
		f32 t = p.n * crossProduct(invTheta*crossProduct(r, p.n), r);

		//Kraftstoß (Impulsänderung)
		Vector3f F = p.n * ((-1-res) * v / (1/c.m + t));

		//neuer Orthogonalteil der Geschwindigkeit
		vOrth += F/c.m;

		//neuer Parallelteil der Winkelgeschwindigkeit
		omegaPar += invTheta * crossProduct(r, F);

		//neue Winkelgeschwindigkeit
		dRot += .9f*(omegaPar + (1-mue)*omegaOrth - c.omega);

		//neue Parallelgeschwindigkeit abhängig von Reibung
		vPar = mue * crossProduct(r, omegaPar);

		//neue Geschwindigkeit
		dSpeed += .9f*(vOrth + vPar - vPar.getParallelPartToNormal(p.n) - c.v);
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Plane& p, const Cuboid& c, 
		const Vector3f& pt,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
		operator () (c, p, pt, dPos, dSpeed, dRot);
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Cuboid& c, const Sphere& s, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{	
		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f vOrthC = c.v.getParallelPartToNormal(n);

		//Parallelteil
		Vector3f vParC = c.v - vOrthC;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omegaOrthC = c.omega.getParallelPartToNormal(n);

		//und der parallele Teil
		Vector3f omegaParC = c.omega - omegaOrthC;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c.k][s.k];
		f32 mue = staticFricCoef[c.k][s.k];
		
		//Quader und Ebene sollen sich nicht schneiden
		dPos += correctPosition(c, Plane(n, pt));

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f rC = pt - c.x;
		Vector3f rS = pt - s.x;

		//Zerlegung des Radius
		Vector3f rParC = rC - rC.getParallelPartToNormal(n);

		Matrix3f invTheta = c.getInverseInertia();

		f32 v = n * (c.v + crossProduct(c.omega, rC) - s.v - crossProduct(s.omega, rS));
		f32 tC = n * crossProduct(invTheta*crossProduct(rC, n), rC);
		//f32 tS = 2.5f * 1/s.m;

		//Kraftstoß (Impulsänderung)
		Vector3f F = n * ((-1-res) * v / (1/c.m + 3.5f/s.m + tC));

		//neuer Orthogonalteil der Geschwindigkeit
		vOrthC += F/c.m;

		//neuer Parallelteil der Winkelgeschwindigkeit
		omegaParC += invTheta * crossProduct(rC, F);

		//neue Winkelgeschwindigkeit
		dRot += omegaParC + (1-mue)*omegaOrthC - c.omega;
		//s.omega = omegaParS + res * omegaOrthS;

		//neue Parallelgeschwindigkeit abhängig von Reibung
		vParC = mue * crossProduct(rC, omegaParC);

		//neue Geschwindigkeit
		dSpeed += vOrthC + vParC - vParC.getParallelPartToNormal(n) - c.v;
	
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Sphere& s, const Cuboid& c, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{	
		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f vOrthC = c.v.getParallelPartToNormal(n);
		Vector3f vOrthS = s.v.getParallelPartToNormal(n);

		//Parallelteil
		Vector3f vParC = c.v - vOrthC;
		Vector3f vParS = s.v - vOrthS;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omegaOrthC = c.omega.getParallelPartToNormal(n);
		Vector3f omegaOrthS = s.omega.getParallelPartToNormal(n);

		//und der parallele Teil
		Vector3f omegaParC = c.omega - omegaOrthC;
		Vector3f omegaParS = s.omega - omegaOrthS;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c.k][s.k];
		f32 mue = staticFricCoef[c.k][s.k];
		
		//Quader und Ebene sollen sich nicht schneiden
		dPos += correctPosition(s, Plane(n, pt));

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f rC = pt - c.x;
		Vector3f rS = pt - s.x;

		//Zerlegung des Radius
		Vector3f rParC = rC - rC.getParallelPartToNormal(n);
		Vector3f rParS = rS - rS.getParallelPartToNormal(n);

		//neuer Parallelteil der Winkelgeschwindigkeit
		Vector3f omegaParNeu = (1.f-5.f/7*mue) * omegaParS - 
			5.f/7*mue / (s.r*s.r) * rS.crossProduct(vParS);

		//neue Winkelgeschwindigkeit
		dRot += omegaParNeu + /*(1-mue)**/omegaOrthS - s.omega;//keine Reibung wegen Kugel

	
		Vector3f tmpv = s.m*vOrthS + c.m*vOrthC;
		f32 m = s.m + c.m;

		//neue Geschwindigkeiten
		dSpeed += (tmpv - res * c.m * (vOrthS-vOrthC)) / m //neuer Orthogonalteil
			+ (1-mue) * vParS + crossProduct((1-mue)*omegaParS-omegaParNeu, rS) //neuer Parallelteil
			- s.v;
	}



	CUDA_CALLABLE_MEMBER void operator () (
		Sphere& s1, const Sphere& s2) const
	{
		Vector3f n = (s2.x-s1.x).getNormalized();

		//Teile der Geschwindigkeiten, die senkrecht zur Berührebene liegen
		Vector3f v1Orth = s1.v.getParallelPartToNormal(n);
		Vector3f v2Orth = s2.v.getParallelPartToNormal(n);

		//Parallelteile
		Vector3f v1Par = s1.v - v1Orth;
		Vector3f v2Par = s2.v - v2Orth;

		//Teile der Rotationsgeschwindigkeiten, die senkrecht zur Berührebene liegen
		Vector3f omega1Orth = s1.omega.getParallelPartToNormal(n);

		//und die Parallelteile
		Vector3f omega1Par = s1.omega - omega1Orth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[s1.k][s2.k];
		f32 mue = staticFricCoef[s1.k][s2.k];

		//Radius als Vektoren
		Vector3f r1 = n * s1.r;

		//neue Parallelteile der Rotationen
		Vector3f omegaParNeu = (1.f-5.f/7*mue) * omega1Par - 
			5.f/7*mue / (s1.r*s1.r) * r1.crossProduct(v1Par);

		//neue Winkelgeschwindigkeiten
		s1.omega = omegaParNeu + /*res **/ omega1Orth;

		Vector3f tmpv = s1.m*v1Orth + s2.m*v2Orth;
		f32 m = s1.m + s2.m;

		//neue Geschwindigkeiten
		s1.v = (tmpv - res * s2.m * (v1Orth-v2Orth)) / m //neuer Orthogonalteil
			+ (1-mue) * v1Par + crossProduct((1-mue)*omega1Par-omegaParNeu, r1); //neuer Parallelteil;
	}




	CUDA_CALLABLE_MEMBER void operator () (
		const Sphere& s1, const Sphere& s2, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{

		//Berührebene erzeugen
		Plane p(n, pt);

		//Position von s2 korrigieren
		dPos += correctPosition(s1, p);

		//Teile der Geschwindigkeiten, die senkrecht zur Berührebene liegen
		Vector3f v1Orth = s1.v.getParallelPartToNormal(n);
		Vector3f v2Orth = s2.v.getParallelPartToNormal(n);

		//Parallelteile
		Vector3f v1Par = s1.v - v1Orth;
		Vector3f v2Par = s2.v - v2Orth;

		//Teile der Rotationsgeschwindigkeiten, die senkrecht zur Berührebene liegen
		Vector3f omega1Orth = s1.omega.getParallelPartToNormal(n);

		//und die Parallelteile
		Vector3f omega1Par = s1.omega - omega1Orth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[s1.k][s2.k];
		f32 mue = staticFricCoef[s1.k][s2.k];

		//Radien als Vektoren
		Vector3f r1 = (pt - s1.x).getNormalized() * s1.r;

		//neue Parallelteile der Rotationen
		Vector3f omegaParNeu = (1.f-5.f/7*mue) * omega1Par - 
			5.f/7*mue / (s1.r*s1.r) * r1.crossProduct(v1Par);

		//neue Winkelgeschwindigkeiten
		dRot += omegaParNeu + /*res **/ omega1Orth - s1.omega;

		Vector3f tmpv = s1.m*v1Orth + s2.m*v2Orth;
		f32 m = s1.m + s2.m;

		//neue Geschwindigkeiten
		dSpeed += (tmpv - res * s2.m * (v1Orth-v2Orth)) / m //neuer Orthogonalteil
			+ (1-mue) * v1Par + crossProduct((1-mue)*omega1Par-omegaParNeu, r1) //neuer Parallelteil
			- s1.v;
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Sphere& s, const Plane& p, 
		const Vector3f& pt,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{

		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f vOrth = s.v.getParallelPartToNormal(p.n);

		//Parallelteil
		Vector3f vPar = s.v - vOrth;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omegaOrth = s.omega.getParallelPartToNormal(p.n);

		//und der parallele Teil
		Vector3f omegaPar = s.omega - omegaOrth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[s.k][p.k];
		f32 mue = staticFricCoef[s.k][p.k];
		
		//Kugel und Ebene sollen sich nicht schneiden
		dPos += correctPosition(s, p);										

		// Radius als Vektor
		Vector3f r = (pt - s.x).getNormalized() * s.r;

		//neuer Parallelteil der Rotation
		Vector3f omegaParNeu = (1.f-5.f/7*mue) * omegaPar - 
			5.f/7*mue / (s.r*s.r) * r.crossProduct(vPar);

		f32 f = fNearlyEqual(abs(p.n[1]),1.f) ? 1 : .99f;

		//neue Winkelgeschwindigkeit
		dRot += f*(omegaParNeu + /*res **/ omegaOrth - s.omega);

		//neue Geschwindigkeit
		dSpeed += f*(-res*vOrth //neuer Orthogonalteil
			+ (1-mue) * vPar + crossProduct((1-mue)*omegaPar-omegaParNeu, r) //neuer Parallelteil
			- s.v);
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Cuboid& c1, const FixCuboid& c2, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
	
		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f v1Orth = c1.v.getParallelPartToNormal(n);

		//Parallelteil
		Vector3f v1Par = c1.v - v1Orth;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omega1Orth = c1.omega.getParallelPartToNormal(n);

		//und der parallele Teil
		Vector3f omega1Par = c1.omega - omega1Orth;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c1.k][c2.k];
		f32 mue = staticFricCoef[c1.k][c2.k];

		dPos += correctPosition(c1, Plane(n, pt));

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f r1 = pt - c1.x;

		Matrix3f invTheta1 = c1.getInverseInertia();

		f32 v = n * (c1.v + crossProduct(c1.omega, r1));
		f32 t1 = n * crossProduct(invTheta1*crossProduct(r1, n), r1);

		//Kraftstoß (Impulsänderung)
		Vector3f F = n * ((-1-res) * v / (1/c1.m + t1));

		//neuer Orthogonalteil der Geschwindigkeit
		v1Orth += F/c1.m;

		//neuer Parallelteil der Winkelgeschwindigkeit
		omega1Par += invTheta1 * crossProduct(r1, F);

		//neue Winkelgeschwindigkeit
		dRot += omega1Par + (1-mue)*omega1Orth - c1.omega;

		//abhängig von Reibung Geschwindigkeit aus Winkelgeschwindigkeit bestimmen
		v1Par = mue * crossProduct(r1, omega1Par);

		//neue Geschwindigkeit
		dSpeed += v1Orth	+ v1Par - v1Par.getParallelPartToNormal(n) - c1.v;

	}

	CUDA_CALLABLE_MEMBER void operator () (
		const FixCuboid& fc, const Cuboid& c, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
		return operator() (c, fc, pt, n, dPos, dSpeed, dRot);
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const Sphere& s, const FixCuboid& c, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
		//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f vOrthS = s.v.getParallelPartToNormal(n);

		//Parallelteil
		Vector3f vParS = s.v - vOrthS;

		//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
		Vector3f omegaOrthS = s.omega.getParallelPartToNormal(n);

		//und der parallele Teil
		Vector3f omegaParS = s.omega - omegaOrthS;

		//Koeffizienten abhängig von den Materialien
		f32 res = resCoef[c.k][s.k];
		f32 mue = staticFricCoef[c.k][s.k];
		
		//Quader und Ebene sollen sich nicht schneiden
		dPos += correctPosition(s, Plane(n, pt));

		// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
		Vector3f rC = pt - c.x;
		Vector3f rS = pt - s.x;

		//Zerlegung des Radius
		Vector3f rParC = rC - rC.getParallelPartToNormal(n);
		Vector3f rParS = rS - rS.getParallelPartToNormal(n);

		//neuer Parallelteil der Winkelgeschwindigkeit
		Vector3f omegaParNeu = (1.f-5.f/7*mue) * omegaParS - 
			5.f/7*mue / (s.r*s.r) * rS.crossProduct(vParS);

		//neue Winkelgeschwindigkeit
		dRot += omegaParNeu + /*(1-mue)**/omegaOrthS - s.omega;//keine Reibung wegen Kugel


		//neue Geschwindigkeiten
		dSpeed += -res * vOrthS //neuer Orthogonalteil
			+ (1-mue) * vParS + crossProduct((1-mue)*omegaParS-omegaParNeu, rS)  //neuer Parallelteil
			- s.v;
	}

	CUDA_CALLABLE_MEMBER void operator () (
		const FixCuboid& fc, const Sphere& s, 
		const Vector3f& pt, const Vector3f& n,
		Vector3f& dPos, Vector3f& dSpeed, Vector3f& dRot) const
	{
		return operator () (s, fc, pt, n, dPos, dSpeed, dRot);
	}

	/*CUDA_CALLABLE_MEMBER void operator () (const Plane& p, Sphere& s, const Vector3f& pt) const{
		operator () (s, p, pt);
	}*/


private:
	CUDA_CALLABLE_MEMBER Vector3f correctPosition(const Sphere& s, const Plane& p) const{
		//Position der Kugel sollte auf jeden Fall vollständig "diesseits"
		//der Ebene bleiben. Dazu einfach die neue Position der Kugel 
		//korrigieren, indem senkrecht zur Ebene zurückgesetzt wird
		f32 d = p.orientatedDistanceTo(s.x);

		//in Abhängigkeit von der Seite, auf der sich der Mittelpunkt befindet,
		//entweder den Radius zum Abstand addieren oder subtrahieren
		f32 r = s.r + EPSILON;
		d += d>0 ? -r : r;
		
		//s.x -= d*p.n;//Position korrigieren

		return -d*p.n;
	}

	CUDA_CALLABLE_MEMBER Vector3f correctPosition(const Cuboid& c, const Plane& p) const {

		f32 d = p.orientatedDistanceTo(c.x);

		//in Abhängigkeit von der Seite, auf der sich der Mittelpunkt befindet,
		//entweder den Radius zum Abstand addieren oder subtrahieren
		f32 r = c.getProjectionOnNormal(p.n)/2 + EPSILON;

		d += d>0 ? -r : r;

		return -d*p.n;
	}
};





#endif