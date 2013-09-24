#include "CollisionHandler.h"


inline void CollisionHandler::operator () (Cuboid& c1, Cuboid& c2, const Vector3f& pt, const Vector3f& n) const {

	//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f v1Orth = c1.v.getParallelPartToNormal(n);
	Vector3f v2Orth = c2.v.getParallelPartToNormal(n);

	//Parallelteil
	Vector3f v1Par = c1.v - v1Orth;
	Vector3f v2Par = c2.v - v2Orth;

	//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f omega1Orth = c1.omega.getParallelPartToNormal(n);
	Vector3f omega2Orth = c2.omega.getParallelPartToNormal(n);

	//und der parallele Teil
	Vector3f omega1Par = c1.omega - omega1Orth;
	Vector3f omega2Par = c2.omega - omega2Orth;

	//Koeffizienten abhängig von den Materialien
	f32 res = resCoef[c1.k][c2.k];
	//f32 mue = staticFricCoef[c1.k][c2.k];

	// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
	Vector3f r1 = pt - c1.x;
	Vector3f r2 = pt - c2.x;
	//c1.x -= .01f*r1;
	//c2.x -= .01f*r2;

	//Zerlegung des Radius
	Vector3f r1Par = r1 - r1.getParallelPartToNormal(n);
	Vector3f r2Par = r2 - r2.getParallelPartToNormal(n);

	//Rotationsachse ist senkrecht zu r und der Normalen
	Vector3f rotN1 = crossProduct(r1, n);
	Vector3f rotN2 = crossProduct(r2, n);

	//Trägheitsmoment bezüglich neuer Rotationsachse
	f32 theta1 = rotN1 ? rotN1 * ((c1.getInertia()+getSteinerSummand(r1, c1.m)) * rotN1) / rotN1.lengthSqr() : 0;
	f32 theta2 = rotN2 ? rotN2 * ((c2.getInertia()+getSteinerSummand(r2, c2.m)) * rotN2) / rotN2.lengthSqr() : 0;

	//Kraftstoß (Impulsänderung)
	Vector3f F = (-1-res) * (v1Orth + crossProduct(c1.omega, r1).getParallelPartToNormal(n)
		- v2Orth - crossProduct(c2.omega, r2).getParallelPartToNormal(n)) / 
		(1/c1.m + 1/c2.m + (theta1==0 ? 0 : r1Par.lengthSqr()/theta1) + (theta2==0 ? 0 : r2Par.lengthSqr()/theta2));

	//neuer Orthogonalteil der Geschwindigkeit
	v1Orth += F/c1.m;
	v2Orth -= F/c2.m;

	//neuer Parallelteil der Winkelgeschwindigkeit
	omega1Par += (c1.getInertia()+getSteinerSummand(r1, c1.m)).inverse() * crossProduct(r1, F);
	omega2Par -= (c2.getInertia()+getSteinerSummand(r2, c2.m)).inverse() * crossProduct(r2, F);

	//neue Winkelgeschwindigkeit
	c1.omega = .99f*(omega1Par + omega1Orth);
	c2.omega = .99f*(omega2Par + omega2Orth);

	//abhängig von Reibung Geschwindigkeit aus Winkelgeschwindigkeit bestimmen
	v1Par = crossProduct(r1, omega1Par);
	v2Par = crossProduct(r2, omega2Par);

	//neue Geschwindigkeit
	c1.v = .99f*(v1Orth	+ v1Par - v1Par.getParallelPartToNormal(n));
	c2.v = .99f*(v2Orth	+ v2Par - v2Par.getParallelPartToNormal(n));

}

inline void CollisionHandler::operator () (Cuboid& c, const Plane& p, const Vector3f& pt) const {

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
	//c.x -= p.orientatedDistanceTo(pt)*p.n;

	// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
	Vector3f r = pt - c.x;

	//Zerlegung des Radius
	Vector3f rPar = r - r.getParallelPartToNormal(p.n);

	//Rotationsachse ist senkrecht zu r und lot
	Vector3f rotN = crossProduct(r, p.n);

	//Trägheitsmoment bezüglich neuer Rotationsachse
	f32 theta = rotN.lengthSqr()==0 ? 0 : rotN * ((c.getInertia()+getSteinerSummand(r, c.m)) * rotN) / rotN.lengthSqr();

	//Kraftstoß (Impulsänderung)
	Vector3f F = (-1-res) * (vOrth + crossProduct(c.omega, r).getParallelPartToNormal(p.n)) / 
		(1/c.m + (theta==0 ? 0 : rPar.lengthSqr()/theta));

	//neuer Orthogonalteil der Geschwindigkeit
	vOrth += F/c.m;

	//neuer Parallelteil der Winkelgeschwindigkeit
	omegaPar += (c.getInertia()+getSteinerSummand(r, c.m)).inverse() * crossProduct(r, F);

	//neue Winkelgeschwindigkeit
	c.omega = .99f*(omegaPar + omegaOrth);

	//neue Parallelgeschwindigkeit abhängig von Reibung
	vPar = mue * crossProduct(r, omegaPar)-crossProduct(r, omegaPar).getParallelPartToNormal(p.n);

	//neue Geschwindigkeit
	c.v = .99f*(vOrth + vPar - vPar.getParallelPartToNormal(p.n));
}


inline void CollisionHandler::operator () (const Plane& p, Cuboid& c, const Vector3f& pt) const {
	operator () (c, p, pt);
}


inline void CollisionHandler::operator () (Cuboid& c, Sphere& s, const Vector3f& pt, const Vector3f& n) const {
	
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
	Vector3f omegaParS = c.omega - omegaOrthS;

	//Koeffizienten abhängig von den Materialien
	f32 res = resCoef[c.k][s.k];
	//f32 mue = staticFricCoef[c.k][p.k];
		
	//Quader und Ebene sollen sich nicht schneiden
	/*Plane p(n, pt);
	correctPosition(s, p);*/

	// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
	Vector3f rC = pt - c.x;
	Vector3f rS = pt - s.x;

	//Zerlegung des Radius
	Vector3f rParC = rC - rC.getParallelPartToNormal(n);
	Vector3f rParS = rS - rS.getParallelPartToNormal(n);

	//Rotationsachse ist senkrecht zu r und lot
	Vector3f rotN = crossProduct(rC, n);

	//Trägheitsmoment bezüglich neuer Rotationsachse
	f32 theta = rotN.lengthSqr()==0 ? 0 : rotN * ((c.getInertia()+getSteinerSummand(rC, c.m)) * rotN) / rotN.lengthSqr();

	//Kraftstoß (Impulsänderung)
	Vector3f F = (-1-res) * (vOrthC + crossProduct(omegaParC, rC).getParallelPartToNormal(n)
		- vOrthS - crossProduct(omegaParS, rS).getParallelPartToNormal(n)) / 
		(1/c.m + 1/s.m + (theta==0 ? 0 : rParC.lengthSqr()/theta) + /*rParS.lengthSqr()*/1.f/(.4f*s.m/**s.r*s.r*/));


	//neuer Orthogonalteil der Geschwindigkeit
	vOrthS = (s.m*vOrthS + c.m*vOrthC - res * c.m * (vOrthS-vOrthC)) / (s.m + c.m);
	vOrthC += F/c.m;

	//neuer Parallelteil der Winkelgeschwindigkeit
	omegaParC += (c.getInertia()+getSteinerSummand(rC, c.m)).inverse() * crossProduct(rC, F);
	omegaParS = .99f * (2.f/7* omegaParS - (5.f/7 / (s.r*s.r)) * rS.crossProduct(vParS));

	//neue Winkelgeschwindigkeit
	c.omega = .99f*(omegaParC + omegaOrthC);
	s.omega = omegaParS + res * omegaOrthS;

	//neue Parallelgeschwindigkeit abhängig von Reibung
	vParC = crossProduct(rC, omegaParC);
	vParS = crossProduct(rS, omegaParS);

	//neue Geschwindigkeit
	c.v = .99f*(vOrthC + vParC - vParC.getParallelPartToNormal(n));
	s.v = vOrthS + vParS;
	
}


inline void CollisionHandler::operator () (Sphere& s, Cuboid& c, const Vector3f& pt, const Vector3f& n) const {
	operator () (c, s, pt, n);
}


inline void CollisionHandler::operator () (Sphere& s1, Sphere& s2, const Vector3f& pt, const Vector3f& n) const {

	//Berührpunkt, wird so gewählt, dass s1 kraft autoritärer Willkür
	//passend liegt und s2 ggf. noch angepasst werden muss
	//Vector3f b = s1.x + s1.r * n;

	//Berührebene erzeugen
	Plane p(n, pt);

	//Position von s2 korrigieren
	correctPosition(s2, p);
	correctPosition(s1, p);

	//Teile der Geschwindigkeiten, die senkrecht zur Berührebene liegen
	Vector3f v1Orth = s1.v.getParallelPartToNormal(n);
	Vector3f v2Orth = s2.v.getParallelPartToNormal(n);

	//Parallelteile
	Vector3f v1Par = s1.v - v1Orth;
	Vector3f v2Par = s2.v - v2Orth;

	//Teile der Rotationsgeschwindigkeiten, die senkrecht zur Berührebene liegen
	Vector3f omega1Orth = s1.omega.getParallelPartToNormal(n);
	Vector3f omega2Orth = s2.omega.getParallelPartToNormal(n);

	//und die Parallelteile
	Vector3f omega1Par = s1.omega - omega1Orth;
	Vector3f omega2Par = s2.omega - omega2Orth;

	//Koeffizienten abhängig von den Materialien
	f32 res = resCoef[s1.k][s2.k];
	f32 mue = staticFricCoef[s1.k][s2.k];

	/*if((v1Orth-v2Orth).lengthSqr() < .25f)
		res = 0.01f;*/

	//Radien als Vektoren
	Vector3f r1 = pt - s1.x;
	Vector3f r2 = pt - s2.x;

	//neue Parallelteile der Rotationen
	omega1Par = .99f * (2.f/7* omega1Par - 5.f/7 / (s1.r*s1.r) * r1.crossProduct(v1Par));
	omega2Par = .99f * (2.f/7* omega2Par - 5.f/7 / (s2.r*s2.r) * r2.crossProduct(v2Par));

	//neue Winkelgeschwindigkeiten
	s1.omega = omega1Par + res * omega1Orth;
	s2.omega = omega2Par + res * omega2Orth;

	Vector3f tmpv = s1.m*v1Orth + s2.m*v2Orth;
	f32 m = s1.m + s2.m;

	//neue Geschwindigkeiten
	s1.v = (tmpv - res * s2.m * (v1Orth-v2Orth)) / m //neuer Orthogonalteil
		- (mue * omega1Par.crossProduct(r1) + (1-mue) * v1Par); //neuer Parallelteil
	s2.v = (tmpv - res * s1.m * (v2Orth-v1Orth)) / m //neuer Orthogonalteil
		- (mue * omega2Par.crossProduct(r2) + (1-mue) * v2Par); //neuer Parallelteil
}

inline void CollisionHandler::operator () (Sphere& s, const Plane& p, const Vector3f& pt) const {

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
	f32 mue = 1;//staticFricCoef[s.k][p.k];

	/*if(vOrth.lengthSqr() < .25f)
		res = 0.01f;*/
		
	//Kugel und Ebene sollen sich nicht schneiden
	//correctPosition(s, p);										<==== hier ist der Fehler!!!!!!!!!!!!!

	// Radius als Vektor
	Vector3f r = pt - s.x;

	//neuer Parallelteil der Rotation
	omegaPar = .99f * (2.f/7* omegaPar - 
		5.f/7 / (r*r) * r.crossProduct(vPar));

	//neue Winkelgeschwindigkeit
	s.omega = omegaPar + res * omegaOrth;

	//neue Geschwindigkeit
	s.v = -res*vOrth //neuer Orthogonalteil
		- (mue * omegaPar.crossProduct(r) + (1-mue) * vPar); //neuer Parallelteil
}

inline void CollisionHandler::operator () (const Plane& p, Sphere& s, const Vector3f& pt) const {
	operator () (s, p, pt);
}



inline void CollisionHandler::operator () (Cuboid& c, const FixCuboid& fc, const Vector3f& pt, const Vector3f& n) const {
	
	//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f vOrth = c.v.getParallelPartToNormal(n);

	//Parallelteil
	Vector3f vPar = c.v - vOrth;

	//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f omegaOrth = c.omega.getParallelPartToNormal(n);

	//und der parallele Teil
	Vector3f omegaPar = c.omega - omegaOrth;

	//Koeffizienten abhängig von den Materialien
	f32 res = resCoef[c.k][fc.k];
	f32 mue = staticFricCoef[c.k][fc.k];

	// Verbindungsvektor vom Schwerpunkt zum Kontaktpunkt
	Vector3f r = pt - c.x;
	//c.x -= .1f*r;
	//c2.x -= .01f*r2;

	//Zerlegung des Radius
	Vector3f rPar = r - r.getParallelPartToNormal(n);

	//Rotationsachse ist senkrecht zu r und der Normalen
	Vector3f rotN = crossProduct(r, n);

	//Trägheitsmoment bezüglich neuer Rotationsachse
	f32 theta = rotN ? rotN * ((c.getInertia()+getSteinerSummand(r, c.m)) * rotN) / rotN.lengthSqr() : 0;

	//Kraftstoß (Impulsänderung)
	Vector3f F = (-1-res) * (vOrth + crossProduct(c.omega, r).getParallelPartToNormal(n)) / 
		(1/c.m + (theta==0 ? 0 : rPar.lengthSqr()/theta));

	//neuer Orthogonalteil der Geschwindigkeit
	vOrth += F/c.m;

	//neuer Parallelteil der Winkelgeschwindigkeit
	omegaPar += (c.getInertia()+getSteinerSummand(r, c.m)).inverse() * crossProduct(r, F);

	//neue Winkelgeschwindigkeit
	c.omega = .99f*(omegaPar + omegaOrth);

	//abhängig von Reibung Geschwindigkeit aus Winkelgeschwindigkeit bestimmen
	vPar = (1-mue) * vPar + mue * crossProduct(r, omegaPar);

	//neue Geschwindigkeit
	c.v = .99f*(vOrth	+ vPar - vPar.getParallelPartToNormal(n));

}

inline void CollisionHandler::operator () (const FixCuboid& fc, Cuboid& c, const Vector3f& pt, const Vector3f& n) const {
	return operator () (c, fc, pt, n);
}


inline void CollisionHandler::operator () (Sphere& s, const FixCuboid& fc, const Vector3f& pt, const Vector3f& n) const {
	
	//Teil der Geschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f vOrth = s.v.getParallelPartToNormal(n);

	//Parallelteil
	Vector3f vPar = s.v - vOrth;

	//Teil der Rotationsgeschwindigkeit, der senkrecht zur Ebene liegt
	Vector3f omegaOrth = s.omega.getParallelPartToNormal(n);

	//und der parallele Teil
	Vector3f omegaPar = s.omega - omegaOrth;

	//Koeffizienten abhängig von den Materialien
	f32 res = resCoef[fc.k][s.k];
	f32 mue = staticFricCoef[fc.k][s.k];
		
	//Quader und Ebene sollen sich nicht schneiden
	Plane p(n, pt);
	correctPosition(s, p);

	// Radius als Vektor
	Vector3f r = pt - s.x;

	//neuer Parallelteil der Rotation
	omegaPar = .99f * (2.f/7* omegaPar - 
		5.f/7 / (r*r) * r.crossProduct(vPar));

	//neue Winkelgeschwindigkeit
	s.omega = omegaPar + res * omegaOrth;

	//neue Geschwindigkeit
	s.v = -res*vOrth //neuer Orthogonalteil
		- (mue * omegaPar.crossProduct(r) + (1-mue) * vPar); //neuer Parallelteil
}

inline void CollisionHandler::operator () (const FixCuboid& fc, Sphere& s, const Vector3f& pt, const Vector3f& n) const {
	return operator () (s, fc, pt, n);
}



inline Vector3f CollisionHandler::correctPosition(Sphere& s, const Plane& p) const {
	//Position der Kugel sollte auf jeden Fall vollständig "diesseits"
	//der Ebene bleiben. Dazu einfach die neue Position der Kugel 
	//korrigieren, indem senkrecht zur Ebene zurückgesetzt wird
	f32 d = p.orientatedDistanceTo(s.x);

	//in Abhängigkeit von der Seite, auf der sich der Mittelpunkt befindet,
	//entweder den Radius zum Abstand addieren oder subtrahieren
	if(d > 0.f){
		d -= s.r+.001f;
	}
	else if(d < 0.f){
		d += s.r+.001f;
	}
		
	s.x -= d*p.n;//Position korrigieren

	return -d*p.n;
}



inline Matrix3f CollisionHandler::getSteinerSummand(const Vector3f& a, f32 m) const {
	f32 ma0 = m*a[0];
	f32 ma1 = m*a[1];
	f32 ma2 = m*a[2];

	f32 ma00 = ma0*a[0];
	f32 ma11 = ma1*a[1];
	f32 ma22 = ma2*a[2];
	f32 ma01 = -ma0*a[1];
	f32 ma02 = -ma0*a[2];
	f32 ma12 = -ma1*a[2];

	return Matrix3f(ma11+ma22,	ma01,		ma02,
					ma01,		ma00+ma22,	ma12,
					ma02,		ma12,		ma00+ma11);
}