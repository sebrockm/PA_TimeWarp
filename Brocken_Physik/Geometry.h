#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include "opengl.h"
#include "GLManager.h"

#include <vector>


using namespace std;


class IGeometry{
public:

	/*virtual void setIndices(const vector<GLuint>& indices) = 0;

	virtual void setVertices(const vector<GLfloat>& vertices) = 0;

	virtual void construct() = 0;

	virtual void destruct() = 0;*/
	
	virtual void draw() const = 0;
};



template <class Model>
class Geometry : public IGeometry{
private:
	static GLuint vaid; //Vertexarray id

	static vector<GLManager::Vertex> vertexData;
	static vector<GLuint> indexData;

	static GLuint indexBufferSize;
	
	static GLuint bufferIds[2]; 

	static bool constructed;


public:
	const Model* model;

public:
	Geometry(const Model* m = 0);

	virtual ~Geometry();
/*
	static void setIndices(const vector<GLuint>& indices);

	static void setVertices(const vector<GLfloat>& vertices);*/

	static void construct();

	static void destruct();

	static void create();
	
	virtual void draw() const;

};


#endif