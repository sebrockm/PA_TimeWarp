#include "Geometry.h"
#include "GLManager.h"

#include "Sphere.h"
#include "Plane.h"
#include "Cuboid.h"
#include "RigidBody.h"
#include "Connector.h"
#include "FixCuboid.h"
#include "KDTree.h"



template <class Model>
GLuint Geometry<Model>::vaid = -1;

template <class Model>
vector<GLManager::Vertex> Geometry<Model>::vertexData = vector<GLManager::Vertex>();

template <class Model>
vector<GLuint> Geometry<Model>::indexData = vector<GLuint>();

template <class Model>
GLuint Geometry<Model>::indexBufferSize = -1;

template <class Model>
GLuint Geometry<Model>::bufferIds[] = {-1,-1};

template <class Model>
bool Geometry<Model>::constructed = false;




template <class Model>
Geometry<Model>::Geometry(const Model* m):model(m){}

template <class Model>
Geometry<Model>::~Geometry(){
	//destruct();
}
//
//template <class Model>
//void Geometry<Model>::setIndices(const vector<GLuint>& indices){
//	indexData = indices;
//}
//
//template <class Model>
//void Geometry<Model>::setVertices(const vector<GLfloat>& vertices){
//	vertexData = vertices;
//}

template <class Model>
void Geometry<Model>::construct(){
	if(!constructed){
		glGenVertexArrays(1, &vaid);
		glBindVertexArray(vaid);

		glGenBuffers(2, bufferIds);
			
		glBindBuffer(GL_ARRAY_BUFFER, bufferIds[0]);
		glBufferData(GL_ARRAY_BUFFER, vertexData.size()*sizeof(GLManager::Vertex), &vertexData[0] , GL_STATIC_DRAW);

		glEnableVertexAttribArray(ATTRIB_POS);
		glVertexAttribPointer(ATTRIB_POS, 3, GL_FLOAT, GL_FALSE, sizeof(GLManager::Vertex), (void*)0);
		glEnableVertexAttribArray(ATTRIB_NOR);
		glVertexAttribPointer(ATTRIB_NOR, 3, GL_FLOAT, GL_FALSE, sizeof(GLManager::Vertex), (void*)(sizeof(Vector3f)));
		glEnableVertexAttribArray(ATTRIB_TEX_POS);
		glVertexAttribPointer(ATTRIB_TEX_POS, 2, GL_FLOAT, GL_FALSE, sizeof(GLManager::Vertex), (void*)(2*sizeof(Vector3f)));

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufferIds[1]);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexData.size()*sizeof(GLuint), &indexData[0], GL_STATIC_DRAW);

		glBindVertexArray(0);

		indexBufferSize = static_cast<GLuint>(indexData.size());
		//RAM entlasten
		vertexData = vector<GLManager::Vertex>();
		indexData = vector<GLuint>();

		constructed = true;
	}
}

template <class Model>
void Geometry<Model>::destruct(){
	if(constructed){
		glDeleteBuffers(2, bufferIds);
		glDeleteVertexArrays(1, &vaid);
		constructed = false;
	}
}
	
template <class Model>
void Geometry<Model>::draw() const {
	if(model->k == glass){
		glEnable(GL_BLEND);
	}

	Matrix4f m2w = model->getModel2World();
	glUniformMatrix4fv(GLManager::instance().mod2worldLoc, 1, GL_TRUE, m2w[0]);
	glUniform1i(GLManager::instance().materialTexLoc, model->k);
		
	glBindVertexArray(vaid);
	glDrawElements(GL_TRIANGLE_STRIP, indexBufferSize, GL_UNSIGNED_INT, (void*)0);

	glDisable(GL_BLEND);
}


template <>
void Geometry<Plane>::draw() const {
	if(model->k == glass){
		glEnable(GL_BLEND);
	}
	glDisable(GL_CULL_FACE);

	Matrix4f m2w = model->getModel2World();
	glUniformMatrix4fv(GLManager::instance().mod2worldLoc, 1, GL_TRUE, m2w[0]);
	glUniform1i(GLManager::instance().materialTexLoc, model->k);
		
	glBindVertexArray(vaid);
	glDrawElements(GL_TRIANGLE_STRIP, indexBufferSize, GL_UNSIGNED_INT, (void*)0);

	glEnable(GL_CULL_FACE);
	glDisable(GL_BLEND);
}


template <>
void Geometry<KDTreeNode>::draw() const {
#ifdef DRAW_KDTREE

	Vector3f x = (model->aabb2 + model->aabb1)/2;
	//x[model->axNo] = model->axis;
	Matrix4f m2w = createTranslationMatrix(x);

	Vector3f dim = model->aabb2 - model->aabb1;
	m2w *= createScalarMatrix(dim[0],dim[1],dim[2]);

	glUniformMatrix4fv(GLManager::instance().mod2worldLoc, 1, GL_TRUE, m2w[0]);
	glUniform1i(GLManager::instance().materialTexLoc, rubber);
	glBindVertexArray(vaid);
	glDrawElements(GL_LINE_STRIP, indexBufferSize, GL_UNSIGNED_INT, (void*)0);

	//rekursiv die Söhne zeichnen
	if(!model->isLeaf()){
		auto& tree = GLManager::instance().cmgr.treeArray;

		Geometry<KDTreeNode> next(&tree[model->left]);
		next.draw();

		next.model = &tree[model->right];
		next.draw();
	}
#endif
}


template <>
void Geometry<Connector>::draw() const {
	Vector3f w1, w2;
	auto& cmgr = GLManager::instance().cmgr;

	if(model->id1 >= cmgr.getOffset<Vector3f>()){
		w1 = model->p1;
	}
	else if(model->id1 >= cmgr.getOffset<FixCuboid>()){
		u32 id = model->id1 - cmgr.getOffset<FixCuboid>();
		w1 = cmgr.fixCuboids[id].getModel2World() * model->p1;
	}
	else if(model->id1 >= cmgr.getOffset<Cuboid>()){
		u32 id = model->id1 - cmgr.getOffset<Cuboid>();
		w1 = cmgr.cuboids[id].getModel2World() * model->p1;
	}
	else if(model->id1 >= cmgr.getOffset<Sphere>()){
		u32 id = model->id1 - cmgr.getOffset<Sphere>();
		w1 = cmgr.spheres[id].getModel2World() * model->p1;
	}

	if(model->id2 >= cmgr.getOffset<Vector3f>()){
		w2 = model->p2;
	}
	else if(model->id2 >= cmgr.getOffset<FixCuboid>()){
		u32 id = model->id2 - cmgr.getOffset<FixCuboid>();
		w2 = cmgr.fixCuboids[id].getModel2World() * model->p2;
	}
	else if(model->id2 >= cmgr.getOffset<Cuboid>()){
		u32 id = model->id2 - cmgr.getOffset<Cuboid>();
		w2 = cmgr.cuboids[id].getModel2World() * model->p2;
	}
	else if(model->id2 >= cmgr.getOffset<Sphere>()){
		u32 id = model->id2 - cmgr.getOffset<Sphere>();
		w2 = cmgr.spheres[id].getModel2World() * model->p2;
	}

	Vector3f target = w2 - w1;
	Vector3f n = crossProduct(Vector3f(1,0,0), target).getNormalized();
	f32 cosphi = target.getNormalized()[0];

	Matrix4f m2w = createTranslationMatrix(w1);

	m2w *= createRotationQuaternion(acos(cosphi), n).getMatrix4();

	m2w *= createScalarMatrix(target.length(), target.length(), target.length());

	glUniformMatrix4fv(GLManager::instance().mod2worldLoc, 1, GL_TRUE, m2w[0]);
	glUniform1i(GLManager::instance().materialTexLoc, rubber);
	glBindVertexArray(vaid);
	glDrawElements(GL_LINE_STRIP, indexBufferSize, GL_UNSIGNED_INT, (void*)0);
}


template <>
void Geometry<Connector>::create() {
	GLManager::Vertex vertex1, vertex2;
	vertex1.texPos = vertex2.texPos = Vector<f32, 2>(0, 0);
	vertex1.normal = vertex2.normal = Vector3f(0,1,0);
	vertex1.pos = Vector3f(0,0,0);
	vertex2.pos = Vector3f(1,0,0);

	vertexData.push_back(vertex1);
	vertexData.push_back(vertex2);
	indexData.push_back(0);
	indexData.push_back(1);
	indexData.push_back(RESTART_INDEX);

	construct();
}


template <>
void Geometry<KDTreeNode>::create() {
	GLManager::Vertex vertex;
	vertex.texPos = Vector<f32, 2>(0, 0);

	glPrimitiveRestartIndex(RESTART_INDEX);
    glEnable(GL_PRIMITIVE_RESTART);

	//0
	vertex.pos = Vector3f(-.5f, -.5f, .5f);
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);

	//1
	vertex.pos[0] = .5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);
	
	//2
	vertex.pos[2] = -.5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);
	
	//3
	vertex.pos[0] = -.5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);

	//4
	vertex.pos = Vector3f(-.5f, .5f, .5f);
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);

	//5
	vertex.pos[0] = .5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);
	
	//6
	vertex.pos[2] = -.5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);
	
	//7
	vertex.pos[0] = -.5f;
	vertex.normal = vertex.pos.getNormalized();
	vertexData.push_back(vertex);

	//Unterseite
	indexData.push_back(0);
	indexData.push_back(1);
	indexData.push_back(2);
	indexData.push_back(3);
	indexData.push_back(0);
	indexData.push_back(RESTART_INDEX);

	//Vorderseite
	indexData.push_back(1);
	indexData.push_back(5);
	indexData.push_back(4);
	indexData.push_back(0);
	indexData.push_back(RESTART_INDEX);

	//Oberseite
	indexData.push_back(5);
	indexData.push_back(6);
	indexData.push_back(7);
	indexData.push_back(4);
	indexData.push_back(RESTART_INDEX);

	//Rückseite
	indexData.push_back(2);
	indexData.push_back(6);
	indexData.push_back(RESTART_INDEX);	
	indexData.push_back(3);
	indexData.push_back(7);
	indexData.push_back(RESTART_INDEX);

	construct();
}


template <>
void Geometry<Plane>::create() {
	GLManager::Vertex vertex;

	//vorne links
	vertex.pos = Vector3f(-.5f, 0, .5f);
	vertex.normal = Vector3f(0, 1, 0);
	vertex.texPos = Vector<f32, 2>(0,0);
	vertexData.push_back(vertex);

	//vorne rechts
	vertex.pos[0] = .5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);
	
	//hinten rechts
	vertex.pos[2] = -.5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);

	//hinten links
	vertex.pos[0] = -.5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(0);
	indexData.push_back(3);
	indexData.push_back(1);
	indexData.push_back(2);

	construct();
}


template <>
void Geometry<Sphere>::create() {
	const int n = 30;
	const int k = 30;
	GLManager::Vertex vertex;
	vertexData.reserve((n+1)*(k+1));
        
    f32 dTheta = PI/k;
    f32 dPhi = 2*PI/n;
    f32 theta = 0;
    for(int j=0; j <= k; ++j) {
        f32 sinTheta = sin(theta);
        f32 cosTheta = cos(theta);
        f32 phi = 0;
        for(int i=0; i <= n; ++i) {
            f32 sinPhi = sin(phi);
            f32 cosPhi = cos(phi);

			vertex.pos = vertex.normal = 
				Vector3f(sinTheta*cosPhi, cosTheta, sinTheta*sinPhi);
			vertex.texPos = Vector<f32, 2>((f32)i/n, (f32)j/k);

            vertexData.push_back(vertex);
                
            phi += dPhi;
        }
        theta += dTheta;
    }

	glPrimitiveRestartIndex(RESTART_INDEX);
    glEnable(GL_PRIMITIVE_RESTART);

	indexData.reserve(k*(2*(n+1)+1));
    for(int j=0; j < k; ++j) {
        for(int i=0; i <= n; ++i) {
            indexData.push_back((j+1)*(n+1) + i);
            indexData.push_back(j*(n+1) + i);
        }
        indexData.push_back(RESTART_INDEX);
    }
    
	construct();
}


//für (Fix)Cuboid
template <class Model>
void Geometry<Model>::create(){
	vertexData.reserve(27);
	indexData.reserve(30);

	GLManager::Vertex vertex;


	glPrimitiveRestartIndex(RESTART_INDEX);
    glEnable(GL_PRIMITIVE_RESTART);

	//Unterseite
	vertex.pos = Vector3f(-.5f, -.5f, .5f);
	vertex.normal = Vector3f(0, -1, 0);
	vertex.texPos = Vector<f32, 2>(0, 0);
	vertexData.push_back(vertex);

	vertex.pos[0] = .5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);
	
	vertex.pos[2] = -.5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);
		
	vertex.pos[0] = -.5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(0);
	indexData.push_back(3);
	indexData.push_back(1);
	indexData.push_back(2);
	indexData.push_back(RESTART_INDEX);

	
	//Vorderseite
	vertex.pos = Vector3f(-.5f, -.5f, .5f);
	vertex.normal = Vector3f(0, 0, 1);
	vertex.texPos = Vector<f32, 2>(0, 0);
	vertexData.push_back(vertex);

	vertex.pos[0] = .5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);
	
	vertex.pos[1] = .5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);
		
	vertex.pos[0] = -.5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(5);
	indexData.push_back(6);
	indexData.push_back(4);
	indexData.push_back(7);
	indexData.push_back(RESTART_INDEX);
	
	
	//linke Seite
	vertex.pos = Vector3f(-.5f, -.5f, .5f);
	vertex.normal = Vector3f(-1, 0, 0);
	vertex.texPos = Vector<f32, 2>(0, 1);
	vertexData.push_back(vertex);

	vertex.pos[1] = .5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);	

	vertex.pos[2] = -.5f;
	vertex.texPos[1] = 0;
	vertexData.push_back(vertex);
		
	vertex.pos[1] = -.5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(8);
	indexData.push_back(9);
	indexData.push_back(11);
	indexData.push_back(10);
	indexData.push_back(RESTART_INDEX);
		
	
	//rechte Seite
	vertex.pos = Vector3f(.5f, -.5f, .5f);
	vertex.normal = Vector3f(1, 0, 0);
	vertex.texPos = Vector<f32, 2>(0, 0);
	vertexData.push_back(vertex);

	vertex.pos[2] = -.5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);

	vertex.pos[1] = .5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);
		
	vertex.pos[2] = .5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(12);
	indexData.push_back(13);
	indexData.push_back(15);
	indexData.push_back(14);
	indexData.push_back(RESTART_INDEX);
			
	
	//Rückseite
	vertex.pos = Vector3f(.5f, -.5f, -.5f);
	vertex.normal = Vector3f(0, 0, -1);
	vertex.texPos = Vector<f32, 2>(0, 0);
	vertexData.push_back(vertex);

	vertex.pos[0] = -.5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);	

	vertex.pos[1] = .5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);
		
	vertex.pos[0] = .5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(16);
	indexData.push_back(17);
	indexData.push_back(19);
	indexData.push_back(18);
	indexData.push_back(RESTART_INDEX);

					
	//Oberseite
	vertex.pos = Vector3f(-.5f, .5f, .5f);
	vertex.normal = Vector3f(0, 1, 0);
	vertex.texPos = Vector<f32, 2>(0, 0);
	vertexData.push_back(vertex);

	vertex.pos[0] = .5f;
	vertex.texPos[0] = 1;
	vertexData.push_back(vertex);

	vertex.pos[2] = -.5f;
	vertex.texPos[1] = 1;
	vertexData.push_back(vertex);
		
	vertex.pos[0] = -.5f;
	vertex.texPos[0] = 0;
	vertexData.push_back(vertex);

	indexData.push_back(21);
	indexData.push_back(22);
	indexData.push_back(20);
	indexData.push_back(23);
	indexData.push_back(RESTART_INDEX);
	
	construct();
}




template class Geometry<Sphere>;
template class Geometry<Plane>;
template class Geometry<RigidBody>;
template class Geometry<Cuboid>;
template class Geometry<FixCuboid>;
//template class Geometry<SpherePointConnector>;
template class Geometry<KDTreeNode>;
template class Geometry<Connector>;