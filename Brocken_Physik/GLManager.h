#ifndef _GLMANAGER_H_
#define _GLMANAGER_H_

//#define PRINT_GL_ERROR

#include <string>
#include <vector>

#include "opengl.h"
#include "types.h"
#include "Camera.h"
//#include "Geometry.h"
#include "Timer.h"
#include "CollisionManager.h"

using namespace std;


class IGeometry;


class GLManager{
private:
	GLManager();
	GLManager(const GLManager&);
	GLManager& operator = (const GLManager&);

public:

	struct Vertex{
		Vector3f pos;
		Vector3f normal;
		Vector<f32, 2> texPos;
	};

	GLuint program, vertexShader, fragmentShader;

	GLint mod2worldLoc, viewProjLoc, materialTexLoc;

	GLuint* materialTex;

	vector<IGeometry*> drawables;

	Camera cam;
	Vector3f camMovement;

	int mouseX, mouseY;
	int viewPortX, viewPortY;

	bool pause;

	Timer timer;
	CollisionManager cmgr;

	int frames;

	f32 div;
	
	static GLManager& instance();


	void printError(const string& name);

	template <class Model>
	void add(const Model& geo);

	void enterMainLoop();

	void destruct();

	void takeScreen();
	
	static void render();

	static void handleKeyUp(u8 key, int x, int y);

	static void handleKeyDown(u8 key, int x, int y);

	static void timerFunc();

	static void handleMouseInput(int key, int state, int x, int y);

	static void handleMouseMotion(int x, int y);

	static void handleChangeSize(int x, int y);

	string getFileContents(const string& file);

	void init(int& argc, char** argv, const char* title = "Game");

	void createShaderProgram(const string& vs, const string& fs);
};

#endif