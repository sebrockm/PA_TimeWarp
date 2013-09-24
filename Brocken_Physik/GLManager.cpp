#include "GLManager.h"
#include "material.h"
#include "Geometry.h"

#include <sstream>
#include <fstream>
#include <vector>
#include <iostream>
#include <cstdlib>

#include <boost\gil\extension\io\jpeg_io.hpp>


using namespace std;


GLManager::GLManager(){}

GLManager& GLManager::instance(){
	static GLManager inst;
	return inst;
}

void GLManager::printError(const string& name) {
#ifdef PRINT_GL_ERROR
	string errorstring;
	GLenum err = glGetError();
	switch(err){
		case GL_NO_ERROR: return;
		case GL_INVALID_ENUM: errorstring = "GL_INVALID_ENUM"; break;
		case GL_INVALID_OPERATION: errorstring = "GL_INVALID_OPERATION"; break;
		case GL_INVALID_VALUE: errorstring = "GL_INVALID_VALUE"; break;
		case GL_OUT_OF_MEMORY: errorstring = "GL_OUT_OF_MEMORY"; break;
	}
	clog << errorstring << " in " << name << endl;
#endif
}

template <class Model>
void GLManager::add(const Model& geo){
	Geometry<Model>* mod = new Geometry<Model>();
	mod->model = &geo;
	drawables.push_back(mod);
}
template void GLManager::add(const Sphere& geo);
template void GLManager::add(const Plane& geo);
template void GLManager::add(const Cuboid& geo);
template void GLManager::add(const FixCuboid& geo);
template void GLManager::add(const KDTreeNode& geo);
template void GLManager::add(const Connector& geo);


void GLManager::enterMainLoop(){
	timer.start();
	glutMainLoop();
}

void GLManager::destruct(){
	glutSetKeyRepeat(GLUT_KEY_REPEAT_DEFAULT);
	for(auto i = drawables.begin(); i != drawables.end(); ++i){
		//(*i)->destruct();
		delete (*i);
	}
	
	Geometry<Sphere>::destruct();
	Geometry<Plane>::destruct();
	Geometry<Cuboid>::destruct();
	Geometry<FixCuboid>::destruct();
	Geometry<KDTreeNode>::destruct();
	Geometry<Connector>::destruct();

	glDeleteTextures(material_N, materialTex);
	delete [] materialTex;
}
	
void GLManager::render(){
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(instance().program);
	instance().printError("render()");
	//Kamera
	instance().cam.move(instance().camMovement[0], instance().camMovement[1], instance().camMovement[2]);
	Matrix4f vp = instance().cam.getViewProj();
	glUniformMatrix4fv(instance().viewProjLoc, 1, GL_TRUE, vp[0]);
		
	//alles zeichnen
	for(auto i = instance().drawables.begin(); i != instance().drawables.end(); ++i){
		(*i)->draw();
	}

	glutSwapBuffers();
	instance().printError("render()");
}

void GLManager::handleKeyUp(u8 key, int x, int y){
		
	switch(key){
	case 27: instance().destruct(); exit(0); break; //Escape
	case 'w': instance().camMovement[0] = 0; break;
	case 's': instance().camMovement[0] = 0; break;
	case 'a': instance().camMovement[1] = 0; break;
	case 'd': instance().camMovement[1] = 0; break;
	case ' ': instance().camMovement[2] = 0; break;
	case 'c': instance().camMovement[2] = 0; break;
	case 'p':	instance().pause ^= true; 
				if(!instance().pause) instance().timer.tick(); 
				break;
	case '+': instance().div /= 1.5f; break;
	case '-': instance().div *= 1.5f; break;
	case '0': instance().div = 1; break;
	case 'm': instance().takeScreen(); break;
	}
		
	glutPostRedisplay();
}

void GLManager::handleKeyDown(u8 key, int x, int y){
	const f32 speed = .2f;

	switch(key){
	case 'w': instance().camMovement[0] = speed; break;
	case 's': instance().camMovement[0] = -speed; break;
	case 'a': instance().camMovement[1] = speed; break;
	case 'd': instance().camMovement[1] = -speed; break;
	case ' ': instance().camMovement[2] = speed; break;
	case 'c': instance().camMovement[2] = -speed; break;
	}
	glutPostRedisplay();
}

void GLManager::timerFunc(){	
	static f32 t = 0;
	static Array<f32, 60> fps;
	static int i = 0;

	if(!instance().pause){
		f32 dt = instance().timer.tick();//.0167f;
		
		instance().cmgr.calculateTime(min(dt, 1.f/60), instance().div);
		fps[i++] = 1/dt;
		i %= fps.size();

		t += dt;
		if(t > 1.f/30){
			glutPostRedisplay();
			t = 0;

			f32 sum = 0;
			for(int j=0; j<fps.size(); j++){
				sum += fps[j];
			}
			instance().frames = static_cast<int>(sum/fps.size());
			std::cout<<fixed<<(sum/fps.size())<<"FPS\t\r";
		}
	}
}

void GLManager::handleMouseInput(int key, int state, int x, int y){
	if (key == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN){
			//Mausposition bei Klick speichern
			instance().mouseX = x;
			instance().mouseY = y;
		}
		else {
			//bei Loslassen zurücksetzen
			instance().mouseX = instance().mouseY = -1;
		}
	}
	else if(key == GLUT_RIGHT_BUTTON && state == GLUT_UP) {
		Sphere& s = instance().cmgr.spheres[instance().cmgr.addSphere()];
		s.x = instance().cam.pos+instance().cam.viewDir*2;
		s.v = instance().cam.viewDir*10;
		s.r = .4f;
		s.m = 10;
		s.k = (Material)(rand()%material_N);
		//createSphere(s);
	}
	glutPostRedisplay();
}

void GLManager::handleMouseMotion(int x, int y){
	if(instance().mouseX >= 0 && instance().mouseY >= 0){ //linke Maustaste gedrückt
		f32 dPhi = (f32)(instance().mouseX-x)/instance().viewPortX;
		f32 dTheta = (f32)(instance().mouseY-y)/instance().viewPortY;
		instance().mouseX = x;
		instance().mouseY = y;
		instance().cam.rotate(2*dPhi, 2*dTheta);
	}
	glutPostRedisplay();
}

void GLManager::handleChangeSize(int x, int y){
	if(x == 0) x = 1;
	if(y == 0) y = 1;

	instance().viewPortX = x;
	instance().viewPortY = y;

	glViewport(0, 0, x, y);
		
	instance().cam.changeViewport(x, y);

	glutPostRedisplay();
}

string GLManager::getFileContents(const string& file) {
	basic_ifstream<GLchar> in(file);
	if(!in){
		clog << "Kann " << file << " nicht öffnen!" << endl;
		return "";
	}

	basic_stringstream<GLchar> sstream;
	sstream << in.rdbuf();

	return sstream.str();
}

void GLManager::init(int& argc, char** argv, const char* title){
	viewPortX = 800;
	viewPortY = 800;
	cam.changeViewport(viewPortX, viewPortY);
	//cam.move(-10,0,5);
	mouseX = mouseY = -1;
	pause = true;
	div = 1;

	glutInit(&argc, argv);
		
	/*glutInitContextVersion (3, 3);
	glutInitContextFlags (GLUT_FORWARD_COMPATIBLE | GLUT_DEBUG);
	glutInitContextProfile(GLUT_CORE_PROFILE);*/
		
	glutInitWindowPosition(500, 100);
	glutInitWindowSize(viewPortX, viewPortY);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

	glutCreateWindow(title);
		
	glutDisplayFunc(render);
	glutIdleFunc(timerFunc);
	glutKeyboardFunc(handleKeyDown);
	glutKeyboardUpFunc(handleKeyUp);
	glutMouseFunc(handleMouseInput);
	glutMotionFunc(handleMouseMotion);
	glutReshapeFunc(handleChangeSize);

	//glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF);

	//glClearColor(1.f, .5f, .1f, 1.f);
	glClearColor(217/255.f, 204/255.f, 60/255.f, 1.f);
	
	glewExperimental = GL_TRUE;//glew bug
	GLuint a = glewInit();
	glGetError();//GL_INVALID_ENUM ignorieren
	printError("glewInit()");
	if(a != GLEW_OK){
		clog << glewGetErrorString(a) << endl;
	}
	if(!glewIsSupported("GL_VERSION_3_3")){
		clog << "OpenGL 3.3 not supported!" << endl;
	}
		
	glViewport(0, 0, viewPortX, viewPortY);

	glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glEnable(GL_DEPTH_TEST);
	glBlendEquation(GL_FUNC_ADD);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_COLOR);
	glDisable(GL_BLEND);
	glLineWidth(3);
		
	printError("init()");

}

void GLManager::createShaderProgram(const string& vs, const string& fs){
	program = glCreateProgram();
		
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		
	string vsFileContents = getFileContents(vs);
	string fsFileContents = getFileContents(fs);
		
	const GLchar* vsFC = vsFileContents.c_str();
	const GLchar* fsFC = fsFileContents.c_str();
		
	int vsLength = static_cast<int>(vsFileContents.size());
	int fsLength = static_cast<int>(fsFileContents.size());

	glShaderSource(vertexShader, 1, &vsFC, &vsLength);
	glShaderSource(fragmentShader, 1, &fsFC, &fsLength);
		
	glCompileShader(vertexShader);
	glCompileShader(fragmentShader);
		
	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);
		
	glBindFragDataLocation(program, 0, "color");

	glBindAttribLocation(program, ATTRIB_POS, "positionMC");
	glBindAttribLocation(program, ATTRIB_NOR, "normalMC");
	glBindAttribLocation(program, ATTRIB_TEX_POS, "texCoords");

	glLinkProgram(program);
	printError("createShaderProgram()");
	GLchar buffer[1000];
	int length;
	glGetProgramInfoLog(program, sizeof(buffer)/sizeof(GLchar)-1, &length, buffer);
	buffer[length] = '\0';
	clog << buffer << endl;

	mod2worldLoc = glGetUniformLocation(program, "model2world");
	viewProjLoc = glGetUniformLocation(program, "viewProj");
	materialTexLoc = glGetUniformLocation(program, "materialTex");
		
	printError("createShaderProgram()");

	
	materialTex = new GLuint[material_N];
	glGenTextures(material_N, materialTex);

	for(int i = 0; i<material_N; i++){
		glActiveTexture(GL_TEXTURE0+i);
		glBindTexture(GL_TEXTURE_2D, materialTex[i]);

		printError("tex-for");
		
		string file;
		switch(i){
		case steel:	file = "steel.jpg";	break;
		case wood:	file = "wood.jpg";	break;
		case glass:	file = "glass.jpg";	break;
		case rubber:file = "rubber.jpg";break;
		}

		using namespace boost::gil;

		rgb8_image_t im;
		jpeg_read_image(file, im);
		
		glTexImage2D(
			GL_TEXTURE_2D,
			0,
			GL_RGB8,
			(GLsizei)im.width(),
			(GLsizei)im.height(),
			0,
			GL_RGB,
			GL_UNSIGNED_BYTE,
			&(view(im)[0][0])
		);
		glGenerateMipmap(GL_TEXTURE_2D);

		printError("glTexImage2d");
		/*glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);*/
	}
	
	
	Geometry<Sphere>::create();
	Geometry<Plane>::create();
	Geometry<Cuboid>::create();
	Geometry<FixCuboid>::create();
	Geometry<KDTreeNode>::create();
	Geometry<Connector>::create();
}

void GLManager::takeScreen(){
	using namespace boost::gil;
	rgb8_image_t im(viewPortX, viewPortY);
	glReadPixels(0,0,viewPortX,viewPortY, GL_RGB, GL_UNSIGNED_BYTE, &(view(im)[0][0]));

	stringstream ss;
	ss << "C:\\Users\\Poppes\\Dropbox\\hgb-thesis-20121208\\images\\screens\\";
	ss << timer.getLast() << " " << frames << "FPS.jpg";
	jpeg_write_view(ss.str(), flipped_up_down_view(view((im))));
}
