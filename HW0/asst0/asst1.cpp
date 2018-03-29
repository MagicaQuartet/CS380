////////////////////////////////////////////////////////////////////////
//
//   Harvard Computer Science
//   CS 175: Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "glsupport.h"

using namespace std;      // for string, vector, iostream and other standard C++ stuff
using namespace std::tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// !!!!!!!! IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!
// Before you start working on this assignment, set the following variable properly
// to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or OpenGL 3.x+ with
// GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to use GLSL 1.3.
// Make sure that your machine supports the version of GLSL you are using. In particular,
// on Mac OS X currently there is no way of using OpenGL 3.x with GLSL 1.3 when
// GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get loaded
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
static const bool g_Gl2Compatible = true;


static int g_width             = 512;       // screen width
static int g_height            = 512;       // screen height

/* for asst0, you can use these global variables */
static float g_vertexTemp      = 0.0;       // for vertex moving
static float g_colorTemp	   = 0.0;			// for changing color 

struct ShaderState {
  GlProgram program;

  // Handles to uniform variables
  GLfloat h_uVertexTemp;
  GLfloat h_uColorTemp;
  GLfloat h_uTime;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aColor;

  ShaderState(const char* vsfn, const char* fsfn) {
    readAndCompileShader(program, vsfn, fsfn);

    const GLuint h = program; // short hand

    // Retrieve handles to uniform variables
    h_uVertexTemp = safe_glGetUniformLocation(h, "uVertexTemp");
	h_uColorTemp = safe_glGetUniformLocation(h, "uColorTemp");
	h_uTime = safe_glGetUniformLocation(h, "uTime");

    // Retrieve handles to vertex attributes
    h_aPosition = safe_glGetAttribLocation(h, "aPosition");
    h_aColor = safe_glGetAttribLocation(h, "aColor");

    if (!g_Gl2Compatible)
      glBindFragDataLocation(h, 0, "fragColor");
    checkGlErrors();
  }
};

static const int g_numShaders = 1;
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/asst1-gl3.vshader", "./shaders/asst1-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/asst1-gl2.vshader", "./shaders/asst1-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

struct SquareGeometry {
  GlBufferObject posVbo, colVbo;

  SquareGeometry() {
    static GLfloat sqPos[6 * 2] = {
		/* for asst0 */
		-0.5, -0.5,
		0.0, -1.0,
		0.5, -0.5,
    };

    static GLfloat sqCol[6 * 3] =  {
		/* for asst0 */
		1,0,0,
		1,0,0,
		1,0,0
    };

    glBindBuffer(GL_ARRAY_BUFFER, posVbo);
    glBufferData(
      GL_ARRAY_BUFFER,
      12*sizeof(GLfloat),
      sqPos,
      GL_STATIC_DRAW);
    checkGlErrors();


    glBindBuffer(GL_ARRAY_BUFFER, colVbo);
    glBufferData(
      GL_ARRAY_BUFFER,
      18*sizeof(GLfloat),
      sqCol,
      GL_STATIC_DRAW);
    checkGlErrors();
  }

  void draw(const ShaderState& curSS) {
    int numverts=6;
    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aColor);

    glBindBuffer(GL_ARRAY_BUFFER, posVbo);
    safe_glVertexAttribPointer(curSS.h_aPosition,
                               2, GL_FLOAT, GL_FALSE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, colVbo);
    safe_glVertexAttribPointer(curSS.h_aColor,
                               3, GL_FLOAT, GL_FALSE, 0, 0);

    glDrawArrays(GL_TRIANGLES,0,numverts);

    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aColor);
  }
};

static shared_ptr<SquareGeometry> g_square; // our global geometries

// C A L L B A C K S ///////////////////////////////////////////////////


// _____________________________________________________
//|                                                     |
//|  display                                            |
//|_____________________________________________________|
///
///  Whenever OpenGL requires a screen refresh
///  it will call display() to draw the scene.
///  We specify that this is the correct function
///  to call with the glutDisplayFunc() function
///  during initialization

static void display(void){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const ShaderState& curSS = *g_shaderStates[0];
  glUseProgram(curSS.program);

  float timeMilliseconds = glutGet(GLUT_ELAPSED_TIME);

  safe_glUniform1f(curSS.h_uVertexTemp, g_vertexTemp);
  safe_glUniform1f(curSS.h_uColorTemp, g_colorTemp);
  safe_glUniform1f(curSS.h_uTime, timeMilliseconds);
  g_square->draw(curSS);

  glutSwapBuffers();

  // check for errors
  checkGlErrors();

  glutPostRedisplay();
}

// _____________________________________________________
//|                                                     |
//|  reshape                                            |
//|_____________________________________________________|
///
///  Whenever a window is resized, a "resize" event is
///  generated and glut is told to call this reshape
///  callback function to handle it appropriately.


static void reshape(int w, int h){
	g_width = w;
	g_height = h;
	glViewport(0, 0, w, h);
	glutPostRedisplay();
}

static void keyboard(unsigned char key, int x, int y){
  /* for asst0 */
  switch (key) {
  case 'h':
	  cout << "Welcome to OpenGL World!\n" << "Please enjoy your self!\n\n";
	  break;
  case 'r':
	  g_colorTemp = 1.0;
	  break;
  case 'g':
	  g_colorTemp = 2.0;
	  break;
  case 'b':
	  g_colorTemp = 3.0;
	  break;
  case 'c':
	  g_colorTemp = 4.0;
	  break;
  case 'p':
	  g_colorTemp = 0.0;
	  break;
  }
  glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y) { 
  /* for asst0 */
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			g_vertexTemp -= 0.1;
		}
	}
	else if (button == GLUT_RIGHT_BUTTON) {
		if (state == GLUT_DOWN) {
			g_vertexTemp += 0.1;
		}
	}
  glutPostRedisplay();
}

// H E L P E R    F U N C T I O N S ////////////////////////////////////

static void initGlutState(int argc, char **argv){
  glutInit(&argc,argv);
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);
  glutInitWindowSize(g_width, g_height);			// create a window
  glutCreateWindow("CS 380 : Hello World!!");		// title the window

  // callback functions
  glutDisplayFunc(display);							// display rendering callback
  glutReshapeFunc(reshape);							// window reshape callback
  glutMouseFunc(mouse);								// mouse click callback
  glutKeyboardFunc(keyboard);						// keyboard callback
}

static void initGLState() {
  glClearColor(128./255,200./255,1,0);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  g_square.reset(new SquareGeometry());
}


// M A I N /////////////////////////////////////////////////////////////

// _____________________________________________________
//|                                                     |
//|  main                                               |
//|_____________________________________________________|
///
///  The main entry-point for the HelloWorld example
///  application.


int main(int argc, char **argv) {
  try {
    initGlutState(argc,argv);
    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();
    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}

