////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <list>
#include <string>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <fstream>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "rigtform.h"
#include "arcball.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "sgutils.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
static int g_objectView = 0;
static int g_skyMode = 0;
static bool g_isPicking = false;

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3; // 3 shaders instead of 2
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
	Cvec3f p, n;

	VertexPN() {}
	VertexPN(float x, float y, float z,
		float nx, float ny, float nz)
		: p(x, y, z), n(nx, ny, nz)
	{}

	// Define copy constructor and assignment operator from GenericVertex so we can
	// use make* functions from geometrymaker.h
	VertexPN(const GenericVertex& v) {
		*this = v;
	}

	VertexPN& operator = (const GenericVertex& v) {
		p = v.pos;
		n = v.normal;
		return *this;
	}
};

struct Geometry {
	GlBufferObject vbo, ibo;
	int vboLen, iboLen;

	Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
		this->vboLen = vboLen;
		this->iboLen = iboLen;

		// Now create the VBO and IBO
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
	}

	void draw(const ShaderState& curSS) {
		// Enable the attributes used by our shader
		safe_glEnableVertexAttribArray(curSS.h_aPosition);
		safe_glEnableVertexAttribArray(curSS.h_aNormal);

		// bind vbo
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
		safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

		// bind ibo
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

		// draw!
		glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

		// Disable the attributes used by our shader
		safe_glDisableVertexAttribArray(curSS.h_aPosition);
		safe_glDisableVertexAttribArray(curSS.h_aNormal);
	}
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_bodySphere;
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static Cvec3f g_objectColors[3] = { Cvec3f(1, 0, 0), Cvec3f(0, 0, 1), Cvec3f(0, 1, 1) };
static RigTForm g_viewpointRbt;
static RigTForm g_sphereRbt;
static double g_arcballScale;
static double g_arcballScreenRadius = 0.25 * min(g_windowHeight, g_windowWidth);

// keyframe

static vector<shared_ptr<SgRbtNode>> g_allNodes;
static list<vector<RigTForm>> g_keyFrameList;
static list<vector<RigTForm>>::iterator g_currentKeyFrameIterator;

// animation
static int g_KeyFrameIndex = -1;
static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback
static bool g_playingAnimation = false;
static bool g_stopAnimation = false;



///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
	// A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
	VertexPN vtx[4] = {
	  VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
	  VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
	  VertexPN(g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
	  VertexPN(g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
	};
	unsigned short idx[] = { 0, 1, 2, 0, 2, 3 };
	g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
	int ibLen, vbLen;
	getCubeVbIbLen(vbLen, ibLen);

	// Temporary storage for cube geometry
	vector<VertexPN> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeCube(1, vtx.begin(), idx.begin());
	g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(20, 20, vbLen, ibLen);

	vector<VertexPN> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeSphere(g_arcballScreenRadius, 20, 20, vtx.begin(), idx.begin());
	g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initBodySphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(40, 40, vbLen, ibLen);

	vector<VertexPN> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeSphere(1, 40, 40, vtx.begin(), idx.begin());
	g_bodySphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
	GLfloat glmatrix[16];
	projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
	safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
	if (g_windowWidth >= g_windowHeight)
		g_frustFovY = g_frustMinFov;
	else {
		const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
		g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
	}
}

static Matrix4 makeProjectionMatrix() {
	return Matrix4::makeProjection(
		g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
		g_frustNear, g_frustFar);
}

static void drawStuff(const ShaderState& curSS, bool picking) {

	// build & send proj. matrix to vshader
	const Matrix4 projmat = makeProjectionMatrix();
	sendProjectionMatrix(curSS, projmat);

	// use the skyRbt as the eyeRbt
	const RigTForm eyeRbt = g_viewpointRbt;
	const RigTForm invEyeRbt = inv(eyeRbt);

	const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
	const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
	safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
	safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

	if (!picking) {
		Drawer drawer(invEyeRbt, curSS);
		g_world->accept(drawer);

		// draw arcball as part of asst3
		if (((g_objectView == 0 && g_skyMode == 0) || g_currentPickedRbtNode != NULL) && !((g_objectView == 1 && g_currentPickedRbtNode == g_robot1Node) || (g_objectView == 2 && g_currentPickedRbtNode == g_robot2Node))) {
			if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
				g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight);
			Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_sphereRbt) * Matrix4::makeScale(Cvec3(g_arcballScale, g_arcballScale, g_arcballScale));
			Matrix4 NMVM = normalMatrix(MVM);
			sendModelViewNormalMatrix(curSS, MVM, NMVM);
			safe_glUniform3f(curSS.h_uColor, g_objectColors[2][0], g_objectColors[2][1], g_objectColors[2][2]);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			g_sphere->draw(curSS);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}
	else {
		Picker picker(invEyeRbt, curSS);
		g_world->accept(picker);
		glFlush();
		g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);

		if (g_currentPickedRbtNode == g_groundNode)
			g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
	}
}

static void pick() {
	// We need to set the clear color to black, for pick rendering.
	// so let's save the clear color
	GLdouble clearColor[4];
	glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);
	glClearColor(0, 0, 0, 0);

	// using PICKING_SHADER as the shader
	glUseProgram(g_shaderStates[PICKING_SHADER]->program);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawStuff(*g_shaderStates[PICKING_SHADER], true);
	// Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
	// to see result of the pick rendering pass
	// glutSwapBuffers();

	//Now set back the clear color
	glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
	checkGlErrors();
}

static void display() {
	glUseProgram(g_shaderStates[g_activeShader]->program);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

	drawStuff(*g_shaderStates[g_activeShader], false);

	glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

	checkGlErrors();
}

static void reshape(const int w, const int h) {
	g_windowWidth = w;
	g_windowHeight = h;
	glViewport(0, 0, w, h);
	cerr << "Size of window is now " << w << "x" << h << endl;
	g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);
	initSphere();
	updateFrustFovY();
	glutPostRedisplay();
}

static void motion(const int x, const int y) {
	const double dx = x - g_mouseClickX;
	const double dy = g_windowHeight - y - 1 - g_mouseClickY;

	RigTForm m;
	if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
		Quat src, dest;
		if (((g_objectView == 0 && g_skyMode == 0) || g_currentPickedRbtNode != NULL) && !((g_objectView == 1 && g_currentPickedRbtNode == g_robot1Node) || (g_objectView == 2 && g_currentPickedRbtNode == g_robot2Node))) {
			Cvec2 sphereCenterScreenSpaceCoord = getScreenSpaceCoord((inv(g_viewpointRbt) * g_sphereRbt).getTranslation(), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
			Cvec3 temp;
			double mouseClickZ = pow(g_arcballScreenRadius, 2) - pow(g_mouseClickX - sphereCenterScreenSpaceCoord[0], 2) - pow(g_mouseClickY - sphereCenterScreenSpaceCoord[1], 2);
			double z = pow(g_arcballScreenRadius, 2) - pow(x - sphereCenterScreenSpaceCoord[0], 2) - pow((g_windowHeight - y - 1) - sphereCenterScreenSpaceCoord[1], 2);

			if (mouseClickZ >= 0) {
				mouseClickZ = sqrt(mouseClickZ);
				temp = Cvec3(g_mouseClickX - sphereCenterScreenSpaceCoord[0], g_mouseClickY - sphereCenterScreenSpaceCoord[1], mouseClickZ).normalize();
			}
			else {
				temp = Cvec3(g_mouseClickX - sphereCenterScreenSpaceCoord[0], g_mouseClickY - sphereCenterScreenSpaceCoord[1], 0).normalize();
			}
			src = Quat(0, -1 * temp[0], -1 * temp[1], -1 * temp[2]);

			if (z >= 0) {
				z = sqrt(z);
				temp = Cvec3(x - sphereCenterScreenSpaceCoord[0], (g_windowHeight - y - 1) - sphereCenterScreenSpaceCoord[1], z).normalize();
			}
			else {
				temp = Cvec3(x - sphereCenterScreenSpaceCoord[0], (g_windowHeight - y - 1) - sphereCenterScreenSpaceCoord[1], 0).normalize();
			}
			dest = Quat(0, temp[0], temp[1], temp[2]);

			m = RigTForm(dest * src);
		}
		else {
			m = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
		}
	}
	else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
		if (g_objectView == 0) {
			m = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale);
		}
		else {
			m = RigTForm(Cvec3(dx, dy, 0) * 0.01);
		}
	}
	else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
		if (g_objectView == 0) {
			m = RigTForm(Cvec3(0, 0, -dy) * g_arcballScale);
		}
		else {
			m = RigTForm(Cvec3(0, 0, -dy) * 0.01);
		}
	}

	if (g_mouseClickDown) {
		if (g_currentPickedRbtNode == NULL || (g_objectView == 1 && g_currentPickedRbtNode == g_robot1Node) || (g_objectView == 2 && g_currentPickedRbtNode == g_robot2Node)) {
			RigTForm a;
			if (g_objectView == 0) {
				if (g_skyMode == 0) {
					a = linFact(getPathAccumRbt(g_world, g_skyNode));
					a = inv(getPathAccumRbt(g_world, g_skyNode, 1)) * a;
					m = inv(m);
				}
				else {
					a = getPathAccumRbt(g_world, g_skyNode);
					a = inv(getPathAccumRbt(g_world, g_skyNode, 1)) * a;
					m = transFact(m) * inv(linFact(m));
				}
				g_skyNode->setRbt(a * m * inv(a) * g_skyNode->getRbt());
				g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
			}
			else if (g_objectView == 1) {
				a = getPathAccumRbt(g_world, g_robot1Node);
				a = inv(getPathAccumRbt(g_world, g_robot1Node, 1)) * a;
				m = transFact(m) * inv(linFact(m));

				g_robot1Node->setRbt(a * m * inv(a) * g_robot1Node->getRbt());
				g_viewpointRbt = getPathAccumRbt(g_world, g_robot1Node);
			}
			else {
				a = getPathAccumRbt(g_world, g_robot2Node);
				a = inv(getPathAccumRbt(g_world, g_robot2Node, 1)) * a;
				m = transFact(m) * inv(linFact(m));

				g_robot2Node->setRbt(a * m * inv(a) * g_robot2Node->getRbt());
				g_viewpointRbt = getPathAccumRbt(g_world, g_robot2Node);
				g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
			}
		}
		else {
			RigTForm a = transFact(getPathAccumRbt(g_world, g_currentPickedRbtNode)) * linFact(g_viewpointRbt);
			a = inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * a;
			g_currentPickedRbtNode->setRbt(a * m * inv(a) * g_currentPickedRbtNode->getRbt());
			g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
		}
		glutPostRedisplay(); // we always redraw if we changed the scene
	}

	g_mouseClickX = x;
	g_mouseClickY = g_windowHeight - y - 1;
}


static void mouse(const int button, const int state, const int x, const int y) {
	g_mouseClickX = x;
	g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

	g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
	g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
	g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

	g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
	g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
	g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

	g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

	if (g_mouseLClickButton && g_isPicking) {
		pick();
		g_isPicking = !g_isPicking;
	}
	
	if (g_currentPickedRbtNode == NULL) {
		g_sphereRbt = RigTForm();
	}
	else {
		g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
	}

	glutPostRedisplay();
}

static void showCurrentKeyFrame() {
	vector<RigTForm>::iterator it_rbt = (*g_currentKeyFrameIterator).begin();
	for (vector<shared_ptr<SgRbtNode>>::iterator it_ptr = g_allNodes.begin(); it_ptr != g_allNodes.end(); ++it_ptr, ++it_rbt) {
		(*it_ptr)->setRbt(*it_rbt);
	}

	if (g_currentPickedRbtNode == NULL) {
		g_sphereRbt = RigTForm();
	}
	else {
		g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
	}

	if (g_objectView == 0) {
		g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
	}
	else if (g_objectView == 1) {
		g_viewpointRbt = getPathAccumRbt(g_world, g_robot1Node);
	}
	else {
		g_viewpointRbt = getPathAccumRbt(g_world, g_robot2Node);
	}
}

// Given t in the range [0, n], perform interpolation and draw the scene
// for the particular t. Returns true if we are at the end of the animation
// sequence, or false otherwise.
bool interpolateAndDisplay(float t) {
	int offset = floor(t);
	list<vector<RigTForm>>::iterator it1 = g_keyFrameList.begin(), it2 = g_keyFrameList.end();
	vector<RigTForm>::iterator it_rbt1, it_rbt2;
	vector<shared_ptr<SgRbtNode>>::iterator it_node;
	
	if (g_stopAnimation) {
		g_stopAnimation = false;
		return true;
	}

	++it1;
	--it2;
	--it2;

	for (int i = 0; i < offset; i++) {
		++it1;
		if (it1 == it2)
			return true;
	}

	it2 = it1;
	++it2;
	
	it_rbt1 = (*it1).begin();
	it_rbt2 = (*it2).begin();

	for (it_node = g_allNodes.begin(); it_node != g_allNodes.end(); ++it_node, ++it_rbt1, ++it_rbt2) {
		//cout << "time: " << t - offset << endl;
		(*it_node)->setRbt(interpolate((*it_rbt1), (*it_rbt2), t - offset));

		//RigTForm temp = (*it_node)->getRbt();
		//cout << "Cvec3: " << temp.getTranslation()[0] << " " << temp.getTranslation()[1] << " " << temp.getTranslation()[2] << " " << "\nQuat: " << temp.getRotation()[0] << " " << temp.getRotation()[1] << " " << temp.getRotation()[2] << " " << temp.getRotation()[3] << endl;
	}
	return false;
}
// Interpret "ms" as milliseconds into the animation
static void animateTimerCallback(int ms) {
	float t = (float)ms / (float)g_msBetweenKeyFrames;
	bool endReached = interpolateAndDisplay(t);

	if (g_objectView == 0) {
		g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
	}
	else if (g_objectView == 1) {
		g_viewpointRbt = getPathAccumRbt(g_world, g_robot1Node);
	}
	else {
		g_viewpointRbt = getPathAccumRbt(g_world, g_robot2Node);
	}

	if (g_currentPickedRbtNode == NULL) {
		g_sphereRbt = RigTForm();
	}
	else {
		g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
	}

	glutPostRedisplay();
	
	if (!endReached)
		glutTimerFunc(1000 / g_animateFramesPerSecond, animateTimerCallback, ms + 1000 / g_animateFramesPerSecond);
	else {
		g_playingAnimation = false;
		g_currentKeyFrameIterator = g_keyFrameList.end();
		--g_currentKeyFrameIterator;
		--g_currentKeyFrameIterator;
		g_KeyFrameIndex = g_keyFrameList.size() - 2;
		cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
		showCurrentKeyFrame();
	}
}

static void keyboard(const unsigned char key, const int x, const int y) {
	switch (key) {
	case 27:
		exit(0);                                  // ESC
	case 'h':
		cout << " ============== H E L P ==============\n\n"
			<< "h\t\thelp menu\n"
			<< "s\t\tsave screenshot\n"
			<< "f\t\tToggle flat shading on/off.\n"
			<< "o\t\tCycle object to edit\n"
			<< "v\t\tCycle view\n"
			<< "m\t\tCycle sky mode"
			<< "drag left mouse to rotate\n" << endl;
		break;
	case 's':
		glFlush();
		writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
		break;
	case 'f':
		g_activeShader ^= 1;
		break;
	case 'v':
		g_objectView = (g_objectView + 1) % 3;
		if (g_objectView == 0) {
			g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
			cout << "Current viewpoint: sky camera" << endl;
		}
		else if (g_objectView == 1) {
			g_viewpointRbt = getPathAccumRbt(g_world, g_robot1Node);
			cout << "Current viewpoint: cube 1" << endl;
		}
		else {
			g_viewpointRbt = getPathAccumRbt(g_world, g_robot2Node);
			cout << "Current viewpoint: cube 2" << endl;
		}
		break;
	case 'm':
		g_skyMode = (g_skyMode + 1) % 2;
		if (g_skyMode == 0) {
			cout << "World-sky frame" << endl;
			g_sphereRbt = RigTForm(Cvec3(0, 0, 0));
		}
		else {
			cout << "Sky-sky frame" << endl;
		}
		break;
	case 'p':
		g_isPicking = !g_isPicking;
		cout << (g_isPicking ? "Picking On" : "Picking Off") << endl;
		break;
	case ' ':		// Show current keyframe
		if (!g_keyFrameList.empty() && !g_playingAnimation) {
			showCurrentKeyFrame();
			cout << "Loading current key frame [" << g_KeyFrameIndex << "] to scene graph" << endl;
		}
		break;
	case 'u':		// Update current keyframe
		if (!g_playingAnimation) {
			if (!g_keyFrameList.empty()) {
				(*g_currentKeyFrameIterator).clear();

				for (vector<shared_ptr<SgRbtNode>>::iterator it_ptr = g_allNodes.begin(); it_ptr != g_allNodes.end(); ++it_ptr) {
					(*g_currentKeyFrameIterator).push_back((*it_ptr)->getRbt());
				}

				cout << "Copying scene graph to current frame [" << g_KeyFrameIndex << "]" << endl;
			}
			else {
				vector<RigTForm> currentFrame;

				for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allNodes.begin(); it != g_allNodes.end(); ++it) {
					currentFrame.push_back((*it)->getRbt());
				}

				g_keyFrameList.push_back(currentFrame);
				g_currentKeyFrameIterator = g_keyFrameList.begin();
				g_KeyFrameIndex = 0;

				cout << "Create new frame [" << g_KeyFrameIndex << "]" << endl;
			}
		}
		break;
	case '>':		// Move to next keyframe
		if (!g_keyFrameList.empty() && !g_playingAnimation) {
			list<vector<RigTForm>>::iterator it = g_keyFrameList.end();
			--it;
			if (g_currentKeyFrameIterator != it) {
				++g_currentKeyFrameIterator;
				g_KeyFrameIndex += 1;
				cout << "Stepped forward to frame [" << g_KeyFrameIndex << "]" << endl;
				showCurrentKeyFrame();
			}
		}
		break;
	case '<':		// Move to previous keyframe
		if (!g_keyFrameList.empty() && g_currentKeyFrameIterator != g_keyFrameList.begin() && !g_playingAnimation) {
			--g_currentKeyFrameIterator;
			g_KeyFrameIndex -= 1;
			showCurrentKeyFrame();

			cout << "Stepped backward to frame [" << g_KeyFrameIndex << "]" << endl;
		}
		break;
	case 'd':		// Delete current keyframe
		if (!g_keyFrameList.empty() && !g_playingAnimation) {
			g_currentKeyFrameIterator = g_keyFrameList.erase(g_currentKeyFrameIterator);

			if (!g_keyFrameList.empty()) {
				if (g_currentKeyFrameIterator != g_keyFrameList.begin()) {
					--g_currentKeyFrameIterator;
					g_KeyFrameIndex -= 1;
				}
				cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
				showCurrentKeyFrame();
			}
			else {
				cout << "Frame list is now EMPTY" << endl;;
			}
		}
		break;
	case 'n':		// Create current keyframe		
	{
		vector<RigTForm> currentFrame;

		for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allNodes.begin(); it != g_allNodes.end(); ++it) {
			currentFrame.push_back((*it)->getRbt());
		}

		if (g_keyFrameList.empty()) {
			g_keyFrameList.push_back(currentFrame);
			g_currentKeyFrameIterator = g_keyFrameList.begin();
			g_KeyFrameIndex = 0;
		}
		else {
			++g_currentKeyFrameIterator;
			g_keyFrameList.insert(g_currentKeyFrameIterator, currentFrame);
			--g_currentKeyFrameIterator;
			g_KeyFrameIndex += 1;
		}
		cout << "Create new frame [" << g_KeyFrameIndex << "]" << endl;
		break;
	}
	case 'i':		// read keyframes file
		if (!g_playingAnimation) {
			ifstream f("animation.txt");
			int keyframes, nodes, cnt;
			double t0, t1, t2, q0, q1, q2, q3;

			g_keyFrameList.clear();
			f >> keyframes >> nodes;
			cnt = keyframes;
			
			while (!f.eof() && f.is_open() && cnt > 0) {
				vector<RigTForm> keyframe;
				for (int i = 0; i < nodes && !f.eof() && f.is_open(); i++) {
					f >> t0 >> t1 >> t2 >> q0 >> q1 >> q2 >> q3;
					Cvec3 _t = Cvec3(t0, t1, t2);
					Quat _r = Quat(q0, q1, q2, q3);
					RigTForm rbt = RigTForm(_t, _r);
					keyframe.push_back(rbt);
				}
				cnt--;

				g_keyFrameList.push_back(keyframe);
			}

			g_KeyFrameIndex = 0;

			cout << keyframes << " frames read." << endl;
			if (!g_keyFrameList.empty()) {
				g_currentKeyFrameIterator = g_keyFrameList.begin();
				showCurrentKeyFrame();
				cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
			}
		}
		break;
	case 'w':		// write keyframes file
	{
		ofstream f("animation.txt", ios::binary);
		cout << "Writing animation to animation.txt" << endl;

		f << g_keyFrameList.size() << " " << g_allNodes.size() << endl;
		for (list<vector<RigTForm>>::iterator it = g_keyFrameList.begin(); it != g_keyFrameList.end(); ++it) {
			vector<RigTForm> keyframe = *it;
			for (vector<RigTForm>::iterator it_rbt = keyframe.begin(); it_rbt != keyframe.end(); ++it_rbt) {
				RigTForm rbt = *it_rbt;
				Cvec3 _t = rbt.getTranslation();
				Quat _r = rbt.getRotation();
				f << _t[0] << " " << _t[1] << " " << _t[2] << " " << _r(0) << " " << _r(1) << " " << _r(2) << " " << _r(3) << endl;
			}
		}
		break;
	}
	case 'y':
		if (!g_playingAnimation) {
			if (g_keyFrameList.size() > 3) {
				if (!g_playingAnimation) {
					cout << "Play animation..." << endl;
					g_playingAnimation = true;
					animateTimerCallback(0);
				}
			}
			else {
				cout << "At least 4 frames are needed to play animation" << endl;
			}
		}
		else {
			g_stopAnimation = true;
		}
		break;
	case '+':
		g_msBetweenKeyFrames -= g_msBetweenKeyFrames > 100 ? 100 : 0;
		cout << g_msBetweenKeyFrames << " ms between key frames" << endl;
		break;
	case '-':
		g_msBetweenKeyFrames += 100;
		cout << g_msBetweenKeyFrames << " ms between key frames" << endl;
		break;
	}
	glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
	glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
	glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
	glutCreateWindow("Assignment 2");                       // title the window

	glutDisplayFunc(display);                               // display rendering callback
	glutReshapeFunc(reshape);                               // window reshape callback
	glutMotionFunc(motion);                                 // mouse movement callback
	glutMouseFunc(mouse);                                   // mouse click callback
	glutKeyboardFunc(keyboard);
}

static void initGLState() {
	glClearColor(128. / 255., 200. / 255., 255. / 255., 0.);
	glClearDepth(0.);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_GREATER);
	glReadBuffer(GL_BACK);
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
	initGround();
	initCubes();
	initSphere();
	initBodySphere();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

	const double ARM_LEN = 0.7,
		ARM_THICK = 0.25,
		LEG_LEN = 1.0,
		LEG_THICK = 0.25,
		TORSO_LEN = 1.5,
		TORSO_THICK = 0.25,
		TORSO_WIDTH = 1,
		HEAD = 0.35;
	const int NUM_JOINTS = 10,
		NUM_SHAPES = 10;

	struct JointDesc {
		int parent;
		float x, y, z;
	};

	JointDesc jointDesc[NUM_JOINTS] = {
		{ -1 }, // torso
	{ 0,  TORSO_WIDTH / 2, TORSO_LEN / 2, 0 }, // upper right arm
	{ 1,  ARM_LEN, 0, 0 }, // lower right arm
	{ 0,  -TORSO_WIDTH / 2, TORSO_LEN / 2, 0 }, // upper left arm
	{ 3,  -ARM_LEN, 0, 0 }, // lower left arm
	{ 0,  TORSO_WIDTH / 2, -TORSO_LEN / 2, 0 }, // upper right leg
	{ 5,  0, -LEG_LEN, 0 }, // lower right leg
	{ 0,  -TORSO_WIDTH / 2, -TORSO_LEN / 2, 0 }, // upper left leg
	{ 7,  0, -LEG_LEN, 0 }, // lower left leg
	{ 0, 0, TORSO_LEN / 2, 0} // head
	};

	struct ShapeDesc {
		int parentJointId;
		float x, y, z, sx, sy, sz;
		shared_ptr<Geometry> geometry;
	};

	ShapeDesc shapeDesc[NUM_SHAPES] = {
		{ 0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube }, // torso
	{ 1, ARM_LEN / 2, 0, 0, ARM_LEN / 2, ARM_THICK / 2, ARM_THICK / 2, g_bodySphere }, // upper right arm
	{ 2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube }, // lower right arm
	{ 3, -ARM_LEN / 2, 0, 0, ARM_LEN / 2, ARM_THICK / 2, ARM_THICK / 2, g_bodySphere }, // upper left arm
	{ 4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube }, // lower left arm
	{ 5, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_bodySphere }, // upper right leg
	{ 6, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube }, // lower right leg
	{ 7, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_bodySphere }, // upper left leg
	{ 8, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube }, // lower left leg
	{ 9, 0, HEAD * 1.5, 0, HEAD, HEAD, HEAD, g_bodySphere} // head
	};

	shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

	for (int i = 0; i < NUM_JOINTS; ++i) {
		if (jointDesc[i].parent == -1)
			jointNodes[i] = base;
		else {
			jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
			jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
		}
	}
	for (int i = 0; i < NUM_SHAPES; ++i) {
		shared_ptr<MyShapeNode> shape(
			new MyShapeNode(shapeDesc[i].geometry,
				color,
				Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
				Cvec3(0, 0, 0),
				Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
		jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
	}
}

static void initScene() {
	g_world.reset(new SgRootNode());

	g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

	g_groundNode.reset(new SgRbtNode());
	g_groundNode->addChild(shared_ptr<MyShapeNode>(
		new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

	g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
	g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

	constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
	constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

	g_world->addChild(g_skyNode);
	g_world->addChild(g_groundNode);
	g_world->addChild(g_robot1Node);
	g_world->addChild(g_robot2Node);
}

int main(int argc, char * argv[]) {
	try {
		initGlutState(argc, argv);

		glewInit(); // load the OpenGL extensions

		cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
		if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
		else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

		initGLState();
		initShaders();
		initGeometry();
		initScene();
		g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
		dumpSgRbtNodes(g_world, g_allNodes);

		glutMainLoop();
		return 0;
	}
	catch (const runtime_error& e) {
		cout << "Exception caught: " << e.what() << endl;
		return -1;
	}
}
