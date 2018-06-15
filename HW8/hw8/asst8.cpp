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
#include "geometry.h"
#include "mesh.h"

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

// --------- Materials

static shared_ptr<Material> g_redDiffuseMat, g_blueDiffuseMat, g_bumpFloorMat, g_arcballMat, g_pickingMat, g_lightMat;
shared_ptr<Material> g_overridingMaterial;

static shared_ptr<Material> g_meshCubeMat;

// --------- Geometry

typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_bodySphere, g_lightSphere;
static shared_ptr<SimpleGeometryPN> g_meshCube;
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_meshCubeNode;
static shared_ptr<SgRbtNode> g_light1Node, g_light2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

// --------- Scene

static const Cvec3 g_light1(1.0, 2.0, 2.0), g_light2(-1.0, 2.0, -2.0);  // define two lights positions in world space
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

// Mesh
Mesh g_mesh;
static int g_vertexFramesPerSecond = 60;
static int g_shadingMode = 0;
static float g_deformSpeed = 0.1;
static int g_subdivisionStep = 0;
static float g_timeElapse = 0;


///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
	int ibLen, vbLen;
	getPlaneVbIbLen(vbLen, ibLen);

	// Temporary storage for cube Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
	g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
	int ibLen, vbLen;
	getCubeVbIbLen(vbLen, ibLen);

	// Temporary storage for cube Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeCube(1, vtx.begin(), idx.begin());
	g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(20, 10, vbLen, ibLen);

	// Temporary storage for sphere Geometry
	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);
	makeSphere(g_arcballScreenRadius, 20, 10, vtx.begin(), idx.begin());
	g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initBodySphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(40, 40, vbLen, ibLen);

	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeSphere(1, 40, 40, vtx.begin(), idx.begin());
	g_bodySphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initLightSphere() {
	int ibLen, vbLen;
	getSphereVbIbLen(40, 40, vbLen, ibLen);

	vector<VertexPNTBX> vtx(vbLen);
	vector<unsigned short> idx(ibLen);

	makeSphere(0.25, 40, 40, vtx.begin(), idx.begin());
	g_lightSphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void meshFlatShading(shared_ptr<SimpleGeometryPN> geometry, Mesh& mesh) {
	vector<VertexPN> vertices;
	
	for (int i = 0; i < mesh.getNumFaces(); i++) {
		const Mesh::Face f = mesh.getFace(i);
		Cvec3 normal = f.getNormal();

		Mesh::Vertex v0 = f.getVertex(0);
		Mesh::Vertex v1 = f.getVertex(1);
		Mesh::Vertex v2 = f.getVertex(2);
		Mesh::Vertex v3 = f.getVertex(3);

		VertexPN vtx0(v0.getPosition(), normal);
		VertexPN vtx1(v1.getPosition(), normal);
		VertexPN vtx2(v2.getPosition(), normal);
		VertexPN vtx3(v3.getPosition(), normal);

		vertices.push_back(vtx0);
		vertices.push_back(vtx1);
		vertices.push_back(vtx2);
		vertices.push_back(vtx0);
		vertices.push_back(vtx2);
		vertices.push_back(vtx3);
	}
	
	geometry->upload(&vertices[0], mesh.getNumFaces() * 6);
}

static void meshSmoothShading(shared_ptr<SimpleGeometryPN> geometry, Mesh& mesh) {
	vector<VertexPN> vertices;

	for (int i = 0; i < mesh.getNumVertices(); i++) {
		const Mesh::Vertex v = mesh.getVertex(i);
		v.setNormal(Cvec3(0, 0, 0));
	}

	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		const Mesh::Face f = mesh.getFace(i);
		Cvec3 normal = f.getNormal();

		for (int j = 0; j < f.getNumVertices(); j++) {
			const Mesh::Vertex v = f.getVertex(j);
			v.setNormal(v.getNormal() + f.getNormal());
		}
	}

	for (int i = 0; i < mesh.getNumVertices(); i++) {
		const Mesh::Vertex v = mesh.getVertex(i);
		int cnt0 = 0;

		Mesh::VertexIterator it(v.getIterator()), it0(it);
		do
		{
			cnt0++;
		} while (++it != it0);                                  // go around once the 1ring

		v.setNormal(v.getNormal() / (float)cnt0);
	}

	for (int i = 0; i < mesh.getNumFaces(); i++) {
		const Mesh::Face f = mesh.getFace(i);

		Mesh::Vertex v0 = f.getVertex(0);
		Mesh::Vertex v1 = f.getVertex(1);
		Mesh::Vertex v2 = f.getVertex(2);
		Mesh::Vertex v3 = f.getVertex(3);

		VertexPN vtx0(v0.getPosition(), v0.getNormal());
		VertexPN vtx1(v1.getPosition(), v1.getNormal());
		VertexPN vtx2(v2.getPosition(), v2.getNormal());
		VertexPN vtx3(v3.getPosition(), v3.getNormal());

		vertices.push_back(vtx0);
		vertices.push_back(vtx1);
		vertices.push_back(vtx2);
		vertices.push_back(vtx0);
		vertices.push_back(vtx2);
		vertices.push_back(vtx3);
	}
	geometry->upload(&vertices[0], mesh.getNumFaces() * 6);
}

static void initMeshCube() {
	SimpleGeometryPN geometry;

	g_mesh.load("./cube.mesh");
	//uploadMeshToSimpleGeometryPN(geometry, m);
	g_meshCube.reset(new SimpleGeometryPN());
	meshFlatShading(g_meshCube, g_mesh);
}

// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
	uniforms.put("uProjMatrix", projMatrix);
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


static void drawStuff(bool picking) {
	Uniforms uniforms;

	// build & send proj. matrix to vshader
	const Matrix4 projmat = makeProjectionMatrix();
	sendProjectionMatrix(uniforms, projmat);

	// use the skyRbt as the eyeRbt
	const RigTForm eyeRbt = g_viewpointRbt;
	const RigTForm invEyeRbt = inv(eyeRbt);

	Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
	Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();

	uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1, 1)));
	uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2, 1)));

	if (!picking) {
		Drawer drawer(invEyeRbt, uniforms);
		g_world->accept(drawer);

		// draw arcball as part of asst3
		if (((g_objectView == 0 && g_skyMode == 0) || g_currentPickedRbtNode != NULL) && !((g_objectView == 1 && g_currentPickedRbtNode == g_robot1Node) || (g_objectView == 2 && g_currentPickedRbtNode == g_robot2Node))) {
			if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)))
				g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight);
			Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_sphereRbt) * Matrix4::makeScale(Cvec3(g_arcballScale, g_arcballScale, g_arcballScale));
			Matrix4 NMVM = normalMatrix(MVM);
			sendModelViewNormalMatrix(uniforms, MVM, NMVM);
			g_arcballMat->draw(*g_sphere, uniforms);
		}
	}
	else {
		Picker picker(invEyeRbt, uniforms);
		g_overridingMaterial = g_pickingMat;
		g_world->accept(picker);
		g_overridingMaterial.reset();
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

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// No more glUseProgram
	drawStuff(true); // no more curSS

					 // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
					 // to see result of the pick rendering pass
					 // glutSwapBuffers();

					 //Now set back the clear color
	glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

	checkGlErrors();
}

static void display() {
	// No more glUseProgram

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawStuff(false); // no more curSS

	glutSwapBuffers();

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
	list<vector<RigTForm>>::iterator it0, it1 = g_keyFrameList.begin(), it2 = g_keyFrameList.end(), it3;
	vector<RigTForm>::iterator it_rbt0, it_rbt1, it_rbt2, it_rbt3;
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
	
	it0 = it2 = it3 = it1;
	--it0;
	++it2;
	++it3;
	++it3;
	
	it_rbt0 = (*it0).begin();
	it_rbt1 = (*it1).begin();
	it_rbt2 = (*it2).begin();
	it_rbt3 = (*it3).begin();
	
	for (it_node = g_allNodes.begin(); it_node != g_allNodes.end(); ++it_node, ++it_rbt0, ++it_rbt1, ++it_rbt2, ++it_rbt3) {
		//std::cout << "time: " << t - offset << endl;
		(*it_node)->setRbt(interpolate((*it_rbt0), (*it_rbt1), (*it_rbt2), (*it_rbt3), t - offset));

		//RigTForm temp = (*it_node)->getRbt();
		//std::cout << "Cvec3: " << temp.getTranslation()[0] << " " << temp.getTranslation()[1] << " " << temp.getTranslation()[2] << " " << "\nQuat: " << temp.getRotation()[0] << " " << temp.getRotation()[1] << " " << temp.getRotation()[2] << " " << temp.getRotation()[3] << endl;
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
		std::cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
		showCurrentKeyFrame();
	}
}

static void catmullClark(Mesh& mesh) {
	for (int i = 0; i < mesh.getNumFaces(); i++) {
		const Mesh::Face f = mesh.getFace(i);
		int num = f.getNumVertices();
		Cvec3 faceVertex = Cvec3(0.0, 0.0, 0.0);

		for (int j = 0; j < num; j++) {
			const Mesh::Vertex v = f.getVertex(j);
			faceVertex = faceVertex + v.getPosition();
		}

		faceVertex /= num;
		//cout << faceVertex[0] << " " << faceVertex[1] << " " << faceVertex[2] << endl << endl;
		mesh.setNewFaceVertex(f, faceVertex);
	}

	for (int i = 0; i < mesh.getNumEdges(); i++) {
		const Mesh::Edge e = mesh.getEdge(i);
		Cvec3 edgeVertex = Cvec3(0.0, 0.0, 0.0);

		edgeVertex = (e.getVertex(0).getPosition() + e.getVertex(1).getPosition() + mesh.getNewFaceVertex(e.getFace(0)) + mesh.getNewFaceVertex(e.getFace(1))) / 4;
		//cout << edgeVertex[0] << " " << edgeVertex[1] << " " << edgeVertex[2] << endl << endl;
		mesh.setNewEdgeVertex(e, edgeVertex);
	}

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		const Mesh::Vertex v = mesh.getVertex(i);
		Cvec3 vertexVertex = v.getPosition();
		Cvec3 sumFaceVertex = Cvec3(0.0, 0.0, 0.0);
		Cvec3 sumRingVertex = Cvec3(0.0, 0.0, 0.0);
		int cnt = 0;

		Mesh::VertexIterator it(v.getIterator()), it0(it);
		do
		{
			const Mesh::Vertex v_ = it.getVertex();
			const Mesh::Face f = it.getFace();
			sumRingVertex = sumRingVertex + v_.getPosition();
			sumFaceVertex = sumFaceVertex + mesh.getNewFaceVertex(f);
			cnt++;
		} while (++it != it0);                                  // go around once the 1ring

		vertexVertex = vertexVertex * (cnt - 2) / cnt + sumRingVertex / pow(cnt, 2) + sumFaceVertex / pow(cnt, 2);
		//cout << vertexVertex[0] << " " << vertexVertex[1] << " " << vertexVertex[2] << endl << endl;
		mesh.setNewVertexVertex(v, vertexVertex);
	}

	mesh.subdivide();
}

static void vertexTimerCallback(int ms) {
	Mesh g_meshTemp;
	g_meshTemp = Mesh(g_mesh);
	g_timeElapse += g_deformSpeed;
	for (int i = 0; i < g_meshTemp.getNumVertices(); i++) {
		const Mesh::Vertex v = g_meshTemp.getVertex(i);
		const Mesh::Vertex v_ = g_mesh.getVertex(i);

		v.setPosition(v_.getPosition() + v_.getPosition() * (sin(g_timeElapse * sqrt(i+1)) * 0.8));
	}

	for (int i = 0; i < g_subdivisionStep; i++) {
		catmullClark(g_meshTemp);
	}

		//cout << "g_meshSubdivided: " << g_meshSubdivided.getNumVertices() << " g_meshTemp : " << g_meshTemp.getNumVertices() << endl;

	if (g_shadingMode == 0)
		meshFlatShading(g_meshCube, g_meshTemp);
	else
		meshSmoothShading(g_meshCube, g_meshTemp);
	glutPostRedisplay();
	glutTimerFunc(1000 / g_vertexFramesPerSecond, vertexTimerCallback, 0);
}

static void keyboard(const unsigned char key, const int x, const int y) {
	switch (key) {
	case 27:
		exit(0);                                  // ESC
	case 'h':
		std::cout << " ============== H E L P ==============\n\n"
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
	case 'v':
		g_objectView = (g_objectView + 1) % 3;
		if (g_objectView == 0) {
			g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
			std::cout << "Current viewpoint: sky camera" << endl;
		}
		else if (g_objectView == 1) {
			g_viewpointRbt = getPathAccumRbt(g_world, g_robot1Node);
			std::cout << "Current viewpoint: cube 1" << endl;
		}
		else {
			g_viewpointRbt = getPathAccumRbt(g_world, g_robot2Node);
			std::cout << "Current viewpoint: cube 2" << endl;
		}
		break;
	case 'm':
		g_skyMode = (g_skyMode + 1) % 2;
		if (g_skyMode == 0) {
			std::cout << "World-sky frame" << endl;
			g_sphereRbt = RigTForm(Cvec3(0, 0, 0));
		}
		else {
			std::cout << "Sky-sky frame" << endl;
		}
		break;
	case 'p':
		g_isPicking = !g_isPicking;
		std::cout << (g_isPicking ? "Picking On" : "Picking Off") << endl;
		break;
	case ' ':		// Show current keyframe
		if (!g_keyFrameList.empty() && !g_playingAnimation) {
			showCurrentKeyFrame();
			std::cout << "Loading current key frame [" << g_KeyFrameIndex << "] to scene graph" << endl;
		}
		break;
	case 'u':		// Update current keyframe
		if (!g_playingAnimation) {
			if (!g_keyFrameList.empty()) {
				(*g_currentKeyFrameIterator).clear();

				for (vector<shared_ptr<SgRbtNode>>::iterator it_ptr = g_allNodes.begin(); it_ptr != g_allNodes.end(); ++it_ptr) {
					(*g_currentKeyFrameIterator).push_back((*it_ptr)->getRbt());
				}

				std::cout << "Copying scene graph to current frame [" << g_KeyFrameIndex << "]" << endl;
			}
			else {
				vector<RigTForm> currentFrame;

				for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allNodes.begin(); it != g_allNodes.end(); ++it) {
					currentFrame.push_back((*it)->getRbt());
				}

				g_keyFrameList.push_back(currentFrame);
				g_currentKeyFrameIterator = g_keyFrameList.begin();
				g_KeyFrameIndex = 0;

				std::cout << "Create new frame [" << g_KeyFrameIndex << "]" << endl;
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
				std::cout << "Stepped forward to frame [" << g_KeyFrameIndex << "]" << endl;
				showCurrentKeyFrame();
			}
		}
		break;
	case '<':		// Move to previous keyframe
		if (!g_keyFrameList.empty() && g_currentKeyFrameIterator != g_keyFrameList.begin() && !g_playingAnimation) {
			--g_currentKeyFrameIterator;
			g_KeyFrameIndex -= 1;
			showCurrentKeyFrame();

			std::cout << "Stepped backward to frame [" << g_KeyFrameIndex << "]" << endl;
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
				std::cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
				showCurrentKeyFrame();
			}
			else {
				std::cout << "Frame list is now EMPTY" << endl;;
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
		std::cout << "Create new frame [" << g_KeyFrameIndex << "]" << endl;
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

			std::cout << keyframes << " frames read." << endl;
			if (!g_keyFrameList.empty()) {
				g_currentKeyFrameIterator = g_keyFrameList.begin();
				showCurrentKeyFrame();
				std::cout << "Now at frame [" << g_KeyFrameIndex << "]" << endl;
			}
		}
		break;
	case 'w':		// write keyframes file
	{
		ofstream f("animation.txt", ios::binary);
		std::cout << "Writing animation to animation.txt" << endl;

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
					std::cout << "Play animation..." << endl;
					g_playingAnimation = true;
					animateTimerCallback(0);
				}
			}
			else {
				std::cout << "At least 4 frames are needed to play animation" << endl;
			}
		}
		else {
			g_stopAnimation = true;
		}
		break;
	case '+':
		g_msBetweenKeyFrames -= g_msBetweenKeyFrames > 100 ? 100 : 0;
		std::cout << g_msBetweenKeyFrames << " ms between key frames" << endl;
		break;
	case '-':
		g_msBetweenKeyFrames += 100;
		std::cout << g_msBetweenKeyFrames << " ms between key frames" << endl;
		break;
	case 'f':
		g_shadingMode = (g_shadingMode + 1) % 2;
		break;
	case '0':
		if (g_subdivisionStep < 7) {
			g_subdivisionStep++;
		}
		break;
	case '9':
		if (g_subdivisionStep > 0) {
			g_subdivisionStep--;
		}
		break;
	case '7':
		g_deformSpeed /= 2;
		break;
	case '8':
		if (g_deformSpeed < 10)
			g_deformSpeed *= 2;
		break;
	}

	glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
	glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
	glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
	glutCreateWindow("Assignment 8");                       // title the window

	glutDisplayFunc(display);                               // display rendering callback
	glutReshapeFunc(reshape);                               // window reshape callback
	glutMotionFunc(motion);                                 // mouse movement callback
	glutMouseFunc(mouse);                                   // mouse click callback
	glutKeyboardFunc(keyboard);
	glutTimerFunc(1000 / g_vertexFramesPerSecond, vertexTimerCallback, 0);
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

static void initMaterials() {
	// Create some prototype materials
	Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
	Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

	// copy diffuse prototype and set red color
	g_redDiffuseMat.reset(new Material(diffuse));
	g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

	// copy diffuse prototype and set blue color
	g_blueDiffuseMat.reset(new Material(diffuse));
	g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

	// normal mapping material
	g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
	g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
	g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

	// copy solid prototype, and set to wireframed rendering
	g_arcballMat.reset(new Material(solid));
	g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
	g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// copy solid prototype, and set to color white
	g_lightMat.reset(new Material(solid));
	g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

	// pick shader
	g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));

	g_meshCubeMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader"));
	g_meshCubeMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));
}

static void initGeometry() {
	initGround();
	initCubes();
	initSphere();
	initBodySphere();
	initLightSphere();
	initMeshCube();
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {

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
		shared_ptr<SgGeometryShapeNode> shape(
			new MyShapeNode(shapeDesc[i].geometry,
				material, // USE MATERIAL as opposed to color
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
		new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

	g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
	g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

	constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
	constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

	g_light1Node.reset(new SgRbtNode(RigTForm(g_light1)));
	g_light1Node->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_lightSphere, g_lightMat, Cvec3(0, 0, 0))));
	g_light2Node.reset(new SgRbtNode(RigTForm(g_light2)));
	g_light2Node->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_lightSphere, g_lightMat, Cvec3(0, 0, 0))));

	g_meshCubeNode.reset(new SgRbtNode(RigTForm(Cvec3(0, 1, 0))));
	g_meshCubeNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_meshCube, g_meshCubeMat, Cvec3(0, 0, 0))));

	g_world->addChild(g_skyNode);
	g_world->addChild(g_groundNode);
	g_world->addChild(g_robot1Node);
	g_world->addChild(g_robot2Node);
	g_world->addChild(g_light1Node);
	g_world->addChild(g_light2Node);
	g_world->addChild(g_meshCubeNode);
}

int main(int argc, char * argv[]) {
	try {
		initGlutState(argc, argv);

		glewInit(); // load the OpenGL extensions

		std::cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
		if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
		else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

		initGLState();
		initMaterials();
		initGeometry();
		initScene();
		g_viewpointRbt = getPathAccumRbt(g_world, g_skyNode);
		dumpSgRbtNodes(g_world, g_allNodes);

		glutMainLoop();
		return 0;
	}
	catch (const runtime_error& e) {
		std::cout << "Exception caught: " << e.what() << endl;
		return -1;
	}
}
