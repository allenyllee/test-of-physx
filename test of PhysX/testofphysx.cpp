#include <iostream>
#include <PxPhysicsAPI.h>
#include <glut.h>
//#include <WinDef.h>
//#include <WinNT.h>
//#include <gl\GL.h>
//#include <PxExtensionsAPI.h>
#include <PxVisualDebuggerExt.h>
//#include <pvd\PxVisualDebugger.h>
#include <PxMat33.h>
#include <sstream>
#include <vector>
//#include <gl\GL.h>
//#include <gl\GLU.h>
//#include <GLIncludes.h>
//#include <RendererWindow.h>

using namespace std;
using namespace physx;
//using namespace SampleRenderer;


//link libraries
#pragma comment(lib, "PhysX3CHECKED_x86.lib")
#pragma comment(lib, "PhysX3CommonCHECKED_x86.lib")
#pragma comment(lib, "PhysX3ExtensionsCHECKED.lib")
#pragma comment(lib, "PhysXVisualDebuggerSDKCHECKED.lib")

//Window size constant
const int WINDOW_WIDTH=1024,
	WINDOW_HEIGHT=768;
const int BOXES_NB = 100;

//Physx Physics
static PxPhysics *mPhysics = NULL;
static PxFoundation *mFoundation = NULL;
static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader = PxDefaultSimulationFilterShader;

PxMaterial* mMaterial = NULL;
PxScene *mScene = NULL;
PxReal myTimestep = 1.0f/60.0f;
//PxRigidActor *box = NULL;
vector <PxRigidActor*> boxes; 

//for mouse dragging
int oldX=0, oldY=0;
float rX=15, rY=0; //degree
float fps=0;
int startTime=0;
int totalFrames=0;
int state = 1;
float dist=-15;
float transX=0;
float transY=0;
string buffer;

//for picking
PxDistanceJoint *mMouseJoint = NULL;
PxRigidDynamic *mMouseSphere = NULL;
PxReal mMouseDepth = 0.0f;
PxRigidDynamic *mSelectedActor = NULL;
struct Ray{
	PxVec3 orig, dir;
};
int mousePosX,mousePosY;


GLdouble modelMatrix[16];
GLdouble projMatrix[16];
GLint viewPort[4];
bool isPick = false;

//functions
void initPhysX(void);
void initScene(void);
void OnShutdown(void);
void shutdownPhysX(void);

void CreatePlane(void);
void CreateCube(void);

void RendererActors(void);
void DrawActor(PxRigidActor *actor);
void DrawShape(PxShape* shape);
void DrawBox(PxShape *pShape);
void getColumnMajor(PxMat33 m, PxVec3 t, float* mat);

void Display(void);
void StepPhysX(void);
void DrawAxes(void);
void RenderSpacedBitmapString(int x, int y, int spacing, void *font, char *string); 
void DrawGrid(int GRID_SIZE);
void SetOrthoForFont(void);
void RenderSpaceBitmapString(int x, int y, int spacing, void *font, const char *string);
void ResetPerspectiveProjection(void);

//pick actor
void MoveActor(int x, int y);
bool PickActor(int x, int y);
void LetGoActor(void);
PxRigidDynamic *CreateSphere(const PxVec3 &pos, const PxReal radius, const PxReal density);
void ViewProject(PxVec3 &v, int &xi, int &yi, float &depth);
void ViewUnProject(int xi, int yi, float depth, PxVec3 &v);
void DrawDebugLine(int x1, int y1, int z1, int x2, int y2, int z2);
void DrawDebugPoint(int x, int y, int z);
void MotionCallback(int x, int y);

void OnIdle(void);
void OnReshape(int nw, int nh);
void Mouse(int button, int s, int x, int y);
void Motion(int x, int y);
void initGL(int argc, char** argv);

void PxVirtualDebugger(void);

/*
void fatalError(const char * msg)
{
printf("Fatal Error in SampleApplication: %s\n", msg);
//RendererWindow::close();
exit(1);
}
*/



void initPhysX(){
	cout<<"initializing PhysX"<<endl;

	cout<<"creating Foundation"<<endl;
	// create foundation object with default error and allocator callbacks.
	mFoundation = PxCreateFoundation(
		PX_PHYSICS_VERSION,
		gDefaultAllocatorCallback,
		gDefaultErrorCallback	
	);

	if(mFoundation == NULL){
		cerr<<"Error creating Foundation."<<endl;
		cerr<<"Exiting..."<<endl;
		exit(1);
	}

	cout<<"creating Physics\n";
	// create Physics object with the created foundation and with a 'default' scale tolerance.
	mPhysics = PxCreatePhysics(
		PX_PHYSICS_VERSION,
		*mFoundation,
		PxTolerancesScale()
	);
	
	if(mPhysics == NULL){
		cerr<<"Error creating PhysX3 device."<<endl;
		cerr<<"Exiting..."<<endl;
		exit(1);
	}

	if(!PxInitExtensions(*mPhysics))
		cerr<<"PxInitExtensiond failed!"<<endl;

	cout<<"PhysX initialized\n";

}



void initScene(){
	cout<<"initial scene."<<endl;

	PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.8f, 0.0f);
	//customizeSceneDesc(sceneDesc);

	if(!sceneDesc.cpuDispatcher){
		PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(1);

		if(!mCpuDispatcher)
			cerr<<"PxDafaultCpuDispatcherCreate failed!"<<endl;
		
		sceneDesc.cpuDispatcher = mCpuDispatcher;
	}

	if(!sceneDesc.filterShader)
		sceneDesc.filterShader = gDefaultFilterShader;
	
	/*
	#ifdef PX_WINDOWS
	pxtask::CudaContextManager* mCudaContextManager = NULL;

	if(!sceneDesc.gpuDispatcher && mCudaContextManager)
	{
		sceneDesc.gpuDispatcher = mCudaContextManager->getGpuDispatcher();
	}
	#endif
	*/
	
	mScene = mPhysics->createScene(sceneDesc);

	if(!mScene)
		cerr<<"createScene failed!"<<endl;
	
	mScene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0);
	mScene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
	
	cout<<"scene initialized."<<endl;

}


void OnShutdown() {
	shutdownPhysX();
}

void shutdownPhysX(){
	//cout<<"shutting down\n";
	for(unsigned int i=0;i<boxes.size();i++){
		mScene->removeActor(*boxes[i]);
		boxes[i]->release();
	}
	mScene->release();
	mMaterial->release();
	mPhysics->release();
	//mFoundation->release();
	
}




void CreatePlane(){
	mMaterial = mPhysics->createMaterial(0.5, 0.5, 0.5);
	PxReal d = 0.0f;
	PxTransform pose = PxTransform(PxVec3(0.0f, 0, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxRigidStatic* plane = mPhysics->createRigidStatic(pose);
	if(!plane)
		cerr<<"create plane failed!"<<endl;
	PxShape* shape = plane->createShape(PxPlaneGeometry(), *mMaterial);
	if(!shape)
		cerr<<"create shpae failed!"<<endl;
	mScene->addActor(*plane);
}

void CreateCube(){

	PxReal density = 1.0f;
	PxTransform transform(PxVec3(0.0f, 10.0f, 0.0f), PxQuat(PxHalfPi, PxVec3(0.0f, 0.0f, 1.0f)));
	PxVec3 dimensions(0.5, 0.5, 0.5);
	PxBoxGeometry geometry(dimensions);

	for(int i=0;i<BOXES_NB;i++){
		transform.p = PxVec3(0.0f, 5.0f+5*i, 0.0f);
		PxRigidDynamic *actor = PxCreateDynamic(*mPhysics, transform, geometry, *mMaterial, density);
		if(!actor)
			cerr<<"create actor fialed"<<endl;
		actor->setAngularDamping(0.75);
		actor->setLinearVelocity(PxVec3(0, 0, 0));
		mScene->addActor(*actor);
		boxes.push_back(actor);
	}

	/*
	PxRigidDynamic *actor = PxCreateDynamic(*mPhysics, transform, geometry, *mMaterial, density);
	if(!actor)
		cerr<<"create actor fialed"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	mScene->addActor(*actor);

	box = actor;
	*/
}



void RendererActors(){
	//Render all the actors in the scene
	//DrawActor(box);
	for(unsigned int i=0;i<boxes.size();i++){
		DrawActor(boxes[i]);
	}
}

void DrawActor(PxRigidActor *actor){
	PxU32 nShapes = actor->getNbShapes();
	PxShape** shapes = new PxShape*[nShapes];
	actor->getShapes(shapes, nShapes);
	while(nShapes--){
		DrawShape(shapes[nShapes]);
	}
	delete [] shapes;
}

void DrawShape(PxShape* shape){
	PxGeometryType::Enum type = shape->getGeometryType();
	switch(type){
		case PxGeometryType::eBOX:
			DrawBox(shape);
		break;
	}
}

void DrawBox(PxShape *pShape){
	PxTransform pT = PxShapeExt::getGlobalPose(*pShape);
	PxBoxGeometry bg;
	pShape->getBoxGeometry(bg);
	PxMat33 m = PxMat33(pT.q);
	float mat[16];
	getColumnMajor(m, pT.p, mat);
	glPushMatrix();
		glMultMatrixf(mat);
		glutSolidCube(bg.halfExtents.x*2);
	glPopMatrix();
}

void getColumnMajor(PxMat33 m, PxVec3 t, float* mat) {
   mat[0] = m.column0[0];
   mat[1] = m.column0[1];
   mat[2] = m.column0[2];
   mat[3] = 0;

   mat[4] = m.column1[0];
   mat[5] = m.column1[1];
   mat[6] = m.column1[2];
   mat[7] = 0;

   mat[8] = m.column2[0];
   mat[9] = m.column2[1];
   mat[10] = m.column2[2];
   mat[11] = 0;

   mat[12] = t[0];
   mat[13] = t[1];
   mat[14] = t[2];
   mat[15] = 1;
}


void Display(){

	//Calculate fps
	stringstream stream;
	totalFrames++;
	int current = glutGet(GLUT_ELAPSED_TIME);
	if((current-startTime)>1000){
		float elapsedTime = float (current-startTime);
		fps = ((totalFrames * 1000.0f)/elapsedTime);
		startTime = current;
		totalFrames=0;
	}
	
	stream << fps << endl;

	buffer = "FPS: " + stream.str();
	
	//setup the view transformation using
	//gluLookAt(...);
	
	//Update PhysX
	if(mScene)
		StepPhysX();
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//glTranslatef(0, 0, dist);



		
	
	glTranslatef(transX, transY, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	DrawAxes();
	DrawGrid(10);
	

	
	glEnable(GL_LIGHTING);
		RendererActors();
	glDisable(GL_LIGHTING);

	//glLoadIdentity();
	
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);	
	
	PxVec3 v1,v2;
	ViewUnProject(mousePosX,mousePosY,0,v1);
	ViewUnProject(mousePosX,mousePosY,1,v2);
	DrawDebugLine(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);

	DrawDebugPoint(1,1,1);
	DrawDebugLine(0,0,0,1,1,1);
	
	SetOrthoForFont();
		glColor3f(1,1,1);
		//Show the fps
		RenderSpaceBitmapString(20, 20, 0, GLUT_BITMAP_HELVETICA_12, buffer.c_str());

	ResetPerspectiveProjection();




	glutSwapBuffers();
}

void StepPhysX(){
	mScene->simulate(myTimestep);
	
	//...perform useful work here using previous frame's state data        
	while(!mScene->fetchResults() )     
	{
      // do something useful        
	}

}

void DrawAxes()
{	 
	//To prevent the view from disturbed on repaint
	//this push matrix call stores the current matrix state
	//and restores it once we are done with the arrow rendering
	glPushMatrix();
		//Z-axis
		glColor3f(0,0,1);
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label			
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Z");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);
		//X-axis
		glColor3f(1,0,0);
		glRotatef(90,0,1,0);	
		glPushMatrix();
			glTranslatef(0,0,0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "X");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);
		//Y-axis
		glColor3f(0,1,0);
		glRotatef(90,-1,0,0);	
		glPushMatrix();
			glTranslatef(0,0, 0.8f);
			glutSolidCone(0.0325,0.2, 4,1);
			//Draw label
			glTranslatef(0,0.0625,0.225f);
			RenderSpacedBitmapString(0,0,0,GLUT_BITMAP_HELVETICA_10, "Y");
		glPopMatrix();					
		glutSolidCone(0.0225,1, 4,1);	
	glPopMatrix();
}
void RenderSpacedBitmapString(
							  int x, 
							  int y,
							  int spacing, 
							  void *font,
							  char *string) 
{
	char *c;
	int x1=x;
	for (c=string; *c != '\0'; c++) {
		glRasterPos2i(x1,y);
		glutBitmapCharacter(font, *c);
		x1 = x1 + glutBitmapWidth(font,*c) + spacing;
	}
}
void DrawGrid(int GRID_SIZE)
{
	glBegin(GL_LINES);
	glColor3f(0.75f, 0.75f, 0.75f);
	for(int i=-GRID_SIZE;i<=GRID_SIZE;i++)
	{
		//along z-axis
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);
		//along x-axis
		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

void SetOrthoForFont(){
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);
	glScalef(1, -1, 1);
	glTranslatef(0, -WINDOW_HEIGHT, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void RenderSpaceBitmapString(int x, int y, int spacing, void *font, const char *string){
	const char *c; 
	int x1=x;
	for(c=string; *c != '\0'; c++){
		glRasterPos2i(x1,y);
		glutBitmapCharacter(font, *c);
		x1 = x1 + glutBitmapWidth(font,*c) +spacing;
	}
}

void ResetPerspectiveProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


void OnIdle(){
	glutPostRedisplay();
}

void OnReshape(int nw, int nh){
	glViewport(0, 0, nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 0.1f, 100.0f);
	

	
	glMatrixMode(GL_MODELVIEW);
}




void Mouse(int button, int s, int x, int y){

	
	if(s == GLUT_DOWN){
		if(button == GLUT_LEFT_BUTTON){
			//isPick = PickActor(x,y);
		}
		oldX = x;
		oldY = y;
	} 
		
	if(s == GLUT_UP){
		LetGoActor();
		isPick = false;
	}

	/*
	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;
	*/
	state = button;
}

void MotionCallback(int x, int y){
	mousePosX = x;
	mousePosY = y;
};

void Motion(int x, int y){
	
	if(isPick){
		MoveActor(x,y);
	}else{
		switch(state){
			case GLUT_MIDDLE_BUTTON:
				dist *= (1 + (y - oldY)/60.0f);
				break;
			case GLUT_LEFT_BUTTON:
				rY += (x - oldX)/10.0f;
				rX += (y - oldY)/10.0f;
				break;
			case GLUT_RIGHT_BUTTON:
				transX += (x-oldX)/60.0f;
				transY -= (y-oldY)/60.0f;
				break;
		}
	}
	/*
	if(state == 0)
		dist *= (1 + (y - oldY)/60.0f);
	else{
		rY += (x - oldX)/10.0f;
		rX += (y - oldY)/10.0f;
	}
	*/
	oldX = x;
	oldY = y;

	glutPostRedisplay(); //redisplay through glutMainLoop
}

void initGL(int argc, char** argv){
	
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("GLUT PhysX3 DEMO - Simple Box");
	
	glutDisplayFunc(Display);
	glutIdleFunc(OnIdle);
	glutReshapeFunc(OnReshape);
	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);
	glutPassiveMotionFunc(MotionCallback);
	




	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	GLfloat ambient[4]={0.25f, 0.25f, 0.25f, 0.25f};
	GLfloat diffuse[4]={1, 1, 1, 1};
	GLfloat mat_diffuse[4]={0.85f, 0, 0, 0};

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat_diffuse);

	glDisable(GL_LIGHTING);
}

void MoveActor(int x, int y){
	if(!mMouseSphere)
		return;
	PxVec3 pos;
	ViewUnProject(x, y, mMouseDepth, pos);
	mMouseSphere->setGlobalPose(PxTransform(pos));
}

void DrawDebugLine(int x1, int y1, int z1, int x2, int y2, int z2){
	GLfloat size[2];
	GLfloat increment;
	//PxVec3 v1, v2;

	//ViewUnProject(x, y, 0.0f, v1);
	//ViewUnProject(x, y, 1.0f, v2);

	//set drawing color
	glColor3f(0.0f, 1.0f, 0.0f);
	/*
	//get supported point size range and increment
	glGetFloatv(GL_POINT_SIZE_RANGE, size);
	glGetFloatv(GL_POINT_SIZE_GRANULARITY, &increment);

	//set initial point size
	glPointSize(size[0]+5*increment);
	
	//draw points
	
	glBegin(GL_POINTS);
		glVertex3f((GLfloat)x1, (GLfloat)y1, (GLfloat)z1);
		glVertex3f((GLfloat)x2, (GLfloat)y2, (GLfloat)z2);
	glEnd();
	*/
	//get supported line width range and increment
	glGetFloatv(GL_LINE_WIDTH_RANGE, size);
	glGetFloatv(GL_LINE_WIDTH_GRANULARITY, &increment);

	//set initial line width
	glLineWidth(size[0]+increment);

	//draw lines
	glBegin(GL_LINES);
		glVertex3f((GLfloat)x1, (GLfloat)y1, (GLfloat)z1);
		glVertex3f((GLfloat)x2, (GLfloat)y2, (GLfloat)z2);
	glEnd();
	//glutPostRedisplay();
}

void DrawDebugPoint(int x, int y, int z){
	GLfloat size[2];
	GLfloat increment;
	
	//set drawing color
	glColor3f(0.0f, 1.0f, 0.0f);

	//get supported point size range and increment
	glGetFloatv(GL_POINT_SIZE_RANGE, size);
	glGetFloatv(GL_POINT_SIZE_GRANULARITY, &increment);

	//set initial point size
	glPointSize(size[0]+5*increment);
	
	//draw points
	glBegin(GL_POINTS);
		glVertex3f((GLfloat)x, (GLfloat)y, (GLfloat)z);
	glEnd();

}

bool PickActor(int x, int y){
	LetGoActor();
	Ray ray;
	ViewUnProject(x, y, 0.0f, ray.orig);
	ViewUnProject(x, y, 1.0f, ray.dir);
	//DrawDebugLine(ray.orig, ray.dir);
	ray.dir -=ray.orig;
	float length = ray.dir.magnitude();
	ray.dir.normalize();

	PxRaycastHit hit;
	PxShape* closestShape;
	mScene->raycastSingle(ray.orig, ray.dir, length, PxSceneQueryFlag::eIMPACT, hit);
	closestShape = hit.shape;
	if(!closestShape) return false;
	//if(!closestShape->getActor().is()) return false;
	
	int hitx, hity;
	ViewProject(hit.impact, hitx, hity, mMouseDepth);
	mMouseSphere = CreateSphere(hit.impact, 0.1f, 1.0f);

	mMouseSphere->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);

	mSelectedActor = (PxRigidDynamic*) &closestShape->getActor();
	mSelectedActor->wakeUp();

	PxTransform mFrame, sFrame;
	mFrame.q = mMouseSphere->getGlobalPose().q;
	mFrame.p = mMouseSphere->getGlobalPose().transformInv(hit.impact);
	sFrame.q = mSelectedActor->getGlobalPose().q;
	sFrame.p = mSelectedActor->getGlobalPose().transformInv(hit.impact);

	mMouseJoint = PxDistanceJointCreate(*mPhysics,mMouseSphere, mFrame, mSelectedActor, sFrame);
	mMouseJoint->setDamping(1);
	mMouseJoint->setSpring(200);
	mMouseJoint->setMinDistance(0);
	mMouseJoint->setMaxDistance(0);
	mMouseJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
	mMouseJoint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
	return true;
}

void LetGoActor(){
	if(mMouseJoint)
		mMouseJoint->release();
	mMouseJoint = NULL;
	if(mMouseSphere)
		mMouseSphere->release();
	mMouseSphere = NULL;

}

PxRigidDynamic *CreateSphere(const PxVec3 &pos, const PxReal radius, const PxReal density){
	PxTransform transform(pos, PxQuat::createIdentity());
	PxSphereGeometry geometry(radius);

	PxMaterial *mMaterial = mPhysics->createMaterial(0.5,0.5,0.5);

	PxRigidDynamic *actor = PxCreateDynamic(*mPhysics, transform, geometry, *mMaterial, density);
	if(!actor)
		cerr<<"create actor failed!"<<endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	mScene->addActor(*actor);
	return actor;
}

void ViewProject(PxVec3 &v, int &xi, int &yi, float &depth){
	glGetIntegerv(GL_VIEWPORT, viewPort);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
	GLdouble winX, winY, winZ;
	
	gluProject((GLdouble)v.x, (GLdouble)v.y, (GLdouble)v.z, modelMatrix, projMatrix, viewPort, &winX, &winY, &winZ);
	xi = (int)winX;
	yi = viewPort[3] - (int)winY;
	depth = (float)winZ;
}




void ViewUnProject(int xi, int yi, float depth, PxVec3 &v){

	GLdouble wx, wy, wz;
	GLdouble posX, posY, posZ;
	
	wx = (float) xi;
	wy = (float) viewPort[3] - (float)yi;
	//glReadPixels((int)wx, (int)wy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &wz);
	//wz = (wz-GL_DEPTH_BIAS)/GL_DEPTH_SCALE ;
	gluUnProject(wx, wy, depth, modelMatrix, projMatrix, viewPort, &posX, &posY, &posZ);
	v = PxVec3((PxReal)posX, (PxReal)posY, (PxReal)posZ);
}


void PxVirtualDebugger(){
	if(mPhysics->getPvdConnectionManager() == NULL)
		return;
	const char* pvd_host_ip = "127.0.0.1";
	int port = 5425;
	unsigned timeout = 100;
	PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();
	PVD::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(mPhysics->getPvdConnectionManager(),
    pvd_host_ip, port, timeout, connectionFlags);
	if(theConnection)
		theConnection->release();

}


void main(int argc, char** argv){
	atexit(OnShutdown);
	initGL(argc, argv);

	initPhysX();
	initScene();
	CreatePlane();
	CreateCube();
	PxVirtualDebugger();

	glutMainLoop();

	//system("PAUSE");

	//return 0;
}

