/********************************************************************
* Program Name: Particle Sphere Examples
* Author: Lily Shellhammer
* Date: Jan 2nd, 2018
* Description: 
********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef WIN32
#include <windows.h>
#pragma warning(disable:4996)
#include "glew.h"
#endif


#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <stdio.h>
#include <cstdlib>
#include <vector>


#include "particle.h"
/****************************************************************
*  CONSTANTS AND #DEFINES
*****************************************************************/
//int numParticles = 20;

int dimensions = 10; //diameter of particle sphere
bool testNoise = false; //whether you want messy snowballs or not
float rate = 5.; //what you divide by to slow down initial velocities
bool direct_collision = false; //whether you want snowballs to hit fly 
							   //directly at each other or your own choice of speeds
int count = 0;
float kElastic = 0;
float kDampening = 0;
float radius = 1.;
float mass = 1.;

bool inelastic = false;
float inelasticPercent = .58;
bool fakeInelastic = true;//true;


#define ESCAPE		0x1b
#define MS_IN_THE_ANIMATION_CYCLE	10000
const GLfloat BACKCOLOR[ ] = { 0., 0., 0., 1. };
const int INIT_WINDOW_SIZE = { 600 };
const float MINSCALE = { 0.05f };

const float GRAVITY = -9.81;

// what the glui package defines as true and false:
const int GLUITRUE  = { true  };
const int GLUIFALSE = { false };

// multiplication factors for input interaction:
//  (these are known from previous experience)
const float ANGFACT = { 1. };
const float SCLFACT = { 0.005f };

// active mouse buttons (or them together):
const int LEFT   = { 4 };
const int MIDDLE = { 2 };
const int RIGHT  = { 1 };

// which button:
enum ButtonVals
{
	RESET,
	QUIT
};


/**************************************************************** 
* Non CONST Global variables
*****************************************************************/
int		ActiveButton;			// current button that is down
// Declaration of vector of particles
std::vector<particle> particles;
info particlesInfo;
std::vector<particle> particlesB;
info particlesBInfo;

int		MainWindow;				// window id for main graphics window
float	Scale;					// scaling factor
float	Time;
int		Xmouse, Ymouse;			// mouse values
float	Xrot, Yrot;				// rotation angles in degrees
/**************************************************************** 
* PROTOTYPES
*****************************************************************/
//Basics
void	Animate( );
void	Display( );
void	InitGraphics( );
void	InitLists( );
void	Keyboard( unsigned char, int, int );
void	MouseButton( int, int, int, int );
void	MouseMotion( int, int );

void	Reset( );
float	ElapsedSeconds( );

//functions specific to this assignment
void initParticles();
void initVelocities();
void updateAll();
void initScene();
bool dist(particle a, particle b);
void initColors();
void color(particle* a, particle *b);
//float dist(particle a, particle b);

/****************************************
* Function Name: Main
* Description: 
*****************************************/
int main( int argc, char *argv[ ] )
{
	glutInit( &argc, argv );
	InitGraphics( );
	//create all the points
	initScene();
	initParticles();
	initColors();
	//set all values to beginning
	Reset();

	glutSetWindow( MainWindow );
	glutMainLoop( );

	return 0;
}

float quadratic(int v1i, int v2i){
	float a = (-2.*(v1i - v2i) + sqrt(pow((2.*v1i - 2.*v2i),2) - 8.*v1i*v2i))/2.;
	float b = (-2.*(v1i - v2i) - sqrt(pow((2.*v1i - 2.*v2i),2) - 8.*v1i*v2i))/2.;
	fprintf(stderr, "A is: %d\tB is: %d\n", a, b);
	return a;
}

float otherhalf(int v1i, int v2i, int a){
	return (v1i + v2i - a);
}
/****************************************
* Function Name: Velocity after collision
* Description: calculates velocity after elastic or inelastic collision
*****************************************/
void velocity_after_collision_calculate(particle* a, particle *b){
	float m1 = mass;
	float m2 = mass;
	float v1[3];
	float v2[3];
	float v1Holder[3];
	float v2Holder[3];
	v1[0] = a->velocity[0];
	v1[1] = a->velocity[1];
	v1[2] = a->velocity[2];
	v1Holder[0] = a->velocity[0];
	v1Holder[1] = a->velocity[1];
	v1Holder[2] = a->velocity[2];

	v2[0] = b->velocity[0];
	v2[1] = b->velocity[1];
	v2[2] = b->velocity[2];
	v2Holder[0] = b->velocity[0];
	v2Holder[1] = b->velocity[1];
	v2Holder[2] = b->velocity[2];

	if(inelastic){
		v1Holder[0] = v1[0]* ((m1-m2)/(m1+m2)) + v2[0]*((2.*m2)/m1+m2);
		v2Holder[0] = v1[0]*(2.*m1/(m1+m2))+ v2[0]*((m2- m1)/(m1+m2));

		v1Holder[1] = v1[1]* ((m1-m2)/(m1+m2)) + v2[1]*((2.*m2)/m1+m2);
		v2Holder[1] = v1[1]*(2.*m1/(m1+m2))+ v2[1]*((m2- m1)/(m1+m2));

		v1Holder[2] = v1[2]* ((m1-m2)/(m1+m2)) + v2[2]*((2.*m2)/m1+m2);
		v2Holder[2] = v1[2]*(2.*m1/(m1+m2))+ v2[2]*((m2- m1)/(m1+m2));


		b->velocity[0] = quadratic(v1Holder[0], v2Holder[0]);
		a->velocity[0] = otherhalf(v1Holder[0], v2Holder[0], b->velocity[0]);

		b->velocity[1] = quadratic(v1Holder[1], v2Holder[1]);
		a->velocity[1] = otherhalf(v1Holder[1], v2Holder[1], b->velocity[1]);

		b->velocity[2] = quadratic(v1Holder[2], v2Holder[2]);
		a->velocity[2] = otherhalf(v1Holder[2], v2Holder[2], b->velocity[2]);
		
		// a->velocity[1] = -1*sqrt(2*pow(v2Holder[1],2) - (2*pow(v2Holder[1],2)*inelasticPercent));
		// b->velocity[1] = sqrt(2*pow(v2Holder[1],2) - (2*pow(v2Holder[1],2)*inelasticPercent));
		

		// a->velocity[2] = sqrt(2*pow(v2Holder[2],2) - (2*pow(v2Holder[2],2)*inelasticPercent));
		// b->velocity[2] = sqrt(2*pow(v2Holder[2],2) - (2*pow(v2Holder[2],2)*inelasticPercent));
	}
	else if(fakeInelastic){
		a->velocity[0] = (v1[0]* ((m1-m2)/(m1+m2)) + v2[0]*((2.*m2)/m1+m2)) * inelasticPercent;
		b->velocity[0] = (v1[0]*(2.*m1/(m1+m2))+ v2[0]*((m2- m1)/(m1+m2))) * inelasticPercent;

		a->velocity[1] = (v1[1]* ((m1-m2)/(m1+m2)) + v2[1]*((2.*m2)/m1+m2)) * inelasticPercent;
		b->velocity[1] = (v1[1]*(2.*m1/(m1+m2))+ v2[1]*((m2- m1)/(m1+m2))) * inelasticPercent;

		a->velocity[2] = (v1[2]* ((m1-m2)/(m1+m2)) + v2[2]*((2.*m2)/m1+m2)) * inelasticPercent;
		b->velocity[2] = (v1[2]*(2.*m1/(m1+m2))+ v2[2]*((m2- m1)/(m1+m2)))* inelasticPercent;
	}
	else{
		a->velocity[0] = (v1[0]* ((m1-m2)/(m1+m2)) + v2[0]*((2.*m2)/m1+m2));
		b->velocity[0] = (v1[0]*(2.*m1/(m1+m2))+ v2[0]*((m2- m1)/(m1+m2)));

		a->velocity[1] = (v1[1]* ((m1-m2)/(m1+m2)) + v2[1]*((2.*m2)/m1+m2));
		b->velocity[1] = (v1[1]*(2.*m1/(m1+m2))+ v2[1]*((m2- m1)/(m1+m2)));

		a->velocity[2] = (v1[2]* ((m1-m2)/(m1+m2)) + v2[2]*((2.*m2)/m1+m2));
		b->velocity[2] = (v1[2]*(2.*m1/(m1+m2))+ v2[2]*((m2- m1)/(m1+m2)));
	}
}


/****************************************
* Function Name: dist
* Description: distance between two particles, including radius 
*****************************************/
bool dist(particle a, particle b){
	//return pow( (pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z -b.z), 2)), 0.5);
	if(a.x - b.x <= radius && a.x - b.x > -1.*radius)
		if(a.y - b.y <= radius && a.y - b.y > -1.*radius)
			if(a.z - b.z <= radius && a.z - b.z > -1.*radius)
				return true;
	return false;
}

/****************************************
* Function Name: color
* Description: adds color when a collision occurs
*****************************************/
void color(particle *a, particle *b){
	if(a->color[1] > 0.){
		a->color[1] -= .005;
		if(a->color[2] > 0.)
			a->color[2] -= .001;
	}

	if(b->color[1] > 0.){
		b->color[1] -= .01;
		if(b->color[2] > 0.)
			b->color[2] -= .02;
	}
}
/****************************************
* Function Name: checkCollisions
* Description: 
*****************************************/
void checkCollisions(){
	//compare every particle to every other particle
	for(int i = 0; i < particles.size(); i++){
		for(int j = i; j < particlesB.size(); j++){
			if(dist(particles[i], particlesB[j])) {//<= (particles[i].radius + particlesB[j].radius)+.1){
					
				velocity_after_collision_calculate(&particles[i], &particlesB[j]);
				color(&particles[i], &particlesB[j]);
				count++;
				//fprintf( stderr, "COLLISION DETECTED between particles %d and %d\n", i, j);

			}
		}
		for(int j = i; j < particles.size(); j++){
			if(i!= j){
				if(dist(particles[i], particles[j])){// <= (particles[i].radius + particles[j].radius)+.1){

					velocity_after_collision_calculate(&particles[i], &particles[j]);
					color(&particles[i], &particles[j]);
					count++;
					//fprintf( stderr, "COLLISION DETECTED between particles %d and %d\n", i, j);

					
				}
			}
		}
	}
	for(int i = 0; i < particlesB.size(); i++){
		for(int j = i; j < particlesB.size(); j++){
			if(i!= j){
				if(dist(particlesB[i], particlesB[j])){// <= (particlesB[i].radius + particlesB[j].radius)+.1){
						
					velocity_after_collision_calculate(&particlesB[i], &particlesB[j]);
					color(&particlesB[i], &particlesB[j]);
					//fprintf( stderr, "COLLISION DETECTED between particles %d and %d\n", i, j);
					count++;
				}
			}
		}
	}
	//fprintf(stderr, "Count is: %d\t\t", count);
}


/****************************************
* Function Name: initDistance
* Description: A distance funciton specifically used for when we are creating the snowball spheres
*****************************************/
float initDistance(int h, int w, int d, int radius){
	int x = h - radius;
	int y = w - radius;
	int z = d - radius;
	float distance = pow( (pow(x, 2) + pow(y, 2) + pow(z, 2)), 0.5);
	return distance;
}


/****************************************
* Function Name: start
* Description: Arrange particle arrays A and B into balls
*****************************************/
void initScene(){
	/* set each snow balls color*/
	// particlesBInfo.color[0] = .6;
	// particlesBInfo.color[1] = .8;
	// particlesBInfo.color[2] = 1.;

	/* set each snow balls starting point*/
	particlesInfo.center[0] = 10.;
	particlesInfo.center[1] = 0.;
	particlesInfo.center[2] = 0.;

	particlesBInfo.center[0] = -10.;
	particlesBInfo.center[1] = 0.;
	particlesBInfo.center[2] = 0.;

}

void initColors(){
	for(int i = 0; i < particles.size(); i++){
		particles[i].color[0] = 1.;
		particles[i].color[1] = 1.;
		particles[i].color[2] = 1.;
	}
	// particlesInfo.color[0] = 1.;
	// particlesInfo.color[1] = 1.;
	// particlesInfo.color[2] = 1.;

	for(int i = 0; i < particles.size(); i++){
		particlesB[i].color[0] = 1.;
		particlesB[i].color[1] = 1.;
		particlesB[i].color[2] = 1.;
	}
}
/****************************************
* Function Name: initParticles
* Description: create particles in vector 
*****************************************/
void initParticles(){


	//testing noise to make snowball less square
	float noise[3]; 

	// make sure dimensions are even
	if(dimensions %2 == 1){
		dimensions = dimensions -1;
	}
	// set radius
	int radius =  dimensions/2.;

	// make iterator
	std::vector<particle>::iterator iter;
	bool outermost;
	// for all dimensions, draw particles that lie within sphere radius
	for(int h = 0; h < dimensions; h++){	 // iterate through height, then width, then depth
		for(int w = 0; w < dimensions; w++){
			for(int d = 0; d < dimensions; d++){
				if( initDistance(h, w, d, radius) <= radius){	 // if we are less than or equal to radius, we are inside circle
					iter = particles.begin();						 // then draw a point at that h,w,d place
					
					//BUG!!!
						outermost = true;
					// if we are testing noise, add a little noise to the snowball's rigid lines
					if(testNoise){
						noise[0] = (rand()%2)* (-1) + (rand()%10)/10.;
						noise[1] = (rand()%2)* (-1) + (rand()%10)/10.;
						noise[2] = (rand()%2)* (-1) + (rand()%10)/10.;
						particles.insert(iter, particle( (h + particlesInfo.center[0] + noise[0]), (w + particlesInfo.center[1] + noise[1]), (d + particlesInfo.center[2] + noise[2]), outermost));
					}
					else
						particles.insert(iter, particle( (h + particlesInfo.center[0]), (w + particlesInfo.center[1]), (d + particlesInfo.center[2]), outermost));
					
				}
			}
		}
	}
	//BUG!!!
		outermost = true;
	// do the same for sphere B
	for(int h = 0; h < dimensions; h++){	 // iterate through height, then width, then depth
		for(int w = 0; w < dimensions; w++){
			for(int d = 0; d < dimensions; d++){
				if( initDistance(h, w, d, radius) <= radius){	 // if we are less than or equal to radius, we are inside circle
					iter = particlesB.begin();						 // then draw a point at that h,w,d place
					if(testNoise){
						noise[0] = (rand()%2)* (-1) + (rand()%10)/10.;
						noise[1] = (rand()%2)* (-1) + (rand()%10)/10.;
						noise[2] = (rand()%2)* (-1) + (rand()%10)/10.;
						particlesB.insert(iter, particle( (h + particlesBInfo.center[0] + noise[0]), (w + particlesBInfo.center[1] + noise[1]), (d + particlesBInfo.center[2] + noise[2]), outermost));
					}
					else
						particlesB.insert(iter, particle( (h + particlesBInfo.center[0]), (w + particlesBInfo.center[1]), (d + particlesBInfo.center[2]), outermost));
					
				}
			}
		}
	}

	initVelocities();
}

/****************************************
* Function Name: initVelocities
* Description: start the snowballs trajectory
		**** DISCLAIMER ****
			This only has snowballs fly DIRECTLY at each other. They don't intersect at an angle yet
*****************************************/
void initVelocities(){
	float speed[3];
	if(direct_collision){	//head directly toward one another
		speed[0] = particlesBInfo.center[0] - particlesInfo.center[0];
		speed[1] = particlesBInfo.center[1] - particlesInfo.center[1];
		speed[2] = particlesBInfo.center[2] - particlesInfo.center[2];
	}
	else{
		speed[0] = -1;
		speed[1] = 1;
		speed[2] = 0;
	}

	for(int i = 0; i < particles.size(); i++){
		particles[i].velocity[0] = speed[0] / rate;
		particles[i].velocity[1] = speed[1] / rate;
		particles[i].velocity[2] = speed[2] / rate;
	}

	float speedB[3];
	if(direct_collision){
		speedB[0] = particlesInfo.center[0] - particlesBInfo.center[0];
		speedB[1] = particlesInfo.center[1] - particlesBInfo.center[1];
		speedB[2] = particlesInfo.center[2] - particlesBInfo.center[2];
	}
	else{
		speedB[0] = 1;
		speedB[1] = 1;
		speedB[2] = 0;
	}
	for(int i = 0; i < particles.size(); i++){
		particlesB[i].velocity[0] = speedB[0] / rate;
		particlesB[i].velocity[1] = speedB[1] / rate;
		particlesB[i].velocity[2] = speedB[2] / rate;
	}
}


void updateSpringForce(){

}

/****************************************
* Function Name: updateAll
* Description: Update positions of particles
*****************************************/
void updateAll(){
	checkCollisions();
	updateSpringForce();
	for(int i = 0; i < particles.size(); i++){

		//Calculate gravity
		particles[i].velocity[1] += 1./100.0*GRAVITY;
		//Update position
		particles[i].x += particles[i].velocity[0]*1./10.0;
		particles[i].y += particles[i].velocity[1]*1./10.0;
		particles[i].z += particles[i].velocity[2]*1./10.0;


	}
	for(int i = 0; i < particlesB.size(); i++){
		particlesB[i].velocity[1] += 1./100.0*GRAVITY;

		particlesB[i].x += particlesB[i].velocity[0]*1./10.0;
		particlesB[i].y += particlesB[i].velocity[1]*1./10.0;
		particlesB[i].z += particlesB[i].velocity[2]*1./10.0;
	}
}

/****************************************
* Function Name: Animate
* Description: Everytime glut main loop has nothing to do
*****************************************/
void Animate( )
{
	int ms = glutGet( GLUT_ELAPSED_TIME );	// milliseconds
	ms  %=  MS_IN_THE_ANIMATION_CYCLE;
	Time = (float)ms  /  (float)MS_IN_THE_ANIMATION_CYCLE; 
	//Put animation changes here
	//Let force display call happen:
	updateAll();

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}

/****************************************
* Function Name: Display
* Description: Show everything
*****************************************/
void Display( )
{
	//start over animation if its been the whole length
	/*if(ElapsedSeconds()-10 <= 0.0001 && ElapsedSeconds()-10 > 0){
		fprintf( stdout, "Starting over simulation\n");
		Reset();
	}*/

	glutSetWindow( MainWindow );
	// erase background
	glDrawBuffer( GL_BACK );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable( GL_DEPTH_TEST );

	glShadeModel( GL_FLAT );

	// Set the viewport to a square centered in the window:
	GLsizei vx = glutGet( GLUT_WINDOW_WIDTH );
	GLsizei vy = glutGet( GLUT_WINDOW_HEIGHT );
	GLsizei v = vx < vy ? vx : vy;			// minimum dimension
	GLint xl = ( vx - v ) / 2;
	GLint yb = ( vy - v ) / 2;
	glViewport( xl, yb,  v, v );


	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity( );

	gluPerspective( 90., 1.,	0.1, 1000. );
	//float height = float(numParticles)/2.0;
	gluLookAt( 0., 0., 25.,     5., 5, 5.,     0., 1., 0. );


	//Rotate and scale scene
	glRotatef( (GLfloat)Yrot, 0., 1., 0. );
	glRotatef( (GLfloat)Xrot, 1., 0., 0. );

	if( Scale < MINSCALE )
		Scale = MINSCALE;
	glScalef( (GLfloat)Scale, (GLfloat)Scale, (GLfloat)Scale );


	//update the particles then draw them
	
	//fprintf( stdout, "Num particles: %d\n", particles.size());
	/*fprintf(stdout, "Particle 0 position: %.1f, %.1f, %.1f\n", particles[0].x, particles[0].y, particles[0].z);
	fprintf(stdout, "Particle 1 position: %.1f, %.1f, %.1f\n", particles[1].x, particles[1].y, particles[1].z);
	*/
	for(int j = 0; j < particles.size(); j++){
		glPushMatrix();
		glColor3f(particles[j].color[0], particles[j].color[1], particles[j].color[2]);
		glTranslatef(particles[j].x, particles[j].y, particles[j].z);
		glutSolidCube(radius);
		glPopMatrix();
	}
	for(int j = 0; j < particlesB.size(); j++){
		glPushMatrix();
		glColor3f(particlesB[j].color[0], particlesB[j].color[1], particlesB[j].color[2]);
		glTranslatef(particlesB[j].x, particlesB[j].y, particlesB[j].z);
		glutSolidCube(radius);
		glPopMatrix();
	}

	glEnable( GL_NORMALIZE );


	glutSwapBuffers( );
	glFlush( );
}
/****************************************
* Function Name: InitGraphics
* Description: init glut and OpenGl also set up display lists and callback functions
*****************************************/
void InitGraphics( )
{
	// request the display modes:
	// ask for red-green-blue-alpha color, double-buffering, and z-buffering:
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );

	// set the initial window configuration:
	glutInitWindowPosition( 0, 0 );
	glutInitWindowSize( INIT_WINDOW_SIZE, INIT_WINDOW_SIZE );

	// open the window and set its title:
	MainWindow = glutCreateWindow( "Particles" );
	glutSetWindowTitle( "Particles" );

	// set the framebuffer clear values:
	glClearColor( BACKCOLOR[0], BACKCOLOR[1], BACKCOLOR[2], BACKCOLOR[3] );

	glutSetWindow( MainWindow );
	glutDisplayFunc( Display );
	glutKeyboardFunc( Keyboard );
	glutMouseFunc( MouseButton );
	glutMotionFunc( MouseMotion );
	glutPassiveMotionFunc( NULL );
	glutEntryFunc( NULL );
	glutSpecialFunc( NULL );
	glutSpaceballMotionFunc( NULL );
	glutSpaceballRotateFunc( NULL );
	glutSpaceballButtonFunc( NULL );
	glutButtonBoxFunc( NULL );
	glutDialsFunc( NULL );
	glutTabletMotionFunc( NULL );
	glutTabletButtonFunc( NULL );
	glutMenuStateFunc( NULL );
	glutTimerFunc( -1, NULL, 0 );
	glutIdleFunc( Animate );

	// init glew (a window must be open to do this):

#ifdef WIN32
	GLenum err = glewInit( );
	if( err != GLEW_OK )
	{
		fprintf( stderr, "glewInit Error\n" );
	}
	else
		fprintf( stderr, "GLEW initialized OK\n" );
	fprintf( stderr, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif

}
/****************************************
* Function Name: KeyBoard
* Description: 
*****************************************/
void Keyboard( unsigned char c, int x, int y )
{
	switch( c )
	{
		case 'q':
		case 'Q':
		case ESCAPE:
			glutSetWindow( MainWindow );
			glFinish( );
			glutDestroyWindow( MainWindow );
			exit( 0 );
			break;				// happy compiler

		default:
			fprintf( stderr, "Don't know what to do with keyboard hit: '%c' (0x%0x)\n", c, c );
	}
	// force a call to Display( ):
	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}

/****************************************
* Function Name: Reset
* Description: Reset transformations and colors, globla variables set
*****************************************/
void Reset( )
{
	ActiveButton = 0;
	Scale  = 1.0;
	Xrot = Yrot = 0.;
	//start();
}


/****************************************
* Function Name: Elapsed Seconds
* Description: 
*****************************************/
float ElapsedSeconds( )
{
	// get # of milliseconds since the start of the program:
	int ms = glutGet( GLUT_ELAPSED_TIME );

	// convert it to seconds:
	return (float)ms / 1000.f;
}

/****************************************
* Function Name: MouseButton
* Description: When mouse button transitions up or down
*****************************************/
void MouseButton( int button, int state, int x, int y )
{
	int b = 0;			// LEFT, MIDDLE, or RIGHT

	// get the proper button bit mask:

	switch( button )
	{
		case GLUT_LEFT_BUTTON:
			b = LEFT;		break;

		case GLUT_MIDDLE_BUTTON:
			b = MIDDLE;		break;

		case GLUT_RIGHT_BUTTON:
			b = RIGHT;		break;

		default:
			b = 0;
			fprintf( stderr, "Unknown mouse button: %d\n", button );
	}


	// button down sets the bit, up clears the bit:
	if( state == GLUT_DOWN )
	{
		Xmouse = x;
		Ymouse = y;
		ActiveButton |= b;		// set the proper bit
	}
	else
	{
		ActiveButton &= ~b;		// clear the proper bit
	}
}


/****************************************
* Function Name: MouseMotion
* Description: called when the mouse moves while a button is down:
*****************************************/
void MouseMotion( int x, int y )
{
	
	int dx = x - Xmouse;		// change in mouse coords
	int dy = y - Ymouse;

	if( ( ActiveButton & LEFT ) != 0 )
	{
		Xrot += ( ANGFACT*dy );
		Yrot += ( ANGFACT*dx );
	}

	if( ( ActiveButton & MIDDLE ) != 0 )
	{
		Scale += SCLFACT * (float) ( dx - dy );
		// keep object from turning inside-out or disappearing:
		if( Scale < MINSCALE )
			Scale = MINSCALE;
	}

	Xmouse = x;			// new current position
	Ymouse = y;

	glutSetWindow( MainWindow );
	glutPostRedisplay( );
}


