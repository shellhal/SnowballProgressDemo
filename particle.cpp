#include "particle.h"
#include <cstdlib>

particle::particle(){
	x = 0;
	y = 0;
	z = 0;
	outermost = false;
}
particle::particle(float x, float y, float z, bool outermost){// int n1 = -1, int n2 = -1, int n3 = -1, int n4 = -1, int n5 = -1, int n6 = -1){
	this->x = x;
	this->y = y;
	this->z = z;
	this->outermost = outermost;
	color[0] = 1.;
	color[1] = 1.;
	color[2] = 1.;
	neighbors_size = 0;
}

void particle::moveTo(float x, float y, float z){
	this->x = x;
	this->y = y;
	this->z = z;
}

void particle::setVelocity(float xn, float yn, float zn){
	x = xn;
	y = yn;
	z = zn;
}