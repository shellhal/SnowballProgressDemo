#include "particle.h"
#include <cstdlib>

particle::particle(){
	x = 0;
	y = 0;
	z = 0;
	radius = 1.;
	mass = 0.0001;
	binding_coefficient = 1;
	outermost = false;
}
particle::particle(float x, float y, float z){
	this->x = x;
	this->y = y;
	this->z = z;
	radius = 1.;
	mass = 0.0001;
	binding_coefficient = 1;
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