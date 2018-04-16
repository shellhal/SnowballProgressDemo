#ifndef PARTICLE_H
#define PARTICLE_H


struct info{
	float center[3];
	int num_particles;
	float color[3];
};

class particle{
public:
	float x, y, z;
	float velocity[3];
	bool outermost;
	float binding_coefficient;
	float radius;
	float mass;	

	particle();
	particle(float x, float y, float z);
	void moveTo(float x, float y, float z);
	void setVelocity(float xn, float yn, float zn);

};

#endif

