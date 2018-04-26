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
	int neighbors[6];
	int neighbors_size;
	float color[3];

	particle();
	particle(float x, float y, float z, bool outermost);
	void moveTo(float x, float y, float z);
	void setVelocity(float xn, float yn, float zn);

};

#endif

