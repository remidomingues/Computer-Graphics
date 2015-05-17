#ifndef RAYTRACING_H
#define RAYTRACING_H

#include <glm/glm.hpp>
#include <SDL.h>
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::mat3;

/* Intersection between a ray and an object */
struct Intersection {
	vec3 position;
	float distance;
	int objectIndex;
};

/* 3D axis direction based on the camera point of view */
typedef enum Direction {
	RIGHT,
	LEFT,
	FORWARD,
	BACKWARD,
	UP,
	DOWN
};

/* Light source distribution */
typedef enum LightDistribution {
	Uniform = 0,
	Jittered = 1 // Recommended
};

/* Anti-aliasing */
typedef enum AntiAliasing {
	Disabled = 2,
	Uniform8x = 3,
	StochasticSampling2x = 4,
	StochasticSampling4x = 5,
	StochasticSampling8x = 6,
	StochasticSampling16x = 7,
	StochasticSampling64x = 8
};


// ----------------------------------------------------------------------------
// FUNCTIONS
void antiAliasing();
void antiAliasingOnPixel(vec3** screenPixels, const pair<int, int>& p, int rank);
bool closestIntersection(vec3 start, vec3 dir, int startObjIdx, Intersection& closestIntersection);
void computePixelsIntensity();
vec3 directLight(const Intersection& i, int rank);
void display();
void displayConfig();
void draw();
vec3 getAxis(Direction dir);
bool getColor(const vec3& start, const vec3& d_ray, int startObjIdx, vec3& color, int bounce, float refractiveIndice, float weight, int rank);
float getDistance(vec3 p1, vec3 p2);
float getElapsedTime();
bool getLightPower(const Intersection& i, const vec3& lightPos, float lightDistance, vec3 &r, vec3& power);
void getReflectedDirection(const vec3& incident, const vec3& normal, vec3& reflected);
bool getReflectionRefractionDirections(float i1, float i2, const vec3& incident, vec3& normal, vec3& reflected, vec3& refracted, float* refractionPercentage);
void initLights();
void initLightSurface();
void postProcess();
void process(vec3** screenPixels, int width, int height, int widthOffset, int heightOffset, int threadRank);
float sobelOperator(int x, int y);
vec3 stochasticSampling(int action, int pixelx, int pixely, int rank);
vec3 supersamplingAA(int x, int y);
vec3 traceRayFromCamera(float x, float y, int rank);
void update();
void updateRotationMatrix();

#endif
