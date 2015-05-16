#ifndef RAYTRACING_H
#define RAYTRACING_H

#include <glm/glm.hpp>
#include <SDL.h>
#include "TestModel.h"

/* Raytracer supporting the following features:
* - Objects: triangles and spheres
* - Materials:
*		- Diffuse (absorption)
*		- Specular (reflection)
*		- Diffuse and specular (absorption and reflection)
*		- Glass (refraction and reflection)
* - Direct illumination + constant indirect illumination
* - Soft shadows (NxN light sources) according to the following light sources distribution:
*		- Uniform
*		- Jittered by stochastic sampling
* - Anti-aliasing:
*		- Uniform 8x
*		- Jittered by stochastic sampling 2x, 4x, 8x, 16x, 64x
*
* The most important constants to tweak are:
*	- EXPORT_AND_EXIT
*	- SCREEN_WIDTH and SCREEN_HEIGHT
*	- LIGHT_ROWS and LIGHT_COLS
*	- ANTI_ALIASING
*/

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
