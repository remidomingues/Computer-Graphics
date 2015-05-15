#ifndef RAYTRACER_H
#define RAYTRACER_H

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
*		- Supersampling 8x (uniform)
*		- Stochastic sampling 2x, 4x, 8x, 16x, 64x (jittered)
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
	Supersampling8x = 3,
	StochasticSampling2x = 4,
	StochasticSampling4x = 5,
	StochasticSampling8x = 6,
	StochasticSampling16x = 7,
	StochasticSampling64x = 8
};


// ----------------------------------------------------------------------------
// RENDERING PARAMETERS
// If true, render the image, export it in a .bmp file then exit
const bool EXPORT_AND_EXIT = true;

// Rendering resolution
const int SCREEN_WIDTH = 4096;
const int SCREEN_HEIGHT = 4096;

// Light distribution
LightDistribution LIGHTS_DISTRIBUTION = LightDistribution::Jittered;

// Number of sampled lights will be LIGHT_ROWS * LIGHT_COLS
int LIGHT_ROWS = 16;
int LIGHT_COLS = 16;

// Maximum number of light bounces by refraction and reflection
const int MAX_BOUNCES = 20;

// Anti aliasing
const AntiAliasing ANTI_ALIASING = AntiAliasing::StochasticSampling16x;


// ----------------------------------------------------------------------------
// CONSTANTS
/* Camera */
// Focal length
const float FOCAL_LENGTH = SCREEN_WIDTH;
// Rotation angle added for each rotation
const float ROTATION = 0.125;
// Translation axis unit percentage added for each translation
const float TRANSLATION = 0.5;

/* Light */
const vec3 LIGHT_CENTER(0, -1.1, 0);
vec3 LIGHT_NORMAL;
const float LIGHT_SURFACE_DIAMETER = 1;
const float LIGHT_DIAMETER = 0.3;

/* Anti-aliasing */
const float SOBEL_THRESHOLD = 0.5;
const vec3 INTENSITY_WEIGHTS(0.2989, 0.5870, 0.1140);

/* Materials */
float AIR_REFRACTIVE_INDEX = 1.0;
float GLASS_REFRACTIVE_INDEX = 1.52;
float DIFFUSE_SPECULAR_REFLECTION = 0.18;

/* Colors */
const vec3 SHADOW_COLOR = 0.0f * vec3(1, 1, 1);
const vec3 VOID_COLOR(0.75, 0.75, 0.75);


// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
/* Screen */
SDL_Surface* screen;
vec3 screenPixels[SCREEN_HEIGHT][SCREEN_WIDTH];

/* Model */
vector<Object3D*> objects;

/* Camera */
// Position
vec3 cameraPos(0, 0, -3);
// Rotation matrix
mat3 R = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
// Rotation angle
float yaw = 0;

/* Light */
vector<vec3> lightPositions;
pair<int, int> lightIdx;
const vec3 lightColor = 14.f * vec3(1, 1, 1);
const vec3 indirectLight = 0.55f * vec3(1, 1, 1);
const int NOT_STARTOBJIDX = -1;

/* Anti-aliasing */
float screenPixelsIntensity[SCREEN_HEIGHT][SCREEN_WIDTH];


// ----------------------------------------------------------------------------
// FUNCTIONS
void antiAliasing();
bool closestIntersection(vec3 start, vec3 dir, int startObjIdx, Intersection& closestIntersection);
void computePixelsIntensity();
vec3 directLight(const Intersection& i);
void draw();
vec3 getAxis(Direction dir);
bool getColor(const vec3& start, const vec3& d_ray, int startObjIdx, vec3& color, int bounce, float refractiveIndice);
float getDistance(vec3 p1, vec3 p2);
bool getLightPower(const Intersection& i, const vec3& lightPos, float lightDistance, vec3 &r, vec3& power);
void getReflectedDirection(const vec3& incident, const vec3& normal, vec3& reflected);
bool getReflectionRefractionDirections(float i1, float i2, const vec3& incident, vec3& normal, vec3& reflected, vec3& refracted, float* refractionPercentage);
void initLights();
float sobelOperator(int x, int y);
vec3 stochasticSampling(int action, int pixelx, int pixely);
vec3 supersamplingAA(int x, int y);
vec3 traceRayFromCamera(float x, float y);
void update();
void updateRotationMatrix();

#endif
