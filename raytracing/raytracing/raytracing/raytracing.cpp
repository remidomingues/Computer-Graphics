#define _USE_MATH_DEFINES

#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include <math.h>
#include <list>
#include <time.h>
#include <algorithm>
#include "SDLauxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::mat3;

struct Intersection
{
	vec3 position;
	float distance;
	int objectIndex;
};

typedef enum { RIGHT, LEFT, FORWARD, BACKWARD, UP, DOWN } Direction;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
const bool EXPORT_AND_EXIT = true;
const vec3 SHADOW_COLOR = 0.0f * vec3(1, 1, 1);
const vec3 VOID_COLOR(0.75, 0.75, 0.75);

/* Screen */
const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;
vec3 screenPixels[SCREEN_HEIGHT][SCREEN_WIDTH];

/* Model */
vector<Object3D*> objects;

/* Camera */
// Focal length
const float focalLength = SCREEN_WIDTH;
// Position
vec3 cameraPos(0, 0, -3);
// Rotation matrix
mat3 R = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
// Current rotation angle
float yaw = 0;
// Rotation constant - Angle update on the y axis for a rotation
const float ROTATION = 0.125;
const float TRANSLATION = 0.5;

/* Light */
typedef enum LightDistribution {
	Uniform,
	Jittered
};
LightDistribution LIGHT_DISTRIBUTION = LightDistribution::Uniform;
const vec3 LIGHT_CENTER(0, -1.1, 0);
vec3 LIGHT_NORMAL;
// Number of sampled lights will be LIGHT_ROWS * LIGHT_COLS
int LIGHT_ROWS = 16;
int LIGHT_COLS = 16;
const float LIGHT_SURFACE_DIAMETER = 1;
const float LIGHT_DIAMETER = 0.3;

vector<vec3> lightPositions;
pair<int, int> lightIdx;
const vec3 lightColor = 14.f * vec3(1, 1, 1);
const vec3 indirectLight = 0.55f * vec3(1, 1, 1);
const int MAX_BOUNCES = 20;
const int NOT_STARTOBJIDX = -1;

/* Anti-aliasing*/
typedef enum AntiAliasing {
	Disabled,
	Supersampling8x,
	StochasticSampling2x,
	StochasticSampling4x,
	StochasticSampling8x,
	StochasticSampling16x
};
/* Constants*/
const float SOBEL_THRESHOLD = 0.5;
const vec3 INTENSITY_WEIGHTS(0.2989, 0.5870, 0.1140);
const AntiAliasing ANTI_ALIASING = AntiAliasing::Disabled;
float screenPixelsIntensity[SCREEN_HEIGHT][SCREEN_WIDTH];

// ----------------------------------------------------------------------------
// FUNCTIONS

void initLights();
void UpdateRotationMatrix();
bool ClosestIntersection(vec3 start, vec3 dir, int startObjIdx, Intersection& closestIntersection);
void Update();
void Draw();
vec3 GetAxis(Direction dir);
float GetDistance(vec3 p1, vec3 p2);
vec3 DirectLight(const Intersection& i);
vec3 traceRayFromCamera(float x, float y);

int main(int argc, char* argv[])
{
	srand(time(NULL));
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
	R[1][1] = 1;
	UpdateRotationMatrix();
	int time_ms = SDL_GetTicks(), t2, dt; // Timer

	// Initialize the model
	LoadTestModel(objects);
	// Initialize the light sources and the light surface
	initLights();

	while (NoQuitMessageSDL())
	{
		Update();
		Draw();

		t2 = SDL_GetTicks();
		dt = float(t2 - time_ms);
		time_ms = t2;
		cout << "Render time: " << dt << " ms." << endl;

		if (EXPORT_AND_EXIT) {
			break;
		}
	}
	
	char c[20];
	_itoa_s(dt, c, 10);
	string filename("screenshot (");
	filename.append(c).append("ms).bmp");
	SDL_SaveBMP(screen, filename.c_str());

	// Cleaning
	for (Object3D* o : objects) {
		free(o);
	}

	return 0;
}

/* ================================ */
/*             CAMERA               */
/* ================================ */
/* Update the rotation matrix */
void UpdateRotationMatrix()
{
	R[0][0] = cos(yaw);
	R[0][2] = sin(yaw);
	R[2][0] = -sin(yaw);
	R[2][2] = cos(yaw);
}

vec3 GetAxis(Direction dir) {
	if (dir == Direction::UP) {
		return TRANSLATION * -vec3(R[1][0], R[1][1], R[1][2]);
	}
	else if (dir == Direction::DOWN) {
		return TRANSLATION * vec3(R[1][0], R[1][1], R[1][2]);
	}
	else if (dir == Direction::RIGHT) {
		return TRANSLATION * vec3(R[0][0], R[0][1], -R[0][2]);
	}
	else if (dir == Direction::LEFT) {
		return TRANSLATION * -vec3(R[0][0], R[0][1], -R[0][2]);
	}
	else if (dir == Direction::FORWARD) {
		return TRANSLATION * vec3(-R[2][0], R[2][1], R[2][2]);
	}
	// Backward
	return TRANSLATION * -vec3(-R[2][0], R[2][1], R[2][2]);
}

void Update()
{
	Uint8* keystate = SDL_GetKeyState(0);

	/* Camera rotation and translation */
	if (keystate[SDLK_UP])
	{
		// Move camera forward
		cameraPos += GetAxis(Direction::FORWARD);
	}
	if (keystate[SDLK_DOWN])
	{
		// Move camera backward
		cameraPos += GetAxis(Direction::BACKWARD);
	}
	if (keystate[SDLK_LEFT])
	{
		// Move camera to the left
		cameraPos += GetAxis(Direction::LEFT);

		/* Move the camera to the left while rotating to the right */
		// Rotate camera to the right
		yaw += ROTATION;

		UpdateRotationMatrix();
	}
	if (keystate[SDLK_RIGHT])
	{
		/* Move the camera to the right while rotating to the left */
		// Rotate camera to the left
		yaw -= ROTATION;

		UpdateRotationMatrix();

		// Move camera to the right
		cameraPos += GetAxis(Direction::RIGHT);
	}
}


/* ================================ */
/*             LIGHT                */
/* ================================ */
void initLights() {
	float startx, startz, endx, endz;

	// Light sources
	lightPositions.clear();
	lightPositions.reserve(LIGHT_ROWS * LIGHT_COLS);

	// Generate light sources positions
	if (LIGHT_DISTRIBUTION == LightDistribution::Uniform) {
		startx = LIGHT_CENTER.x - LIGHT_DIAMETER / 2;
		startz = LIGHT_CENTER.z - LIGHT_DIAMETER / 2;
		endx = LIGHT_CENTER.x + LIGHT_DIAMETER / 2;
		endz = LIGHT_CENTER.z + LIGHT_DIAMETER / 2;

		for (float z = startz; z <= endz + 0.0001; z += LIGHT_DIAMETER / (LIGHT_ROWS - 1)) {
			for (float x = startx; x <= endx + 0.0001; x += LIGHT_DIAMETER / (LIGHT_COLS - 1)) {
				lightPositions.push_back(vec3(x, LIGHT_CENTER.y, z));
			}
		}
	}

	// Light surface
	startx = LIGHT_CENTER.x - LIGHT_SURFACE_DIAMETER;
	startz = LIGHT_CENTER.z - LIGHT_SURFACE_DIAMETER;
	endx = LIGHT_CENTER.x + LIGHT_SURFACE_DIAMETER;
	endz = LIGHT_CENTER.z + LIGHT_SURFACE_DIAMETER;

	vec3 A(startx, LIGHT_CENTER.y, startz);
	vec3 B(startx, LIGHT_CENTER.y, endz);
	vec3 C(endx, LIGHT_CENTER.y, startz);
	vec3 D(endx, LIGHT_CENTER.y, endz);

	objects.push_back(new Triangle(D, B, C, vec3(1, 1, 1), Material::Diffuse));
	objects.push_back(new Triangle(B, A, C, vec3(1, 1, 1), Material::Diffuse));
	lightIdx.first = objects.size() - 2;
	lightIdx.second = objects.size() - 1;

	LIGHT_NORMAL = objects[lightIdx.first]->normal(vec3(0, 0, 0));
}

// Stochastic sampling applied to light positions
void initJitteredLightPositions() {
	lightPositions.clear();
	lightPositions.reserve(LIGHT_ROWS * LIGHT_ROWS);
	// Cell limits (part of a pixel)
	float x, y, startx, starty, endx, endy, stepx, stepy, factor;

	
	stepx = LIGHT_DIAMETER / LIGHT_ROWS;
	stepy = LIGHT_DIAMETER / LIGHT_COLS;
	startx = LIGHT_CENTER.x - LIGHT_ROWS / 2 * stepx;
	starty = LIGHT_CENTER.x - LIGHT_COLS / 2 * stepx;
	endx = startx + stepx;
	endy = starty + stepy;

	// Rows
	for (int r = 0; r < LIGHT_ROWS; ++r) {
		// Columns
		for (int c = 0; c < LIGHT_COLS; ++c) {
			x = startx + (rand() / (float)RAND_MAX) * stepx;
			y = starty + (rand() / (float)RAND_MAX) * stepy;

			lightPositions.push_back(vec3(x, LIGHT_CENTER.y, y));

			if (c != LIGHT_COLS - 1) {
				if (r % 2 == 0) {
					startx += stepx;
					endx += stepx;
				}
				else {
					startx -= stepx;
					endx -= stepx;
				}
			}
		}
		starty += stepy;
		endy += stepy;
	}
}

/* Return false if the light cannot get to the given point because of another surface (shadow), true otherwise
 * i.e. the vector cannot cross the same distance from the light since it intersects another surface before */
bool getLightPower(const Intersection& i, const vec3& lightPos, float lightDistance, vec3 &r, vec3& power) {
	Intersection closestIntersection;

	r = glm::normalize(lightPos - i.position);

	if (ClosestIntersection(i.position, r, i.objectIndex, closestIntersection) && closestIntersection.distance + 0.001 <= lightDistance) {
		return false;
	}
	power = lightColor;
	return true;
}

/* Return the light intensity hitting the given intersection from the light source */
vec3 DirectLight(const Intersection& i) {
	float distance;
	vec3 r, power(0, 0, 0), tmpPower;

	if (LIGHT_DISTRIBUTION == LightDistribution::Jittered) {
		initJitteredLightPositions();
	}

	for (const vec3& lightPos : lightPositions) {
		distance = GetDistance(i.position, lightPos);

		if (getLightPower(i, lightPos, distance, r, tmpPower)) {
			vec3 normal = objects[i.objectIndex]->normal(i.position);
			float div = 4 * M_PI * distance * distance;
			float max = fmax(glm::dot(r, normal), 0);
			tmpPower.x = tmpPower.x * max / div;
			tmpPower.y = tmpPower.y * max / div;
			tmpPower.z = tmpPower.z * max / div;

			power += tmpPower;
		} else {
			power += SHADOW_COLOR;
		}
	}

	return power / (float) (LIGHT_COLS * LIGHT_ROWS);
}


/* ================================ */
/*              RAYS                */
/* ================================ */
float GetDistance(vec3 p1, vec3 p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

/* Return true if the ray intersects with a triangle, false otherwise. If true, closestIntersection is initialized with the result */
bool ClosestIntersection(vec3 start, vec3 dir, int startObjIdx, Intersection& closestIntersection) {
	closestIntersection.distance = std::numeric_limits<float>::max();
	vec3 position;
	float distance;

	// Intersections with triangles and spheres
	for (int i = 0; i < objects.size(); ++i) {
		if (i != startObjIdx && objects[i]->intersects(start, dir, position)) {
			distance = GetDistance(position, start);
			if (distance <= closestIntersection.distance) {
				closestIntersection.distance = distance;
				closestIntersection.position = position;
				closestIntersection.objectIndex = i;
			}
		}
	}

	return closestIntersection.distance != std::numeric_limits<float>::max();
}

void getReflectedDirection(const vec3& incident, const vec3& normal, vec3& reflected)
{
	reflected = incident - normal * (2 * glm::dot(incident, normal));
}

// i1 and i2 incident and refracted indices. Return false if there is no refraction (total internal reflection)
bool getReflectionRefractionDirections(float i1, float i2, const vec3& incident, vec3& normal, vec3& reflected, vec3& refracted, float* refractionPercentage) {
	float cosI = glm::dot(incident, normal);
	if (cosI > 0) {
		// Incident and normal have the same direction, ray is inside the material
		normal = -normal;
	}
	else {
		// Incident and normal have opposite directions, so the ray is outside the material
		cosI = -cosI;
	}

	float eta = i1 / i2;
	float cs2 = 1.0f - eta * eta * (1 - cosI * cosI);

	getReflectedDirection(incident, normal, reflected);

	if (cs2 < 0) {
		// Total internal reflection
		return false;
	}

	*refractionPercentage = cs2;

	// Refraction
	refracted = glm::normalize(eta * incident + (eta * cosI - sqrt(cs2)) * normal);
	return true;
}

bool getColor(const vec3& start, const vec3& d_ray, int startObjIdx, vec3& color, int bounce, float refractiveIndice) {
	Intersection closestIntersection;

	// Fill a pixel with the color of the closest triangle intersecting the ray, black otherwise
	if (ClosestIntersection(start, d_ray, startObjIdx, closestIntersection)) {
		// Light source hit
		if (closestIntersection.objectIndex == lightIdx.first || closestIntersection.objectIndex == lightIdx.second) {
			color = lightColor;
			return true;
		}

		// Specular material (reflection) and max number of bounces not reached
		if (objects[closestIntersection.objectIndex]->material == Material::Specular && bounce < MAX_BOUNCES) {
			const vec3 normal = objects[closestIntersection.objectIndex]->normal(closestIntersection.position);
			vec3 next_ray;
			getReflectedDirection(d_ray, normal, next_ray);
			return getColor(closestIntersection.position, next_ray, closestIntersection.objectIndex, color, bounce + 1, refractiveIndice);
		}

		// Glass material (refraction)
		else if (objects[closestIntersection.objectIndex]->material == Material::Glass && bounce < MAX_BOUNCES) {
			vec3 reflected, refracted, normal;
			float nextRefractiveIndice, refractionPercentage;
			normal = objects[closestIntersection.objectIndex]->normal(closestIntersection.position);

			// Incident material is air
			if (refractiveIndice == AIR_REFRACTIVE_INDEX) {
				nextRefractiveIndice = GLASS_REFRACTIVE_INDEX;
			}
			// Incident material is glass
			else {
				nextRefractiveIndice = AIR_REFRACTIVE_INDEX;
			}

			// Compute the reflected and refracted vector direction and percentage
			if (getReflectionRefractionDirections(refractiveIndice, nextRefractiveIndice, d_ray, normal, reflected, refracted, &refractionPercentage)) {
				// Refraction
				vec3 color2;
				bool b1 = getColor(closestIntersection.position, refracted, closestIntersection.objectIndex, color, bounce + 1, nextRefractiveIndice);
				bool b2 = getColor(closestIntersection.position, reflected, closestIntersection.objectIndex, color2, bounce + 1, nextRefractiveIndice);
				color = refractionPercentage * color + (1 - refractionPercentage) * color2;
				return true;
			}

			// Reflection caused by an angle between the incident ray and the normal higher than the critical angle (internal total reflection)
			return getColor(closestIntersection.position, reflected, closestIntersection.objectIndex, color, bounce + 1, refractiveIndice);
		}
		// Diffuse and specular material
		else if (objects[closestIntersection.objectIndex]->material == Material::DiffuseSpecular && bounce < MAX_BOUNCES) {
			const vec3 normal = objects[closestIntersection.objectIndex]->normal(closestIntersection.position);
			// Material color
			color = (DirectLight(closestIntersection) + indirectLight) * objects[closestIntersection.objectIndex]->color;
			vec3 next_ray, color2;
			getReflectedDirection(d_ray, normal, next_ray);
			// Reflection color
			getColor(closestIntersection.position, next_ray, closestIntersection.objectIndex, color2, bounce + 1, refractiveIndice);
			color = (1 - DIFFUSE_SPECULAR_REFLECTION) * color + DIFFUSE_SPECULAR_REFLECTION * color2;
			return true;
		}

		// Diffuse material
		// Compute light color according to illumination and material reflectance
		color = (DirectLight(closestIntersection) + indirectLight) * objects[closestIntersection.objectIndex]->color;
		return true;
	}
	else {
		color = VOID_COLOR;
		return false;
	}
}


/* ================================ */
/*         POST-PROCESSING          */
/* ================================ */ 

/* Applied Sobel operator to the pixel neighbors */
float sobelOperator(int x, int y) {
	return abs((screenPixelsIntensity[y - 1][x - 1] + 2.0f * screenPixelsIntensity[y - 1][x] + screenPixelsIntensity[y - 1][x + 1])
		- (screenPixelsIntensity[y + 1][x - 1] + 2.0f * screenPixelsIntensity[y + 1][x] + screenPixelsIntensity[y + 1][x + 1]))
		+ abs((screenPixelsIntensity[y - 1][x + 1] + 2.0f * screenPixelsIntensity[y][x + 1] + screenPixelsIntensity[y + 1][x + 1])
		- (screenPixelsIntensity[y - 1][x - 1] + 2.0f * screenPixelsIntensity[y][x - 1] + screenPixelsIntensity[y + 1][x - 1]));
}

/* Transform the RGB vector of every pixel into its intensity */
void computePixelsIntensity() {
	for (int y = 1; y < SCREEN_HEIGHT - 1; ++y) {
		for (int x = 1; x < SCREEN_WIDTH - 1; ++x) {
			screenPixelsIntensity[y][x] = glm::dot(screenPixels[y][x], INTENSITY_WEIGHTS);
		}
	}
}

// Stochastic sampling anti-aliasing (2x, 4x, 8x or 16x)
vec3 stochasticSamplingAA(int pixelx, int pixely) {
	vec3 color = vec3(0, 0, 0);
	// Anti-aliasing grid
	int rows, cols;
	// Cell limits (part of a pixel)
	float x, y, startx = pixelx - 0.5, starty = pixely - 0.5, endx, endy, stepx, stepy, factor;

	if (ANTI_ALIASING == AntiAliasing::StochasticSampling2x) {
		endx = pixelx + 0.5;
		endy = pixely;
		rows = 2;
		cols = 1;
		factor = 2.0f;
	} else if (ANTI_ALIASING == AntiAliasing::StochasticSampling4x) {
		endx = pixelx;
		endy = pixely;
		rows = 2;
		cols = 2;
		factor = 4.0f;
	} else if(ANTI_ALIASING == AntiAliasing::StochasticSampling8x) {
		endx = pixelx;
		endy = pixely - 0.25;
		rows = 4;
		cols = 2;
		factor = 8.0f;
	} else {
		// StochasticSampling16x
		endx = pixelx - 0.25;
		endy = pixely - 0.25;
		rows = 4;
		cols = 4;
		factor = 16.0f;
	}

	stepx = endx - startx;
	stepy = endy - starty;

	// Rows
	for (int r = 0; r < rows; ++r) {
		// Columns
		for (int c = 0; c < cols; ++c) {
			x = startx + (rand() / (float) RAND_MAX) * stepx;
			y = starty + (rand() / (float) RAND_MAX) * stepy;

			color += traceRayFromCamera(x, y);

			if (c != cols - 1) {
				if (r % 2 == 0) {
					startx += stepx;
					endx += stepx;
				}
				else {
					startx -= stepx;
					endx -= stepx;
				}
			}
		}
		starty += stepy;
		endy += stepy;
	}

	return color / factor;
}

// Supersampling anti-aliasing 8x
vec3 supersamplingAA(int x, int y) {
	vec3 color = screenPixels[y][x];
	for (float y1 = y - 0.5; y1 < y + 1; y1 += 0.5) {
		for (float x1 = x - 0.5; x1 < x + 1; x1 += 0.5) {
			if (x1 != x || y1 != y) {
				color += traceRayFromCamera(x1, y1);
			}
		}
	}
	return color / 9.0f;
}

void antiAliasing() {
	list<pair<int, int>> aliasedEdges;

	computePixelsIntensity();

	// Iterate through every pixel
	for (int y = 1; y < SCREEN_HEIGHT-1; ++y) {
		for (int x = 1; x < SCREEN_WIDTH-1; ++x) {

			// If the edge is aliased
			if (sobelOperator(x, y) > SOBEL_THRESHOLD) {
				// Store the pixel
				aliasedEdges.push_back(pair<int, int>(x, y));
			}
		}
	}

	// Apply anti-aliasing by shooting new rays around the edge and averaging the values
	for (pair<int, int> p : aliasedEdges) {
		if (ANTI_ALIASING == AntiAliasing::Supersampling8x) {
			screenPixels[p.second][p.first] = supersamplingAA(p.first, p.second);
		} else {
			screenPixels[p.second][p.first] = stochasticSamplingAA(p.first, p.second);
		}
	}
}


/* ================================ */
/*             DRAWING              */
/* ================================ */

vec3 traceRayFromCamera(float x, float y) {
	vec3 color;

	// Ray direction
	vec3 d_ray(x - SCREEN_WIDTH / 2.0, y - SCREEN_HEIGHT / 2.0, focalLength);
	// Ray rotation
	if (yaw != 0) {
		d_ray = d_ray * R;
	}
	d_ray = glm::normalize(d_ray);

	// Trace a ray and retrieve the pixel color of the intersected object, after illumination and reflection bounces
	getColor(cameraPos, d_ray, NOT_STARTOBJIDX, color, 0, AIR_REFRACTIVE_INDEX);
	return color;
}

void Draw()
{
	vec3 color;

	if (SDL_MUSTLOCK(screen))
		SDL_LockSurface(screen);

	for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		for (int x = 0; x < SCREEN_WIDTH; ++x) {
			screenPixels[y][x] = traceRayFromCamera(x, y);
		}
	}

	if (ANTI_ALIASING != AntiAliasing::Disabled) {
		antiAliasing();
	}

	for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		for (int x = 0; x < SCREEN_WIDTH; ++x) {
			PutPixelSDL(screen, x, y, screenPixels[y][x]);
		}
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);

	SDL_UpdateRect(screen, 0, 0, 0, 0);
}
