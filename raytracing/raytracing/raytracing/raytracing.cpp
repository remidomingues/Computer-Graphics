/* 
 * Raytracer supporting the following features:
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
 * - Multithreading
 *
 * The most important constants to tweak are:
 *	- THREADS
 *	- EXPORT_AND_EXIT
 *	- SCREEN_WIDTH and SCREEN_HEIGHT
 *	- LIGHT_ROWS and LIGHT_COLS
 *	- ANTI_ALIASING
 */

#define _USE_MATH_DEFINES

#include <algorithm>
#include <list>
#include <math.h>
#include <time.h>
#include "SDLauxiliary.h"
#include "Raytracing.h"
#include "Multithreading.h"

/* ==================================================================================
* IMPORTANT : PLEASE MAKE SURE TO COMPILE WITH THE FLAG -o2 to optimize the binary!
* ================================================================================== */

// ----------------------------------------------------------------------------
// RENDERING PARAMETERS
// Number of threads to process the scene (post-processing excluded). One single thread if value is <= 1
const int THREADS = 8;

// If true, render the image, export it in a .bmp file then exit
const bool EXPORT_AND_EXIT = true;

// Rendering resolution
const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;

// Light distribution
LightDistribution LIGHTS_DISTRIBUTION = LightDistribution::Jittered;

// Number of sampled lights will be LIGHT_ROWS * LIGHT_COLS
int LIGHT_ROWS = 16;
int LIGHT_COLS = 16;

// Maximum number of light bounces by refraction and reflection
const int MAX_BOUNCES = 20;
// Minimal weight a ray must have to be refracted or reflected
const float MIN_RAY_WEIGHT = 0.0001f;

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
vec3 LIGHT_CENTER(0, -1.1, 0);
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

/* Progress notification */
bool DISPLAY_PROGRESS = true;
int PROGRESS_STEP = 1;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
int start_time;

/* Screen */
SDL_Surface* screen;
vec3** screenPixels;

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
vec3* uniformLightPositions = NULL;
vec3** jitteredLightPositions;
pair<int, int> lightIdx;
const vec3 lightColor = 14.f * vec3(1, 1, 1);
const vec3 indirectLight = 0.55f * vec3(1, 1, 1);
const int NOT_STARTOBJIDX = -1;

/* Anti-aliasing */
float screenPixelsIntensity[SCREEN_HEIGHT][SCREEN_WIDTH];
list<pair<int, int>>* aliasedEdges = new list<pair<int, int>>();


/* ================================ */
/*             CAMERA               */
/* ================================ */
/* update the rotation matrix */
void updateRotationMatrix()
{
	R[0][0] = cos(yaw);
	R[0][2] = sin(yaw);
	R[2][0] = -sin(yaw);
	R[2][2] = cos(yaw);
}

/* Return a 3D vector corresponding to the specified axis according to camera rotation */
vec3 getAxis(Direction dir) {
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

/* Move the camera if one or more arrow keys are pressed */
void update()
{
	Uint8* keystate = SDL_GetKeyState(0);

	/* Camera rotation and translation */
	if (keystate[SDLK_UP])
	{
		// Move camera forward
		cameraPos += getAxis(Direction::FORWARD);
	}
	if (keystate[SDLK_DOWN])
	{
		// Move camera backward
		cameraPos += getAxis(Direction::BACKWARD);
	}
	if (keystate[SDLK_LEFT])
	{
		// Rotate camera to the right
		yaw += ROTATION;

		updateRotationMatrix();
	}
	if (keystate[SDLK_RIGHT])
	{
		// Rotate camera to the left
		yaw -= ROTATION;

		updateRotationMatrix();
	}

	/* Light translation */
	if (keystate[SDLK_w])
	{
		LIGHT_CENTER += getAxis(Direction::FORWARD);
		initLights();
	}
	if (keystate[SDLK_s])
	{
		LIGHT_CENTER += getAxis(Direction::BACKWARD);
		initLights();
	}
	if (keystate[SDLK_a])
	{
		LIGHT_CENTER += getAxis(Direction::LEFT);
		initLights();
	}
	if (keystate[SDLK_d])
	{
		LIGHT_CENTER += getAxis(Direction::RIGHT);
		initLights();
	}
	if (keystate[SDLK_q])
	{
		LIGHT_CENTER += getAxis(Direction::UP);
		initLights();
	}
	if (keystate[SDLK_e])
	{
		LIGHT_CENTER += getAxis(Direction::DOWN);
		initLights();
	}
}

float getElapsedTime() {
	return float(SDL_GetTicks() - start_time);
}

/* ================================ */
/*             LIGHT                */
/* ================================ */
/* Add the light surface to the model and generate the light sources
* if a uniform distribution is specified */
void initLights() {
	if (LIGHTS_DISTRIBUTION == LightDistribution::Uniform) {
		float startx, startz, endx, endz;

		if (uniformLightPositions != NULL) {
			free(uniformLightPositions);
		}

		uniformLightPositions = new vec3[LIGHT_COLS * LIGHT_ROWS];

		// Generate light sources positions
		startx = LIGHT_CENTER.x - LIGHT_DIAMETER / 2;
		startz = LIGHT_CENTER.z - LIGHT_DIAMETER / 2;
		endx = LIGHT_CENTER.x + LIGHT_DIAMETER / 2;
		endz = LIGHT_CENTER.z + LIGHT_DIAMETER / 2;

		int i = 0;
		for (float z = startz; z <= endz + 0.0001; z += LIGHT_DIAMETER / (LIGHT_ROWS - 1)) {
			for (float x = startx; x <= endx + 0.0001; x += LIGHT_DIAMETER / (LIGHT_COLS - 1)) {
				uniformLightPositions[i].x = x;
				uniformLightPositions[i].y = LIGHT_CENTER.y;
				uniformLightPositions[i].z = z;
				++i;
			}
		}
	}
	else {
		jitteredLightPositions = new vec3*[THREADS];
		for (int i = 0; i < THREADS; ++i) {
			jitteredLightPositions[i] = new vec3[LIGHT_COLS * LIGHT_ROWS];
		}
	}
}

void initLightSurface() {
	// Light surface
	float startx = LIGHT_CENTER.x - LIGHT_SURFACE_DIAMETER;
	float startz = LIGHT_CENTER.z - LIGHT_SURFACE_DIAMETER;
	float endx = LIGHT_CENTER.x + LIGHT_SURFACE_DIAMETER;
	float endz = LIGHT_CENTER.z + LIGHT_SURFACE_DIAMETER;

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

/* Stochastic sampling applying the specified action
* pixelx, pixely and the return value are only relevant for antialiasing calls */
vec3 stochasticSampling(int action, int pixelx, int pixely, int rank) {
	vec3 color = screenPixels[pixely][pixelx];
	int rows = 1, cols = 1, i = 0;
	// Cell limits (subparts of a pixel)
	float x, y, startx, starty, endx, endy, stepx, stepy, factor;

	if (action == LightDistribution::Jittered) {
		rows = LIGHT_ROWS;
		cols = LIGHT_COLS;
		stepx = LIGHT_DIAMETER / rows;
		stepy = LIGHT_DIAMETER / cols;
		startx = LIGHT_CENTER.x - rows / 2 * stepx;
		starty = LIGHT_CENTER.x - cols / 2 * stepx;
	}
	else {
		if (action == AntiAliasing::StochasticSampling2x) {
			rows = 2;
			cols = 1;
		}
		else if (action == AntiAliasing::StochasticSampling4x) {
			rows = 2;
			cols = 2;
		}
		else if (action == AntiAliasing::StochasticSampling8x) {
			rows = 4;
			cols = 2;
		}
		else if (action == AntiAliasing::StochasticSampling16x) {
			rows = 4;
			cols = 4;
		}
		else if (action == AntiAliasing::StochasticSampling64x) {
			rows = 8;
			cols = 8;
		}

		startx = pixelx - 0.5;
		starty = pixely - 0.5;
		stepx = 1.0f / rows;
		stepy = 1.0f / cols;
	}

	endx = startx + stepx;
	endy = starty + stepy;

	// Rows
	for (int r = 0; r < rows; ++r) {
		// Columns
		for (int c = 0; c < cols; ++c) {
			x = startx + (rand() / (float)RAND_MAX) * stepx;
			y = starty + (rand() / (float)RAND_MAX) * stepy;

			if (action == LightDistribution::Jittered) {
				jitteredLightPositions[rank][i].x = x;
				jitteredLightPositions[rank][i].y = LIGHT_CENTER.y;
				jitteredLightPositions[rank][i].z = y;
				++i;
			}
			else {
				color += traceRayFromCamera(x, y, rank);
			}

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

	return color / ((float)(rows * cols) + 1);
}

/* Return false if the light cannot get to the given point because of another surface (shadow), true otherwise
* i.e. the vector cannot cross the same distance from the light since it intersects another surface before */
bool getLightPower(const Intersection& i, const vec3& lightPos, float lightDistance, vec3 &r, vec3& power) {
	Intersection closestI;

	r = glm::normalize(lightPos - i.position);

	if (closestIntersection(i.position, r, i.objectIndex, closestI) && closestI.distance + 0.001 <= lightDistance) {
		return false;
	}
	power = lightColor;
	return true;
}

/* Return the light intensity hitting the given intersection from the light source */
vec3 directLight(const Intersection& i, int rank) {
	float distance;
	vec3 r, power(0, 0, 0), tmpPower;
	vec3* lightPositions = NULL;

	if (LIGHTS_DISTRIBUTION == LightDistribution::Jittered) {
		stochasticSampling(LIGHTS_DISTRIBUTION, 0, 0, rank);
		lightPositions = jitteredLightPositions[rank];
	}
	else {
		lightPositions = uniformLightPositions;
	}

	for(int l = 0; l < LIGHT_COLS * LIGHT_ROWS; ++l) {
		distance = getDistance(i.position, lightPositions[l]);

		if (getLightPower(i, lightPositions[l], distance, r, tmpPower)) {
			vec3 normal = objects[i.objectIndex]->normal(i.position);
			float div = 4 * M_PI * distance * distance;
			float max = fmax(glm::dot(r, normal), 0);
			tmpPower.x = tmpPower.x * max / div;
			tmpPower.y = tmpPower.y * max / div;
			tmpPower.z = tmpPower.z * max / div;

			power += tmpPower;
		}
		else {
			power += SHADOW_COLOR;
		}
	}

	return power / (float)(LIGHT_COLS * LIGHT_ROWS);
}


/* ================================ */
/*              RAYS                */
/* ================================ */
/* Return the distance between two 3D points */
float getDistance(const vec3& p1, const vec3& p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

/* Return true if the ray intersects with a triangle, false otherwise. If true, closestIntersection is initialized with the result */
bool closestIntersection(const vec3& start, const vec3& dir, int startObjIdx, Intersection& closestIntersection) {
	closestIntersection.distance = std::numeric_limits<float>::max();
	vec3 position;
	float distance;

	// Intersections with triangles and spheres
	for (int i = 0; i < objects.size(); ++i) {
		if (i != startObjIdx && objects[i]->intersects(start, dir, position)) {
			distance = getDistance(position, start);
			if (distance <= closestIntersection.distance) {
				closestIntersection.distance = distance;
				closestIntersection.position = position;
				closestIntersection.objectIndex = i;
			}
		}
	}

	return closestIntersection.distance != std::numeric_limits<float>::max();
}

/* Initialize the reflected vector according to the incident vector and object normal */
void getReflectedDirection(const vec3& incident, const vec3& normal, vec3& reflected) {
	reflected = incident - normal * (2 * glm::dot(incident, normal));
}

/* Initialize the reflected and refracted vectors according to the incident vector,
* the object normal (updated if same direction) and the material indices (i1 current, i2 next)
* refractionPercentage is initialized to the percentage of light refracted
* Return true if there is a refraction (and reflection), false otherwise (total internal reflection)
*/
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

/* Return true if the ray from the start position and direction (d_ray) intersects an object, false otherwise (void)
* The intersected object cannot have the index startObjIdx
* coloc is initialized with the color of the object hit (may be multiple bounces if refraction or reflection)
* refractiveIndice is the current indice
*/
bool getColor(const vec3& start, const vec3& d_ray, int startObjIdx, vec3& color, int bounce, float refractiveIndice, float weight, int rank) {
	Intersection intersection;

	if (weight < MIN_RAY_WEIGHT) {
		color = VOID_COLOR;
		return false;
	}

	// Fill a pixel with the color of the closest triangle intersecting the ray, black otherwise
	if (closestIntersection(start, d_ray, startObjIdx, intersection)) {
		// Light source hit
		if (intersection.objectIndex == lightIdx.first || intersection.objectIndex == lightIdx.second) {
			color = lightColor;
			return true;
		}

		// Specular material (reflection) and max number of bounces not reached
		if (objects[intersection.objectIndex]->material == Material::Specular && bounce < MAX_BOUNCES) {
			const vec3 normal = objects[intersection.objectIndex]->normal(intersection.position);
			vec3 next_ray;
			getReflectedDirection(d_ray, normal, next_ray);
			return getColor(intersection.position, next_ray, intersection.objectIndex, color, bounce + 1, refractiveIndice, weight, rank);
		}

		// Glass material (refraction)
		else if (objects[intersection.objectIndex]->material == Material::Glass && bounce < MAX_BOUNCES) {
			vec3 reflected, refracted, normal;
			float nextRefractiveIndice, refractionPercentage;
			normal = objects[intersection.objectIndex]->normal(intersection.position);

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
				bool b1 = getColor(intersection.position, refracted, intersection.objectIndex, color, bounce + 1, nextRefractiveIndice, weight * refractionPercentage, rank);
				bool b2 = getColor(intersection.position, reflected, intersection.objectIndex, color2, bounce + 1, nextRefractiveIndice, weight * (1 - refractionPercentage), rank);
				color = refractionPercentage * color + (1 - refractionPercentage) * color2;
				return b1 || b2;
			}

			// Reflection caused by an angle between the incident ray and the normal higher than the critical angle (internal total reflection)
			return getColor(intersection.position, reflected, intersection.objectIndex, color, bounce + 1, refractiveIndice, weight, rank);
		}

		// Diffuse and specular material
		else if (objects[intersection.objectIndex]->material == Material::DiffuseSpecular && bounce < MAX_BOUNCES) {
			const vec3 normal = objects[intersection.objectIndex]->normal(intersection.position);
			// Material color
			color = (directLight(intersection, rank) + indirectLight) * objects[intersection.objectIndex]->color;
			vec3 next_ray, color2;
			getReflectedDirection(d_ray, normal, next_ray);
			// Reflection color
			getColor(intersection.position, next_ray, intersection.objectIndex, color2, bounce + 1, refractiveIndice, weight * DIFFUSE_SPECULAR_REFLECTION, rank);
			color = (1 - DIFFUSE_SPECULAR_REFLECTION) * color + DIFFUSE_SPECULAR_REFLECTION * color2;
			return true;
		}

		// Diffuse material
		// Compute light color according to illumination and material reflectance
		color = (directLight(intersection, rank) + indirectLight) * objects[intersection.objectIndex]->color;
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

/* Applied Sobel operator to the pixel neighbors to detect the edges needing anti-aliasing */
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

/* Supersampling anti-aliasing 8x */
vec3 supersamplingAA(int x, int y) {
	vec3 color = screenPixels[y][x];
	for (float y1 = y - 0.5; y1 < y + 1; y1 += 0.5) {
		for (float x1 = x - 0.5; x1 < x + 1; x1 += 0.5) {
			if (x1 != x || y1 != y) {
				color += traceRayFromCamera(x1, y1, 0);
			}
		}
	}
	return color / 9.0f;
}

/* Apply anti-aliasing to one pixel */
void antiAliasingOnPixel(vec3** screenPixels, const pair<int, int>& p, int rank) {
	if (ANTI_ALIASING == AntiAliasing::Uniform8x) {
		screenPixels[p.second][p.first] = supersamplingAA(p.first, p.second);
	}
	else {
		screenPixels[p.second][p.first] = stochasticSampling(ANTI_ALIASING, p.first, p.second, rank);
	}
}

/* Return true if the anti-aliasing is enabled, false otherwise */
bool antiAliasingEnabled() {
	return ANTI_ALIASING != AntiAliasing::Disabled;
}

/* Compute the edges needing anti-aliasing accordint to the Sobel operator
* then apply the specified anti-aliasing */
void antiAliasing() {
	int i = 0, size;
	float tmp, elapsed;
	int nextStep = PROGRESS_STEP;
	aliasedEdges->clear();
	computePixelsIntensity();

	// Iterate through every pixel
	for (int y = 1; y < SCREEN_HEIGHT - 1; ++y) {
		for (int x = 1; x < SCREEN_WIDTH - 1; ++x) {

			// If the edge is aliased
			if (sobelOperator(x, y) > SOBEL_THRESHOLD) {
				// Store the pixel
				aliasedEdges->push_back(pair<int, int>(x, y));
			}
		}
	}

	// Apply anti-aliasing by shooting new rays around the edge and averaging the values
	if (THREADS <= 1) {
		size = aliasedEdges->size();

		for (const pair<int, int>& p : *aliasedEdges) {
			antiAliasingOnPixel(screenPixels, p, 0);

			// Progress display
			tmp = (i + 1) * 100 / (float) size;
			if (DISPLAY_PROGRESS && tmp >= nextStep * 2) {
				elapsed = float(SDL_GetTicks() - start_time);
				cout << 50 + nextStep << "% (" << elapsed / 1000.0f << " seconds)" << endl;
				nextStep += PROGRESS_STEP;
			}
			++i;
		}
	} else {
		multithreadedAntiAliasing(screenPixels, aliasedEdges, THREADS);
	}
}


/* ================================ */
/*             DRAWING              */
/* ================================ */

/* Return the color obtained by tracing a ray from the camera position
* to the pixel (x, y) in the 2D screen resolution (may be bounces) */
vec3 traceRayFromCamera(float x, float y, int rank) {
	vec3 color;

	// Ray direction
	vec3 d_ray(x - SCREEN_WIDTH / 2.0, y - SCREEN_HEIGHT / 2.0, FOCAL_LENGTH);
	// Ray rotation
	if (yaw != 0) {
		d_ray = d_ray * R;
	}
	d_ray = glm::normalize(d_ray);

	// Trace a ray and retrieve the pixel color of the intersected object, after illumination and reflection bounces
	getColor(cameraPos, d_ray, NOT_STARTOBJIDX, color, 0, AIR_REFRACTIVE_INDEX, 1.0f, rank);
	return color;
}

/* Scene processing, compute the scene pixels */
void process() {
	start_time = SDL_GetTicks();
	vec3 color;
	float tmp, elapsed;
	int nextStep = PROGRESS_STEP;

	for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		for (int x = 0; x < SCREEN_WIDTH; ++x) {
			screenPixels[y][x] = traceRayFromCamera(x, y, 0);
		}

		// Display percentage of computations achieved
		tmp = (y + 1) * 100 / (float) SCREEN_HEIGHT;
		if (DISPLAY_PROGRESS &&
			((ANTI_ALIASING == AntiAliasing::Disabled && tmp >= nextStep) ||
			(ANTI_ALIASING != AntiAliasing::Disabled && tmp >= nextStep * 2))) {
			elapsed = float(SDL_GetTicks() - start_time);
			cout << nextStep << "% (" << elapsed / 1000.0f << " seconds)" << endl;
			nextStep += PROGRESS_STEP;
		}
	}
}

/* Scene post-processing */
void postProcess() {
	if (ANTI_ALIASING != AntiAliasing::Disabled) {
		if (DISPLAY_PROGRESS) {
			cout << "Applying anti-aliasing..." << endl;
		}
		antiAliasing();
	}
}

/* Display the scene on the screen */
void display() {
	if (SDL_MUSTLOCK(screen))
		SDL_LockSurface(screen);

	for (int y = 0; y < SCREEN_HEIGHT; ++y) {
		for (int x = 0; x < SCREEN_WIDTH; ++x) {
			PutPixelSDL(screen, x, y, screenPixels[y][x]);
		}
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);

	SDL_UpdateRect(screen, 0, 0, 0, 0);
}

/* Draw the scene by tracing rays from the camera */
void draw()
{
	if (DISPLAY_PROGRESS) {
		cout << "Processing scene..." << endl;
	}

	if (THREADS <= 1) {
		process();
	} else {
		multithreadedProcess(screenPixels, SCREEN_WIDTH, SCREEN_HEIGHT, THREADS, ANTI_ALIASING != AntiAliasing::Disabled);
	}

	postProcess();

	display();
}

/* Config display */
void displayConfig() {
	cout << "Cornell Box" << endl << "-----------" << endl << "- Threads: "
		<< THREADS << endl << "- Resolution: " << SCREEN_WIDTH
		<< "x" << SCREEN_HEIGHT << endl << "- Lights: " << LIGHT_ROWS * LIGHT_COLS;
	if (LIGHTS_DISTRIBUTION == LightDistribution::Uniform) {
		cout << " (uniform)" << endl;
	}
	else if (LIGHTS_DISTRIBUTION == LightDistribution::Jittered) {
		cout << " (jittered)" << endl;
	}
	cout << "- Max bounces per ray: " << MAX_BOUNCES << endl << "- Anti-aliasing: ";
	if (ANTI_ALIASING == AntiAliasing::Disabled) {
		cout << "Disabled" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::StochasticSampling2x) {
		cout << "Stochastic Sampling 2x" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::StochasticSampling4x) {
		cout << "Stochastic Sampling 4x" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::StochasticSampling8x) {
		cout << "Stochastic Sampling 8x" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::StochasticSampling16x) {
		cout << "Stochastic Sampling 16x" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::StochasticSampling64x) {
		cout << "Stochastic Sampling 64x" << endl;
	}
	else if (ANTI_ALIASING == AntiAliasing::Uniform8x) {
		cout << "Uniform 8x" << endl;
	}
}

/* Load the model then draw the scene, moving the camera
* between each frame if required */
int main(int argc, char* argv[])
{
	displayConfig();

	// Initialization
	screenPixels = new vec3*[SCREEN_HEIGHT];
	for (int i = 0; i < SCREEN_HEIGHT; ++i) {
		screenPixels[i] = new vec3[SCREEN_WIDTH];
	}
	if (THREADS > 1) {
		mutex = CreateMutex(NULL, FALSE, NULL);
	}

	srand(time(NULL));
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
	R[1][1] = 1;
	updateRotationMatrix();
	int time_ms = SDL_GetTicks(), t2, dt; // Timer

	// Load model
	loadModel(objects);

	// Initialize the light sources and the light surface
	initLights();
	initLightSurface();

	while (NoQuitMessageSDL())
	{
		update();
		draw();

		// Display computation time
		t2 = SDL_GetTicks();
		dt = float(t2 - time_ms);
		time_ms = t2;
		cout << "Render time: " << dt / 1000.0f << " seconds" << endl << endl;

		if (EXPORT_AND_EXIT) {
			break;
		}
	}

	// Export scene into .bmp file
	char c[50];
	string filename("screenshot (");
	_itoa_s(dt / 1000.0f, c, 10);
	filename.append(c).append(" s).bmp");
	SDL_SaveBMP(screen, filename.c_str());

	// Cleaning
	for (int i = 0; i < SCREEN_HEIGHT; ++i) {
		free(screenPixels[i]);
	}
	free(screenPixels);

	if (LIGHTS_DISTRIBUTION == LightDistribution::Uniform) {
		free(uniformLightPositions);
	} else {
		for (int i = 0; i < THREADS; ++i) {
			free(jitteredLightPositions[i]);
		}
		free(jitteredLightPositions);
	}

	if (THREADS > 1) {
		CloseHandle(mutex);
	}

	for (Object3D* o : objects) {
		free(o);
	}
	free(aliasedEdges);

	return 0;
}
