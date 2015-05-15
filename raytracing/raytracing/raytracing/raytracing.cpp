#define _USE_MATH_DEFINES

#include <algorithm>
#include <list>
#include <math.h>
#include <time.h>
#include "SDLauxiliary.h"
#include "Raytracer.h"

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
		// Move camera to the left
		cameraPos += getAxis(Direction::LEFT);

		/* Move the camera to the left while rotating to the right */
		// Rotate camera to the right
		yaw += ROTATION;

		updateRotationMatrix();
	}
	if (keystate[SDLK_RIGHT])
	{
		/* Move the camera to the right while rotating to the left */
		// Rotate camera to the left
		yaw -= ROTATION;

		updateRotationMatrix();

		// Move camera to the right
		cameraPos += getAxis(Direction::RIGHT);
	}
}


/* ================================ */
/*             LIGHT                */
/* ================================ */
/* Add the light surface to the model and generate the light sources
 * if a uniform distribution is specified */
void initLights() {
	float startx, startz, endx, endz;

	// Light sources
	lightPositions.clear();
	lightPositions.reserve(LIGHT_ROWS * LIGHT_COLS);

	// Generate light sources positions
	if (LIGHTS_DISTRIBUTION == LightDistribution::Uniform) {
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

/* Stochastic sampling applying the specified action
 * pixelx, pixely and the return value are only relevant for antialiasing calls */
vec3 stochasticSampling(int action, int pixelx, int pixely) {
	vec3 color(0, 0, 0);
	int rows = 1, cols = 1;
	// Cell limits (subparts of a pixel)
	float x, y, startx, starty, endx, endy, stepx, stepy, factor;

	if (action == LightDistribution::Jittered) {
		rows = LIGHT_ROWS;
		cols = LIGHT_COLS;
		stepx = LIGHT_DIAMETER / rows;
		stepy = LIGHT_DIAMETER / cols;
		startx = LIGHT_CENTER.x - rows / 2 * stepx;
		starty = LIGHT_CENTER.x - cols / 2 * stepx;

		lightPositions.clear();
		lightPositions.reserve(rows * cols);
	} else {
		if (action == AntiAliasing::StochasticSampling2x) {
			rows = 2;
			cols = 1;
		} else if (action == AntiAliasing::StochasticSampling4x) {
			rows = 2;
			cols = 2;
		} else if (action == AntiAliasing::StochasticSampling8x) {
			rows = 4;
			cols = 2;
		} else if (action == AntiAliasing::StochasticSampling16x) {
			rows = 4;
			cols = 4;
		} else if (action == AntiAliasing::StochasticSampling64x) {
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
				lightPositions.push_back(vec3(x, LIGHT_CENTER.y, y));
			} else {
				color += traceRayFromCamera(x, y);
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

	return color / (float) (rows * cols);
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
vec3 directLight(const Intersection& i) {
	float distance;
	vec3 r, power(0, 0, 0), tmpPower;

	if (LIGHTS_DISTRIBUTION == LightDistribution::Jittered) {
		stochasticSampling(LIGHTS_DISTRIBUTION, 0, 0);
	}

	for (const vec3& lightPos : lightPositions) {
		distance = getDistance(i.position, lightPos);

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
/* Return the distance between two 3D points */
float getDistance(vec3 p1, vec3 p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

/* Return true if the ray intersects with a triangle, false otherwise. If true, closestIntersection is initialized with the result */
bool closestIntersection(vec3 start, vec3 dir, int startObjIdx, Intersection& closestIntersection) {
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
bool getColor(const vec3& start, const vec3& d_ray, int startObjIdx, vec3& color, int bounce, float refractiveIndice) {
	Intersection intersection;

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
			return getColor(intersection.position, next_ray, intersection.objectIndex, color, bounce + 1, refractiveIndice);
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
				bool b1 = getColor(intersection.position, refracted, intersection.objectIndex, color, bounce + 1, nextRefractiveIndice);
				bool b2 = getColor(intersection.position, reflected, intersection.objectIndex, color2, bounce + 1, nextRefractiveIndice);
				color = refractionPercentage * color + (1 - refractionPercentage) * color2;
				return true;
			}

			// Reflection caused by an angle between the incident ray and the normal higher than the critical angle (internal total reflection)
			return getColor(intersection.position, reflected, intersection.objectIndex, color, bounce + 1, refractiveIndice);
		}
		// Diffuse and specular material
		else if (objects[intersection.objectIndex]->material == Material::DiffuseSpecular && bounce < MAX_BOUNCES) {
			const vec3 normal = objects[intersection.objectIndex]->normal(intersection.position);
			// Material color
			color = (directLight(intersection) + indirectLight) * objects[intersection.objectIndex]->color;
			vec3 next_ray, color2;
			getReflectedDirection(d_ray, normal, next_ray);
			// Reflection color
			getColor(intersection.position, next_ray, intersection.objectIndex, color2, bounce + 1, refractiveIndice);
			color = (1 - DIFFUSE_SPECULAR_REFLECTION) * color + DIFFUSE_SPECULAR_REFLECTION * color2;
			return true;
		}

		// Diffuse material
		// Compute light color according to illumination and material reflectance
		color = (directLight(intersection) + indirectLight) * objects[intersection.objectIndex]->color;
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
				color += traceRayFromCamera(x1, y1);
			}
		}
	}
	return color / 9.0f;
}

/* Compute the edges needing anti-aliasing accordint to the Sobel operator
 * then apply the specified anti-aliasing */
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
			screenPixels[p.second][p.first] = stochasticSampling(ANTI_ALIASING, p.first, p.second);
		}
	}
}


/* ================================ */
/*             DRAWING              */
/* ================================ */

/* Return the color obtained by tracing a ray from the camera position
 * to the pixel (x, y) in the 2D screen resolution (may be bounces) */
vec3 traceRayFromCamera(float x, float y) {
	vec3 color;

	// Ray direction
	vec3 d_ray(x - SCREEN_WIDTH / 2.0, y - SCREEN_HEIGHT / 2.0, FOCAL_LENGTH);
	// Ray rotation
	if (yaw != 0) {
		d_ray = d_ray * R;
	}
	d_ray = glm::normalize(d_ray);

	// Trace a ray and retrieve the pixel color of the intersected object, after illumination and reflection bounces
	getColor(cameraPos, d_ray, NOT_STARTOBJIDX, color, 0, AIR_REFRACTIVE_INDEX);
	return color;
}

/* Draw the scene by tracing rays from the camera */
void draw()
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

/* Load the model then draw the scene, moving the camera
 * between each frame if required */
int main(int argc, char* argv[])
{
	// Initialization
	srand(time(NULL));
	screen = InitializeSDL(SCREEN_WIDTH, SCREEN_HEIGHT);
	R[1][1] = 1;
	updateRotationMatrix();
	int time_ms = SDL_GetTicks(), t2, dt; // Timer

	// Load model
	LoadTestModel(objects);
	// Initialize the light sources and the light surface
	initLights();

	while (NoQuitMessageSDL())
	{
		update();
		draw();

		// Display computation time
		t2 = SDL_GetTicks();
		dt = float(t2 - time_ms);
		time_ms = t2;
		cout << "Render time: " << dt << " ms." << endl;

		if (EXPORT_AND_EXIT) {
			break;
		}
	}

	// Export scene into .bmp file
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
