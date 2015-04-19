#define _USE_MATH_DEFINES

#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include <math.h>
#include "SDLauxiliary.h"
#include "TestModel.h"


using namespace std;
using glm::vec3;
using glm::mat3;

struct Intersection
{
	vec3 position;
	float distance;
	int triangleIndex;
};

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
int time;

/* Screen */
const int SCREEN_WIDTH = 150;
const int SCREEN_HEIGHT = 150;
SDL_Surface* screen;

/* Model */
vector<Triangle> triangles;

/* Camera */
// Focal length
float focalLength = SCREEN_WIDTH;
// Position
vec3 cameraPos(0, 0, -3);
// Rotation matrix
mat3 R = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);
// Current rotation angle
float yaw = 0;
// Rotation constant - Angle update on the y axis for a rotation
float ROTATION = 0.3;
// Translation
float TRANSLATION = 1;

// ----------------------------------------------------------------------------
// FUNCTIONS

void updateRotationMatrix();
bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle>& triangles, Intersection& closestIntersection);
void Update();
void Draw();

int main( int argc, char* argv[] )
{
	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	R[1][1] = 1;
	updateRotationMatrix();
	time = SDL_GetTicks();	// Set start value for timer.

	// Initialize the model
	LoadTestModel(triangles);

	while( NoQuitMessageSDL() )
	{
		Update();
		Draw();
	}

	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}

/* Return x (t u v) from ray start and direction. t distance, u and v triangle delimiters */
vec3 GetIntersection(vec3 start, vec3 dir, Triangle triangle) {
	vec3 v0 = triangle.v0;
	vec3 v1 = triangle.v1;
	vec3 v2 = triangle.v2;
	vec3 e1 = v1 - v0;
	vec3 e2 = v2 - v0;
	vec3 b = start - v0;
	mat3 A(-dir, e1, e2);
	return glm::inverse(A) * b;
}

/* Return true if the ray intersects with a triangle, false otherwise. If true, closestIntersection is initialized with the result */
bool ClosestIntersection(vec3 start, vec3 dir, const vector<Triangle>& triangles, Intersection& closestIntersection) {
	closestIntersection.distance = std::numeric_limits<float>::max();;
	vec3 tmp;
	
	for (int i = 0; i < triangles.size(); ++i) {
		tmp = GetIntersection(start, dir, triangles[i]);
		if (tmp.x < closestIntersection.distance && tmp.x >= 0 && tmp.y > 0 && tmp.z > 0 && tmp.y + tmp.z < 1) {
			closestIntersection.distance = tmp.x;
			closestIntersection.position = cameraPos + tmp.x * dir;
			closestIntersection.triangleIndex = i;
		}
	}

	return closestIntersection.distance != std::numeric_limits<float>::max();
}

/* Update the rotation matrix */
void updateRotationMatrix()
{
	R[0][0] = cos(yaw);
	R[0][2] = sin(yaw);
	R[2][0] = -sin(yaw);
	R[2][2] = cos(yaw);
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2 - time);
	time = t2;
	cout << "Render time: " << dt << " ms." << endl;

	// Move and rotate camera
	Uint8* keystate = SDL_GetKeyState(0);
	if (keystate[SDLK_UP])
	{
		// Move camera forward
		vec3 forward(-R[2][0], R[2][1], R[2][2]);
		cameraPos += TRANSLATION * forward;
	}
	if (keystate[SDLK_DOWN])
	{
		// Move camera backward
		vec3 forward(-R[2][0], R[2][1], R[2][2]);
		cameraPos += TRANSLATION * -forward;
	}
	if (keystate[SDLK_LEFT])
	{
		/* Move the camera to the left while rotating to the right */
		// Rotate camera to the right
		yaw += ROTATION;

		updateRotationMatrix();

		// Move camera to the left
		vec3 right(R[0][0], R[0][1], -R[0][2]);
		cameraPos += TRANSLATION * -right;
	}
	if (keystate[SDLK_RIGHT])
	{
		/* Move the camera to the right while rotating to the left */
		// Rotate camera to the left
		yaw -= ROTATION;

		updateRotationMatrix();

		// Move camera to the right
		vec3 right(R[0][0], R[0][1], -R[0][2]);
		cameraPos += TRANSLATION * right;
	}
}

void Draw()
{
	Intersection closestIntersection;
	vec3 black(0, 0, 0);

	if( SDL_MUSTLOCK(screen) )
		SDL_LockSurface(screen);

	for( int y=0; y<SCREEN_HEIGHT; ++y )
	{
		for( int x=0; x<SCREEN_WIDTH; ++x )
		{
			// Ray direction
			vec3 d_ray(x - SCREEN_WIDTH / 2.0, y - SCREEN_HEIGHT / 2.0, focalLength);
			// Ray rotation
			if (yaw != 0) {
				d_ray = d_ray * R;
			}
			// Fill a pixel with the color of the closest triangle intersecting the ray, black otherwise
			if (ClosestIntersection(cameraPos, d_ray, triangles, closestIntersection)) {
				PutPixelSDL(screen, x, y, triangles[closestIntersection.triangleIndex].color);
			}
			else {
				PutPixelSDL(screen, x, y, black);
			}

		}
	}

	if( SDL_MUSTLOCK(screen) )
		SDL_UnlockSurface(screen);

	SDL_UpdateRect( screen, 0, 0, 0, 0 );
}