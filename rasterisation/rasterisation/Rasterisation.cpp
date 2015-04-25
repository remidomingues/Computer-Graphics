#define _USE_MATH_DEFINES

#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include <SDL.h>
#include <math.h>
#include "SDLauxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::vec3;
using glm::vec2;
using glm::ivec2;
using glm::mat3;

struct Pixel
{
	int x;
	int y;
	float zinv;
	vec3 illumination;
};

struct Vertex
{
	vec3 position;
	vec3 normal;
	vec3 reflectance;
};

typedef enum { RIGHT, LEFT, FORWARD, BACKWARD, UP, DOWN } Direction;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
int t;

/* Model */
vector<Triangle> triangles;

/* Screen */
const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL_Surface* screen;

/* Depth buffer*/
float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

/* Camera */
vec3 cameraPos(0, 0, -3.001);
float focalLength = SCREEN_WIDTH;
// Rotation angle controlling camera rotation around y-axis
float yaw = 0;
// Rotation constant - Angle update on the y axis for a rotation
float ROTATION = 0.1;
float TRANSLATION = 0.5;
// Rotation matrix
mat3 R = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);

/* Light */
vec3 lightPos(0, -0.5, -0.7);
vec3 lightPower = 14.f*vec3(1, 1, 1);
vec3 indirectLightPowerPerArea = 0.5f*vec3(1, 1, 1);

// ----------------------------------------------------------------------------
// FUNCTIONS

void PixelShader(const Pixel& p);
void DrawPolygon(const vector<Vertex>& vertices);
void DrawPolygonRows(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels);
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void DrawLineSDL(SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3>& vertices);
void Interpolate(Pixel a, Pixel b, vector<Pixel>& result);
void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result);
void UpdateRotationMatrix();
void Update();
float GetDistance(vec3 p1, vec3 p2);
void VertexShader(const vec3& v, ivec2& p);
void VertexShader(const Vertex& v, Pixel& p);
void initDepthBuffer();
void Draw();

int main( int argc, char* argv[] )
{
	LoadTestModel( triangles );
	screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT );
	t = SDL_GetTicks();	// Set start value for timer.

	R[1][1] = 1;
	UpdateRotationMatrix();

	while( NoQuitMessageSDL() )
	{
		Update();
		Draw();
	}

	SDL_SaveBMP( screen, "screenshot.bmp" );
	return 0;
}

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
	if (dir == Direction::DOWN) {
		return TRANSLATION * vec3(R[1][0], R[1][1], R[1][2]);
	}
	if (dir == Direction::RIGHT) {
		return TRANSLATION * vec3(R[0][0], R[0][1], R[0][2]);
	}
	if (dir == Direction::LEFT) {
		return TRANSLATION * -vec3(R[0][0], R[0][1], R[0][2]);
	}
	if (dir == Direction::FORWARD) {
		return TRANSLATION * vec3(R[2][0], R[2][1], R[2][2]);
	}
	if (dir == Direction::BACKWARD) {
		return TRANSLATION * -vec3(R[2][0], R[2][1], R[2][2]);
	}
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	cout << "Render time: " << dt << " ms." << endl;

	// Camera rotation using mouse
	int dx;
	int dy;
	if (SDL_GetRelativeMouseState(&dx, &dy) && SDL_BUTTON(SDL_BUTTON_LEFT)) {
		if (dx != 0) {
			yaw += ROTATION * dx / 50;
			UpdateRotationMatrix();
		}
	}


	/* Camera rotation and translation using keyboard */
	Uint8* keystate = SDL_GetKeyState(0);
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
		yaw -= ROTATION;

		UpdateRotationMatrix();
	}
	if (keystate[SDLK_RIGHT])
	{
		/* Move the camera to the right while rotating to the left */
		// Rotate camera to the left
		yaw += ROTATION;

		UpdateRotationMatrix();

		// Move camera to the right
		cameraPos += GetAxis(Direction::RIGHT);
	}

	if( keystate[SDLK_RCTRL] )
		;

	if( keystate[SDLK_w] )
		;

	if( keystate[SDLK_s] )
		;

	if( keystate[SDLK_d] )
		;

	if( keystate[SDLK_a] )
		;

	if( keystate[SDLK_e] )
		;

	if( keystate[SDLK_q] )
		;
}

float GetDistance(vec3 p1, vec3 p2) {
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

void VertexShader(const vec3& v, ivec2& p) {
	// Translation and rotation into the camera coordinate system
	vec3 v2 = (v - cameraPos) * R;

	// Perspective projection
	p.x = focalLength * v2.x / v2.z + SCREEN_WIDTH / 2;
	p.y = focalLength * v2.y / v2.z + SCREEN_HEIGHT / 2;
}

void VertexShader( const Vertex& v, Pixel& p ) {
	// Translation and rotation into the camera coordinate system
	vec3 v2 = (v.position - cameraPos) * R;

	// Perspective projection
	p.x = focalLength * v2.x / v2.z + SCREEN_WIDTH / 2;
	p.y = focalLength * v2.y / v2.z + SCREEN_HEIGHT / 2;
	p.zinv = 1 / v2.z;

	// Illumination
	vec3 r = lightPos - v.position;
	float distance = GetDistance(v.position, lightPos);
	float div = 4 * M_PI * distance * distance;
	float max = fmax(glm::dot(r, v.normal), 0);
	vec3 directLight(lightPower.x * max / div,
					lightPower.y * max / div,
					lightPower.z * max / div);
	p.illumination = (directLight + indirectLightPowerPerArea) * v.reflectance;
}

void PixelShader(const Pixel& p)
{
	int x = p.x;
	int y = p.y;
	if (p.zinv > depthBuffer[y][x])
	{
		depthBuffer[y][x] = p.zinv;
		PutPixelSDL(screen, x, y, p.illumination);
	}
}

void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result)
{
	int N = result.size();
	vec2 step = vec2(b - a) / float(max(N - 1, 1));
	vec2 current(a);
	for (int i = 0; i<N; ++i)
	{
		result[i] = current;
		current += step;
	}
}

void Interpolate(Pixel a, Pixel b, vector<Pixel>& result) {
	int N = result.size();
	float div = float(max(N - 1, 1));
	vec3 step;
	step.x = (b.x - a.x) / div;
	step.y = (b.y - a.y) / div;
	step.z = (b.zinv - a.zinv) / div;
	vec3 illuminationStep = (b.illumination - a.illumination) / div;
	
	vec3 current(a.x, a.y, a.zinv);
	vec3 curIllumination = a.illumination;

	for (int i = 0; i < N; ++i)
	{
		Pixel p;
		p.x = current.x;
		p.y = current.y;
		p.zinv = current.z;
		p.illumination = curIllumination;
		result[i] = p;

		current += step;
		curIllumination += illuminationStep;
	}
}

/* Interpolate pixel positions between two vertices to draw a line between them */
void DrawLineSDL(SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color) {
	// Interpolate
	ivec2 delta = glm::abs(a - b);
	int pixels = glm::max(delta.x, delta.y) + 1;
	vector<ivec2> line(pixels);
	Interpolate(a, b, line);

	// Draw
	for (ivec2 v : line) {
		PutPixelSDL(surface, v.x, v.y, color);
	}
}

void DrawPolygonEdges(const vector<vec3>& vertices)
{
	int V = vertices.size();

	// Transform each vertex from 3D world position to 2D image position:
	vector<ivec2> projectedVertices(V);
	for (int i = 0; i<V; ++i)
	{
		VertexShader(vertices[i], projectedVertices[i]);
	}

	// Loop over all vertices and draw the edge from it to the next vertex:
	for (int i = 0; i<V; ++i)
	{
		int j = (i + 1) % V; // The next vertex
		vec3 color(1, 1, 1);
		DrawLineSDL(screen, projectedVertices[i], projectedVertices[j], color);
	}
}

void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels)
{
	int y;

	// Compute the number of rows the polygon occupies
	int min_y = numeric_limits<int>::max();
	int max_y = -numeric_limits<int>::max();
	for (const Pixel& vec : vertexPixels) {
		if (vec.y < min_y)
			min_y = vec.y;
		if (vec.y > max_y)
			max_y = vec.y;
	}
	int rows = abs(max_y - min_y) + 1;

	// Resize pixels arrays
	leftPixels.resize(rows);
	rightPixels.resize(rows);

	// Initialize the pixels coordinates
	for (int i = 0; i < rows; ++i)
	{
		y = min_y + i;
		leftPixels[i].x = numeric_limits<int>::max();
		leftPixels[i].y = y;
		rightPixels[i].x = -numeric_limits<int>::max();
		rightPixels[i].y = y;
	}

	int idx;
	// Loop through all edges of the polygon
	for (int i = 0; i < vertexPixels.size(); ++i) {
		for (int j = i + 1; j < vertexPixels.size(); ++j) {
			// Iterpolate the coordinates of each pixel of the polygon edge
			rows = abs(vertexPixels[i].y - vertexPixels[j].y) + 1;
			vector<Pixel> line(rows);
			Interpolate(vertexPixels[i], vertexPixels[j], line);

			// Update the right and left pixels if required
			for (const Pixel& vec : line) {
				idx = vec.y - min_y;
				if (idx >= 0 && idx < leftPixels.size())  {
					if (vec.x < leftPixels[idx].x) {
						leftPixels[idx].x = vec.x;
						leftPixels[idx].zinv = vec.zinv;
						leftPixels[idx].illumination = vec.illumination;
					}
					if (vec.x > rightPixels[idx].x) {
						rightPixels[idx].x = vec.x;
						rightPixels[idx].zinv = vec.zinv;
						rightPixels[idx].illumination = vec.illumination;
					}
				}
			}
		}
	}
}

void DrawPolygonRows(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels) {
	int y, size;
	float z_step, z_current;
	vector<Pixel> row;

	for (int i = 0; i < leftPixels.size(); ++i) {
		y = leftPixels[i].y;
		if (y > 0 && y < SCREEN_HEIGHT) {
			size = abs(rightPixels[i].x - leftPixels[i].x) + 1;
			row.resize(size);
			Interpolate(leftPixels[i], rightPixels[i], row);

			for (Pixel& p : row) {
				if (p.x > 0 && p.x < SCREEN_WIDTH) {
					PixelShader(p);
				}
			}
		}
	}
}

void DrawPolygon(const vector<Vertex>& vertices)
{
	int V = vertices.size();
	vector<Pixel> vertexPixels(V);

	for (int i = 0; i<V; ++i)
		VertexShader(vertices[i], vertexPixels[i]);

	vector<Pixel> leftPixels;
	vector<Pixel> rightPixels;
	ComputePolygonRows(vertexPixels, leftPixels, rightPixels);
	DrawPolygonRows(leftPixels, rightPixels);
}

void initDepthBuffer() {
	for (int y = 0; y<SCREEN_HEIGHT; ++y)
		for (int x = 0; x<SCREEN_WIDTH; ++x)
			depthBuffer[y][x] = 0;
}

void Draw()
{
	initDepthBuffer();
	SDL_FillRect(screen, 0, 0);
	if (SDL_MUSTLOCK(screen))
		SDL_LockSurface(screen);

	for (int i = 0; i<triangles.size(); ++i)
	{
		vector<Vertex> vertices(3);
		vertices[0].position = triangles[i].v0;
		vertices[0].normal = triangles[i].normal;
		vertices[0].reflectance = triangles[i].color;
		vertices[1].position = triangles[i].v1;
		vertices[1].normal = triangles[i].normal;
		vertices[1].reflectance = triangles[i].color;
		vertices[2].position = triangles[i].v2;
		vertices[2].normal = triangles[i].normal;
		vertices[2].reflectance = triangles[i].color;
		DrawPolygon(vertices);
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_UpdateRect(screen, 0, 0, 0, 0);
}