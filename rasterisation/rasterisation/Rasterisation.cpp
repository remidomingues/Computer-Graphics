#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include <SDL.h>
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
};

typedef enum { RIGHT, LEFT, FORWARD, BACKWARD, UP, DOWN } Direction;

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
int t;
int dx;
int dy;
vec3 currentColor;

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
float ROTATION = 0.3;
// Rotation matrix
mat3 R = mat3(0, 0, 0, 0, 0, 0, 0, 0, 0);

// ----------------------------------------------------------------------------
// FUNCTIONS

void DrawPolygon(const vector<vec3>& vertices);
void DrawPolygonRows(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels);
void ComputePolygonRows(const vector<Pixel>& vertexPixels, vector<Pixel>& leftPixels, vector<Pixel>& rightPixels);
void DrawLineSDL(SDL_Surface* surface, ivec2 a, ivec2 b, vec3 color);
void DrawPolygonEdges(const vector<vec3>& vertices);
void Interpolate(Pixel a, Pixel b, vector<Pixel>& result);
void Interpolate(ivec2 a, ivec2 b, vector<ivec2>& result);
void UpdateRotationMatrix();
void Update();
void VertexShader(const vec3& v, Pixel& p);
void VertexShader(const vec3& v, ivec2& p);
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
		return -vec3(R[1][0], R[1][1], R[1][2]);
	}
	if (dir == Direction::DOWN) {
		return vec3(R[1][0], R[1][1], R[1][2]);
	}
	if (dir == Direction::RIGHT) {
		return vec3(R[0][0], R[0][1], R[0][2]);
	}
	if (dir == Direction::LEFT) {
		return -vec3(R[0][0], R[0][1], R[0][2]);
	}
	if (dir == Direction::FORWARD) {
		return vec3(R[2][0], R[2][1], R[2][2]);
	}
	if (dir == Direction::BACKWARD) {
		return -vec3(R[2][0], R[2][1], R[2][2]);
	}
}

void Update()
{
	// Compute frame time:
	int t2 = SDL_GetTicks();
	float dt = float(t2-t);
	t = t2;
	//cout << "Render time: " << dt << " ms." << endl;

	// Camera rotation using mouse
	if (SDL_GetRelativeMouseState(&dx, &dy) && SDL_BUTTON(SDL_BUTTON_LEFT)) {
		if (dx != 0) {
			yaw += ROTATION * dx / 100.0;
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

void VertexShader(const vec3& v, ivec2& p) {
	// Translation and rotation into the camera coordinate system
	vec3 v2 = (v - cameraPos) * R;

	// Perspective projection
	p.x = focalLength * v2.x / v2.z + SCREEN_WIDTH / 2;
	p.y = focalLength * v2.y / v2.z + SCREEN_HEIGHT / 2;
}

void VertexShader( const vec3& v, Pixel& p ) {
	// Translation and rotation into the camera coordinate system
	vec3 zinv(v);
	vec3 v2 = (zinv - cameraPos) * R;

	// Perspective projection
	p.x = focalLength * v2.x / v2.z + SCREEN_WIDTH/2;
	p.y = focalLength * v2.y / v2.z + SCREEN_HEIGHT / 2;
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
	vec3 step;
	step.x = (b.x - a.x) / float(max(N - 1, 1));
	step.y = (b.y - a.y) / float(max(N - 1, 1));
	step.z = (b.zinv - a.zinv) / float(max(N - 1, 1));
	
	vec3 current(a.x, a.y, a.zinv);

	for (int i = 0; i < N; ++i)
	{
		Pixel p;
		p.x = current.x;
		p.y = current.y;
		p.zinv = current.z;
		result[i] = p;
		current += step;
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
	int rows = abs(abs(max_y) - abs(min_y) + 1);

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
			rows = abs(abs(vertexPixels[i].y) - abs(vertexPixels[j].y)) + 1;
			vector<Pixel> line(rows);
			Interpolate(vertexPixels[i], vertexPixels[j], line);

			// Update the right and left pixels if required
			for (const Pixel& vec : line) {
				idx = vec.y - min_y;
				if (idx >= 0 && idx < leftPixels.size())  {
					if (vec.x < leftPixels[idx].x)
						leftPixels[idx].x = vec.x;
					if (vec.x > rightPixels[idx].x)
						rightPixels[idx].x = vec.x;
				}
			}
		}
	}
}

void DrawPolygonRows(const vector<Pixel>& leftPixels, const vector<Pixel>& rightPixels) {
	int y, n;
	for (int i = 0; i < leftPixels.size(); ++i) {
		y = leftPixels[i].y;
		n = abs(abs(leftPixels[i].x) - abs(rightPixels[i].x)) + 1;
		
		for (int x = leftPixels[i].x; x < leftPixels[i].x + n; ++x) {
			//if (< depthBuffer[y][x]) {
				PutPixelSDL(screen, x, y, currentColor);
				//depthBuffer[y][x] = ;
			//}
		}
	}
}

void DrawPolygon(const vector<vec3>& vertices)
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
		currentColor = triangles[i].color;
		vector<vec3> vertices(3);
		vertices[0] = triangles[i].v0;
		vertices[1] = triangles[i].v1;
		vertices[2] = triangles[i].v2;
		DrawPolygon(vertices);
	}

	if (SDL_MUSTLOCK(screen))
		SDL_UnlockSurface(screen);
	SDL_UpdateRect(screen, 0, 0, 0, 0);
}