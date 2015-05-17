#ifndef MULTITHREADING_H
#define MULTITHREADING_H

#include <Windows.h>
#include <glm/glm.hpp>
#include <list>

using glm::vec3;
using namespace std;

/* Processing thread data*/
struct ThreadProcessData
{
	int rank;
	int width;
	int height;
	vec3*** screenPixels;
	pair<int, int>* nextAvailablePixel;
	HANDLE* mutex;
	
	/* Constructor */
	ThreadProcessData(vec3*** screenPixels, pair<int, int>* nextAvailablePixel, int width, int height, int rank, HANDLE* mutex)
		:screenPixels(screenPixels), nextAvailablePixel(nextAvailablePixel), width(width), height(height), rank(rank), mutex(mutex) {
	}
};

/* Anti-aliasing thread data */
struct ThreadAAData
{
	int rank;
	int size;
	list<pair<int, int>>* aliasedEdges;
	vec3*** screenPixels;
	HANDLE* mutex;

	/* Constructor */
	ThreadAAData(vec3*** screenPixels, list<pair<int, int>>* aliasedEdges, int size, int rank, HANDLE* mutex)
		:screenPixels(screenPixels), aliasedEdges(aliasedEdges), size(size), rank(rank), mutex(mutex) {
	}
};

int processIter = 0;
HANDLE mutex = CreateMutex(NULL, FALSE, NULL);
pair<int, int> nextAvailablePixel;

extern int start_time;
extern bool DISPLAY_PROGRESS;
extern int PROGRESS_STEP;

bool antiAliasingEnabled();
void antiAliasingOnPixel(vec3** screenPixels, const pair<int, int>& p, int rank);
float getElapsedTime();
void multithreadedAntiAliasing(vec3** screenPixels, list<pair<int, int>>* aliasedEdges, int N);
void multithreadedProcess(vec3** screenPixels, int width, int height, int N, bool antiAliasing);
vec3 traceRayFromCamera(float x, float y, int rank);
void threadAntiAliasing(vec3*** screenPixels, list<pair<int, int>>* aliasedEdges, int size, int rank, HANDLE* mutex);
void threadProcess(vec3*** screenPixels, pair<int, int>* nextAvailablePixel,
	int width, int height, int rank, HANDLE* mutex);
DWORD WINAPI _threadAntiAliasing(LPVOID lpParam);
DWORD WINAPI _threadProcess(LPVOID lpParameter);

#endif
