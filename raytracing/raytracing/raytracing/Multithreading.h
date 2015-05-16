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
	int widthOffset;
	int heightOffset;
	vec3*** screenPixels;
	
	/* Constructor */
	ThreadProcessData(vec3*** screenPixels, int width, int height, int widthOffset, int heightOffset, int rank)
		:screenPixels(screenPixels), width(width), height(height), widthOffset(widthOffset), heightOffset(heightOffset), rank(rank) {
	}
};

/* Anti-aliasing thread data */
struct ThreadAAData
{
	int rank;
	int size;
	int offset;
	list<pair<int, int>>* aliasedEdges;
	vec3*** screenPixels;

	/* Constructor */
	ThreadAAData(vec3*** screenPixels, list<pair<int, int>>* aliasedEdges, int size, int offset, int rank)
		:screenPixels(screenPixels), aliasedEdges(aliasedEdges), size(size), offset(offset), rank(rank) {
	}
};

extern int start_time;
extern bool DISPLAY_PROGRESS;
extern int PROGRESS_STEP;

void antiAliasingOnPixel(vec3** screenPixels, const pair<int, int>& p, int rank);
int coordsToRank(int p, int q, int P);
int getDataLength(int rank, int length, int N);
float getElapsedTime();
void getProcessesDistribution(int N, int *P, int *Q);
void process(vec3** screenPixels, int width, int height, int widthOffset, int heightOffset, int threadRank);
void multithreadedAntiAliasing(vec3** screenPixels, list<pair<int, int>>* aliasedEdges, int N);
void multithreadedProcess(vec3** screenPixels, int width, int height, int N, bool antiAliasing);
void rankToCoords(int r, int P, int * p, int * q);
DWORD WINAPI _threadAntiAliasing(LPVOID lpParam);
DWORD WINAPI _threadProcess(LPVOID lpParameter);

#endif
