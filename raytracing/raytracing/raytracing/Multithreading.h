#ifndef MULTITHREADING_H
#define MULTITHREADING_H

#include <Windows.h>
#include <glm/glm.hpp>

using glm::vec3;

/* Thread data */
struct ThreadData
{
	int rank;
	int width;
	int height;
	int widthOffset;
	int heightOffset;
	vec3*** screenPixels;
	
	/* Constructor */
	ThreadData(vec3*** screenPixels, int width, int height, int widthOffset, int heightOffset, int rank)
		:screenPixels(screenPixels), width(width), height(height), widthOffset(widthOffset), heightOffset(heightOffset), rank(rank) {
	}
};

int coordsToRank(int p, int q, int P);
int getDataLength(int rank, int length, int N);
void getProcessesDistribution(int N, int *P, int *Q);
void process(vec3** screenPixels, int width, int height, int widthOffset, int heightOffset, int threadRank);
void multithreadedProcess(vec3** screenPixels, int width, int height, int N);
void rankToCoords(int r, int P, int * p, int * q);
DWORD WINAPI threadFunction(LPVOID lpParameter);

#endif
