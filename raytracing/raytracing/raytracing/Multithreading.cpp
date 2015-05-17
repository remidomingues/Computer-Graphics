#include <iostream>
#include <vector>
#include "Multithreading.h"

/* Trace a ray to process the next available pixel until no pixel is left */
void threadProcess(vec3*** screenPixels, pair<int, int>* nextAvailablePixel,
	int width, int height, int rank, HANDLE* mutex) {
	float tmp, elapsed;
	int currentSize, nextStep = PROGRESS_STEP, x, y;

	WaitForSingleObject(*mutex, INFINITE);
	while (nextAvailablePixel->second < height) {
		x = nextAvailablePixel->first;
		y = nextAvailablePixel->second;
		nextAvailablePixel->first = (x + 1) % width;
		if (nextAvailablePixel->first == 0)
			nextAvailablePixel->second += 1;
		ReleaseMutex(*mutex);

		(*screenPixels)[y][x] = traceRayFromCamera(x, y, rank);

		if (rank == 0) {
			tmp = (y + 1) * 100 / (float) height;
			if (DISPLAY_PROGRESS &&
				((!antiAliasingEnabled() && tmp >= nextStep) ||
				(antiAliasingEnabled() && tmp >= nextStep * 2))) {
				elapsed = float(getElapsedTime() - start_time);
				cout << nextStep << "% (" << elapsed / 1000.0f << " seconds)" << endl;
				nextStep += PROGRESS_STEP;
			}
		}
		WaitForSingleObject(*mutex, INFINITE);
	}
	ReleaseMutex(*mutex);
}

/* Call the process function to process a part of the scene */
DWORD WINAPI _threadProcess(LPVOID lpParam)
{
	ThreadProcessData *td = (ThreadProcessData*)lpParam;
	threadProcess(td->screenPixels, td->nextAvailablePixel, td->width, td->height, td->rank, td->mutex);
	return 0;
}

/* Process the scene (post-processing excluded) using N threads
* scattering pixels computations on a PxQ grid (PxQ = N) */
void multithreadedProcess(vec3** screenPixels, int width, int height, int N, bool antiAliasing) {
	vector<HANDLE> handles(N - 1);
	nextAvailablePixel.first = 0;
	nextAvailablePixel.second = 0;

	// Create threads
	for (int r = 0; r < N; ++r) {
		if (r != 0) {
			ThreadProcessData* data = new ThreadProcessData(&screenPixels, &nextAvailablePixel, width, height, r, &mutex);
			handles.push_back(CreateThread(NULL, 0, _threadProcess, data, 0, 0));
		}
	}

	// Process a part of the scene
	threadProcess(&screenPixels, &nextAvailablePixel, width, height, 0, &mutex);

	// Wait for threads
	cout << "Finishing processing..." << endl;
	for (HANDLE& handle : handles) {
		WaitForSingleObject(handle, INFINITE);
	}
}

/* Call the anti-aliasing function for each pixel assigned to the thread */
void threadAntiAliasing(vec3*** screenPixels, list<pair<int, int>>* aliasedEdges, int size, int rank, HANDLE* mutex) {
	float tmp, elapsed;
	int currentSize, nextStep = PROGRESS_STEP;
	pair<int, int> p;

	WaitForSingleObject(*mutex, INFINITE);
	while(!aliasedEdges->empty()) {
		currentSize = aliasedEdges->size();
		// Retrieve the first pixel of the list
		p = aliasedEdges->front();
		aliasedEdges->pop_front();
		ReleaseMutex(*mutex);

		// Progress display
		if (rank == 0) {
			tmp = (size - currentSize) * 100 / (float) size;
			if (DISPLAY_PROGRESS && tmp >= nextStep * 2) {
				elapsed = getElapsedTime();
				cout << 50 + nextStep << "% (" << elapsed / 1000.0f << " seconds)" << endl;
				nextStep += PROGRESS_STEP;
			}
		}

		// Process anti-aliasing
		antiAliasingOnPixel(*screenPixels, p, rank);

		WaitForSingleObject(*mutex, INFINITE);
	}
	ReleaseMutex(*mutex);
}

/* Call the anti-aliasing function for each pixel assigned to the thread */
DWORD WINAPI _threadAntiAliasing(LPVOID lpParam)
{
	ThreadAAData* td = (ThreadAAData*)lpParam;
	threadAntiAliasing(td->screenPixels, td->aliasedEdges, td->size, td->rank, td->mutex);
	return 0;
}

/*Apply anti-aliasing to the scene using N threads */
void multithreadedAntiAliasing(vec3** screenPixels, list<pair<int, int>>* aliasedEdges, int N) {
	int size = aliasedEdges->size();
	vector<HANDLE> handles(N - 1);

	// Create threads
	for (int r = 0; r < N; ++r) {
		if (r != 0) {
			ThreadAAData* data = new ThreadAAData(&screenPixels, aliasedEdges, size, r, &mutex);
			handles.push_back(CreateThread(NULL, 0, _threadAntiAliasing, data, 0, 0));
		}
	}

	// Process a part of the scene
	threadAntiAliasing(&screenPixels, aliasedEdges, size, 0, &mutex);

	// Wait for threads
	cout << "Finishing anti-aliasing..." << endl;
	for (HANDLE& handle : handles) {
		WaitForSingleObject(handle, INFINITE);
	}
}
