#include <iostream>
#include <vector>
#include "Multithreading.h"

using namespace std;

int iter = 0;

/* Initialize the values P and Q in order to distribute
* N threads on a P*Q grid */
void getProcessesDistribution(int N, int *P, int *Q) {
	int i;

	*Q = 1;
	*P = N;

	for (i = 2; i < N; ++i) {
		if (i >= *P) {
			break;
		}

		if (N % i == 0) {
			*Q = i;
			*P = N / i;
		}
	}
}

/* Return the thread rank given its coordinates on the grid */
int coordsToRank(int p, int q, int P) {
	return q * P + p;
}

/* Initialize the threads coordinates in the grid given its rank */
void rankToCoords(int r, int P, int * p, int * q) {
	*q = r / P;
	*p = r % P;
}

/* Return the data length (height or width) for a given process
* according to the total length dedicated to every process
* rank is the coordinate p or q */
int getDataLength(int rank, int length, int N) {
	int size = length / N;

	if (rank < length % N) {
		++size;
	}

	return size;
}

/* Call the process function to process a part of the scene */
DWORD WINAPI _threadProcess(LPVOID lpParam)
{
	ThreadProcessData *td = (ThreadProcessData*)lpParam;
	process(*(td->screenPixels), td->width, td->height, td->widthOffset, td->heightOffset, td->rank);
	return 0;
}

/* Process the scene (post-processing excluded) using N threads
* scattering pixels computations on a PxQ grid (PxQ = N) */
void multithreadedProcess(vec3** screenPixels, int width, int height, int N, bool antiAliasing) {
	int P, Q, myWidth, myHeight;
	int tWidth, tHeight, tWidthOffset = 0, tHeightOffset = 0, rank = 1;
	vector<HANDLE> handles(N - 1);
	HANDLE mutex = CreateMutex(NULL, FALSE, NULL);
	getProcessesDistribution(N, &P, &Q);

	if (iter == 0) {
		cout << "Processing scene using " << N << " threads distributed on a " << P << "x" << Q << " grid..." << endl;
		++iter;
	}

	// Create threads
	for (int q = 0; q < Q; ++q) {
		for (int p = 0; p < P; ++p) {
			tWidth = getDataLength(p, width, P);
			tHeight = getDataLength(q, height, Q);

			if (p != 0 || q != 0) {
				ThreadProcessData* data = new ThreadProcessData(&screenPixels, tWidth, tHeight, tWidthOffset, tHeightOffset, rank, &mutex);
				handles.push_back(CreateThread(NULL, 0, _threadProcess, data, 0, 0));
				++rank;
			} else {
				myWidth = tWidth;
				myHeight = tHeight;
			}

			tWidthOffset += tWidth;
		}
		tWidthOffset = 0;
		tHeightOffset += tHeight;
	}

	// Process a part of the scene
	process(screenPixels, myWidth, myHeight, 0, 0, 0);

	// Wait for threads
	cout << "Finishing processing..." << endl;
	for (HANDLE& handle : handles) {
		WaitForSingleObject(handle, INFINITE);
	}
}

/* Call the anti-aliasing function for each pixel assigned to the thread */
void threadAntiAliasing(vec3*** screenPixels, list<pair<int, int>>* aliasedEdges, int size, int rank, HANDLE* mutex) {
	double tmp;
	int currentSize, elapsed, nextStep = PROGRESS_STEP;
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
				if (elapsed >= 100000) {
					cout << 50 + nextStep << "% (" << elapsed / 1000 << " seconds)" << endl;
				}
				else {
					cout << 50 + nextStep << "% (" << elapsed << " ms)" << endl;
				}
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
	int tSize, tOffset = 0, mySize, size = aliasedEdges->size();
	vector<HANDLE> handles(N - 1);
	HANDLE mutex = CreateMutex(NULL, FALSE, NULL);

	// Create threads
	for (int r = 0; r < N; ++r) {
		tSize = getDataLength(r, size, N);

		if (r != 0) {
			ThreadAAData* data = new ThreadAAData(&screenPixels, aliasedEdges, tSize, r, &mutex);
			handles.push_back(CreateThread(NULL, 0, _threadAntiAliasing, data, 0, 0));
		}
		else {
			mySize = tSize;
		}

		tOffset += tSize;
	}

	// Process a part of the scene
	threadAntiAliasing(&screenPixels, aliasedEdges, mySize, 0, &mutex);

	// Wait for threads
	cout << "Finishing anti-aliasing..." << endl;
	for (HANDLE& handle : handles) {
		WaitForSingleObject(handle, INFINITE);
	}
}
