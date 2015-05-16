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
DWORD WINAPI threadFunction(LPVOID lpParam)
{
	ThreadData *td = (ThreadData*)lpParam;
	process(*(td->screenPixels), td->width, td->height, td->widthOffset, td->heightOffset, td->rank);
	return 0;
}

/* Process the scene (post-processing excluded) using N threads
* scattering pixels computations on a PxQ grid (PxQ = N) */
void multithreadedProcess(vec3** screenPixels, int width, int height, int N) {
	int P, Q, myWidth, myHeight;
	int tWidth, tHeight, tWidthOffset = 0, tHeightOffset = 0, rank = 0;
	vector<HANDLE> handles(N - 1);
	getProcessesDistribution(N, &P, &Q);

	if (iter == 0) {
		cout << N << " threads will be distributed on a " << P << "x" << Q << " grid" << endl;
		++iter;
	}

	// Create threads
	for (int q = 0; q < Q; ++q) {
		for (int p = 0; p < P; ++p) {
			tWidth = getDataLength(p, width, P);
			tHeight = getDataLength(q, height, Q);

			cout << "rank=" << rank << " tWidthOffset=" << tWidthOffset << " tHeightOffset=" << tHeightOffset << " twidth=" << tWidth << " tHeight=" << tHeight << endl;

			if (p != 0 || q != 0) {
				ThreadData* data = new ThreadData(&screenPixels, tWidth, tHeight, tWidthOffset, tHeightOffset, rank);
				handles.push_back(CreateThread(NULL, 0, threadFunction, data, 0, 0));
			} else {
				myWidth = tWidth;
				myHeight = tHeight;
			}

			tWidthOffset += tWidth;
			++rank;
		}
		tWidthOffset = 0;
		tHeightOffset += tHeight;
	}

	// Process a part of the scene
	process(screenPixels, myWidth, myHeight, 0, 0, 0);

	// Wait for threads
	for (HANDLE& handle : handles) {
		WaitForSingleObject(handle, INFINITE);
	}
}
