#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

//Source: http://dzmitryhuba.blogspot.com/2010/10/parallel-merge-sort.html

#include <torc.h>

#define N 1024 * 1024 * 1024

#define NUM_CORES 4
#define CUTOFF N / (NUM_CORES * 16)
int n_gl = N;

int binary_search(int *arr, int low, int high, int elem)
{
	int mid;
	
	if (high + 1 > low) high = high + 1;
	else high = low;
	
	while (low < high){
		
		mid = (low + high) >> 1;
		if (arr[mid] < elem)
			low = mid + 1;
		else
			high = mid;
	}
	return low;
	
}



void sequential_merge(int *to, int *temp, int lowX, int highX,
										  int lowY, int highY,
										  int lowTo)
{
	int highTo;
	highTo = lowTo + highX - lowX + highY - lowY + 1;
	
	for (; lowTo <= highTo; lowTo++){
		
		if      (lowX > highX)
			to[lowTo] = temp[lowY++];
		else if (lowY > highY)
			to[lowTo] = temp[lowX++];
		else
		{
			if (temp[lowX] < temp[lowY])
				to[lowTo] = temp[lowX++];
			else
				to[lowTo] = temp[lowY++];
		}
		
	}
	
	
	
	return;
}





#pragma ompix taskdef in(lowX, highX, lowY, highY) inout(to[n_gl], temp[n_gl])
void parallel_merge(int *to, int *temp, int lowX, int highX,
									   int lowY, int highY,
					int lowTo)
{
	int midX, midY, midTo, lengthX, lengthY;
	
	
	lengthX = highX - lowX + 1;
	lengthY = highY - lowY + 1;
	

	if (lengthX + lengthY <= CUTOFF)
	{
		sequential_merge(to, temp, lowX, highX, lowY, highY, lowTo);
		return;
	}
	
	if (lengthX < lengthY)
	{
		parallel_merge(to, temp, lowY, highY, lowX, highX, lowTo);
		return;
	}
	
	// Medians if all elements are unique
	midX = (lowX + highX) >> 1;
	
	midY = binary_search(temp, lowY, highY, temp[midX]);
	
	midTo     = lowTo + midX - lowX + midY - lowY;
	to[midTo] = temp[midX];
	
	#pragma ompix task
	parallel_merge(to, temp, lowX, midX - 1, lowY, midY - 1, lowTo);
	
	#pragma ompix task
	parallel_merge(to, temp, midX + 1, highX, midY, highY, midTo + 1);
	
	#pragma ompix tasksync
	
	return;
}



int main(){
	
	int i, *A, *R, *temp, status;
	double start, stop;
	
	A = (int *) malloc (N * sizeof(int));
	R = (int *) malloc (N * sizeof(int));
	
	temp = (int *) malloc (N * sizeof(int));
	
	for (i = 0; i < (N >> 1); i++){
		A[i]       = (i + 1) * 2;
		A[i + N/2] = (i + 1) * 3;
	}
	
	start = torc_gettime();
	
	parallel_merge(R, A, 0, (N >> 1) - 1, (N >> 1), N - 1, 0);
	
	stop = torc_gettime();
	
	for (i = 0; i < N; i++)
		1;//printf("R[%d] = %d\n", i, R[i]);
	
	status = 0;
	for (i = 1; i < N; i++)
	{
		if (R[i] < R[i - 1])
		{
			status = i;
			break;
		}
	}

	printf("Parallel merge Results:\n\nCPU Time = %.3lf sec\n", stop - start);
	printf("STATUS = ");
	if (status == 0)
	{
		printf("OK!!!\n");
		
	}else{
		printf("ERROR!!!!! @ element(%d)\n",status);
		//for(i = 0; i < N; i++)
			//printf("A[%d] = %d\n", i, A[i]);
		
	}
	
	
	
	free(A);
	free(R);
	return (0);
}
