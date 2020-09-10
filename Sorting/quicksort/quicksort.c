#include <stdio.h>
#include <stdlib.h>

#include <torc.h>


#define N 1024 * 1024 * 32

#define NUM_CORES 4
#define LOCAL_CUTOFF (N / (NUM_CORES * 4) )

void quicksort(int *a, int size);



#pragma ompix taskdef in(len) inout(A[len])
void quicksort(int *A, int len) {

	int pivot, i, j;

	if (len < 2) return;
	
	pivot = A[len >> 1];



	for (i = 0, j = len - 1; ; i++, j--)
	{
		while (A[i] < pivot) i++;
		while (A[j] > pivot) j--;
	
		if (i >= j) break;
	
		int temp = A[i];
		A[i]     = A[j];
		A[j]     = temp;
	}
	
	if (len >= LOCAL_CUTOFF)
	{

		#pragma ompix task
		quicksort(A, i);

		#pragma ompix task
		quicksort(A + i, len - i);

		#pragma ompix tasksync
	}
	else
	{
		quicksort(A, i);
		quicksort(A + i, len - i);
	}

}


int main(){

	int i, status;
	int *A;
	double start, stop;


	A = (int *) malloc (N * sizeof(int));
	for (i = 0; i < N; i++)
		A[i] = rand() % 1500000;


	start = torc_gettime();

	quicksort(A, N);
	
	stop = torc_gettime();


	status = 0;
	for (i = 1; i < N; i++)
	{
		if (A[i] < A[i - 1])
		{
			status = i;
			break;
		}
	}

	printf("Quicksort results:\n\nCPU Time = %.3lf sec\n", stop - start);
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
	return 0;
}

