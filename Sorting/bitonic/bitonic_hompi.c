#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <torc.h>

#define N 1024 * 1024 * 8
#define NUM_NODES 2
#define NUM_CORES 2 // TODO - Replace with TORC_WORKERS env

#define PLUS 0
#define MINUS 1


int qsort_enable = 0;


int cmp_func_plus  (const void * a, const void * b) {
   return ( *(int *) a - * (int *) b );
}


int cmp_func_minus (const void * a, const void * b) {
   return ( *(int * ) a - *(int * ) b );
}





#pragma ompix taskdef  in(cnt) inout (A[cnt]) in(flag)
void bitonic_merge(int *A, int cnt, int flag)
{

	int i, tmp;
	int y = cnt / 2;

	if (cnt == 1) return;
	
	for (i = 0; i < y; i++){
		if (flag == PLUS && A[i] > A[i + y]){
			tmp      = A[i];
			A[i]     = A[i + y];
			A[i + y] = tmp;
		}
		else if (flag == MINUS && A[i] < A[i + y]){
			/*swap*/
			tmp = A[i];
			A[i]     = A[i + y];
			A[i + y] = tmp;
		}
	}
	bitonic_merge(A,     y, flag);
	bitonic_merge(A + y, y, flag);
	
}






#pragma ompix taskdef  in(cnt) inout (A[cnt]) in(flag)
void bitonic_sort(int *A, int cnt, int flag)
{
	int i, y, my_rank, num_workers;

	if (cnt == 1) return;

	y =  cnt / 2;

	/*
	MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);

	printf("Bitonic sort task: cnt = %d flag = %d, node = %d\n",
		cnt, flag, my_rank); 

	printf("Creating task with y = %d\n", y);
	fflush(stdout);
	*/

#if 1

	if (y >= N / ((NUM_NODES * NUM_CORES )) ){

		num_workers = torc_num_workers();

		#pragma ompix task atnode(my_rank)
		bitonic_sort(A,     y, PLUS);

		#pragma ompix task atnode(my_rank)
		bitonic_sort(A + y, y, MINUS);

		#pragma ompix tasksync

	}else{

		//Split completed, sort serially

		if (qsort_enable)
		{
			if (flag)
				qsort(A, cnt, sizeof(A[0]), cmp_func_plus);
			else
				qsort(A, cnt, sizeof(A[0]), cmp_func_minus);
		}
		else
		{

			bitonic_sort(A,     y, PLUS);

			bitonic_sort(A + y, y, MINUS);

		}


	}


	bitonic_merge(A, cnt, flag);


#else

	bitonic_sort(A, low,    y, PLUS);

	bitonic_sort(A + y, low + y,y, MINUS);

	bitonic_merge(A, low,  cnt, flag);
	
#endif
	return;

}








int main(int argc, char *argv[])
{
	int i, node;
	int *A;
	int status, flag, step;
	
	double start, stop;


	if (argc > 1)
		if (! strcmp("qsort", argv[1]) )
			qsort_enable = 1;

	
	A = (int *) malloc (N * sizeof(int));

	for(i = 0; i < N; i++)
		A[i] = rand() % 1000000;

	step = N / (NUM_NODES);
	

	/* First phase */
	start = torc_gettime();

	for(i = 0, node = 0, flag = 0; i < N; i += step,
										  node++,
										  flag ^= 1){

		#pragma ompix task atnode(node)
		bitonic_sort(A + i, step, flag);

	}

	#pragma ompix tasksync

	
	step *= 2;

	while (step <= N){
		flag = PLUS;
		for (i = 0; i < N; i += step, flag ^= 1){

			#pragma ompix task // TODO - Cyclic distribution at nodes
			bitonic_merge(A + i, step, flag);
	
		}

		#pragma ompix tasksync
		step *= 2;
	}
	
	
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

	printf("Bitonic 2 Results:\n\nCPU Time = %.3lf sec\n", stop - start);
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
	return (0);
	
}
