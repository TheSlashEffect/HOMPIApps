#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <torc.h>


/* 
Serial algorithm source:
https://www.geeksforgeeks.org/merge-sort/
*/
#define N 1024 * 1024 * 32

#define NUM_NODES 2
#define NUM_CORES 4     // To be used later in Hybrid Parallelism
#define SPLT_THRESHOLD 1// Log2(NUM_NODES), provided N % NUM_NODES = 0


#define LOCAL_CUTOFF (N / (NUM_NODES * NUM_CORES) )

void mergesort_tree_task (int *a,  int size, int depth);
void mergesort_local_task(int *a,  int size);
void merge_sort_serial   (int a[], int l,    int r);
void merge               (int a[], int l,    int m, int r);
void print_array         (int a[], int size);
int  check_sort          (int a[], int size);


int n_gl = N, qsort_enable = 0, omp_enable = 0;


int cmp_func  (const void * a, const void * b) {
   return ( *(int *) a - * (int *) b );
}


void merge(int arr[], int l, int m, int r){

	int i, j, k; // Indexes for A, B, result arrays, respectively.
	int n1, n2;

	int *a, *b;

	n1 = m - l + 1;
	n2 = r - m;

	a = (int *) malloc (n1 * sizeof(int));
	b = (int *) malloc (n2 * sizeof(int));

	//printf("merge with l = %d, m = %d, r = %d\n", l, m , r);

	// Copy arr's subarray's contents to a and b arrays
	// Could be done in one loop, but I'll leave it to
	// the compiler's optimizer
	for (i = 0; i < n1; i++)
		a[i] = arr[l + i];
	for (j = 0; j < n2; j++)
		b[j] = arr[m + 1 + j];

	i = 0, j = 0, k = l;


	while (i < n1 && j < n2){

		if (a[i] <= b[j])
			arr[k] = a[i++];
		else
			arr[k] = b[j++];
		k++;
	}

	// What's left of A
	while (i < n1)
		arr[k++] = a[i++];
	
	// What's left of B
	while (j < n2)
		arr[k++] = b[j++];


	free(a);
	free(b);

}


// Will be in working order soon enough
void omp_mergesort(int a[], int size, int threads){

	printf("omp_merge of size %d\n", size);

	if (threads == 1){
		qsort(a, size, sizeof(a[0]), cmp_func);
	}else if (threads > 1){


		#pragma omp task
		omp_mergesort(a, (size >> 1), (threads >> 1));

		#pragma omp task
		omp_mergesort(a + (size >> 1), size - (size >> 1),
								threads - (threads >> 1));

		#pragma omp taskwait

		merge(a, 0, size / 2, size - 1); // Parallel merge coming soon

	}

}





void serial_sort(int *a, int size){

	if (qsort_enable)
		qsort(a, size, sizeof(a[0]), cmp_func);
	else if (omp_enable)
		omp_mergesort(a, size, NUM_CORES);
	else
		merge_sort_serial(a, 0, size - 1);
	

}



/* Split inside node to take advantage of multiple
CPU's */
#pragma ompix taskdef in(size) inout(a[size])
void mergesort_local_task(int *a, int size){

	int c, my_rank;

	// If we reached the task leaf (single core from now on)
	if (size <= LOCAL_CUTOFF){

		serial_sort(a, size);

	}else{ // Must move down local tasktree

		MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);

		#pragma ompix task atnode(my_rank)
		mergesort_local_task(a,  (size >> 1));

		#pragma ompix task atnode(my_rank)
		mergesort_local_task(a + (size >> 1), size - (size >> 1));

		#pragma ompix tasksync // Wait for above tasks to finish

		merge(a, 0, (size - 1)/ 2, size - 1);
		

		
		if (c = check_sort(a, size)){
			printf("Merge error\n");
			exit(-1);
		}


	}


	return;
}



/* Create a merge sort tree across nodes */
#pragma ompix taskdef in(size, depth) inout(a[size])
void mergesort_tree_task(int *a, int size, int depth){

	int c, n0, n1, my_rank;

	// If we reached leaf
	if (depth == SPLT_THRESHOLD){

		//printf("Depth = %d\n", depth);
		MPI_Comm_rank(MPI_COMM_WORLD, &my_rank);

		mergesort_local_task(a, size);

	}else{ // Must move down task tree

		MPI_Comm_rank(MPI_COMM_WORLD, &n0); // Self
		n1 = n0 + (1 << (depth));

		printf("About to split to nodes %d and %d\n", n0, n1);


		#pragma ompix task atnode(n0)
		mergesort_tree_task(a,  (size >> 1), depth + 1);


		#pragma ompix task atnode(n1)
		mergesort_tree_task(a + (size >> 1), size - (size >> 1), depth + 1);


		#pragma ompix tasksync // Wait for above tasks to finish

		//printf("About to merge size %d\n", size);
		//print_array(a, size);
		merge(a, 0, (size - 1)/ 2, size - 1);
		

		
		if (c = check_sort(a, size)){
			printf("Merge error\n");
			exit(-1);
		}


	}


	return;
}










// Reach this (local program) when we're done splitting
void merge_sort_serial(int a[], int l, int r){

	int m, my_rank;

	if (l < r)
	{
		// Avoids overflow for large values
		m = l + (r - l) / 2;

		merge_sort_serial(a,     l, m);
		merge_sort_serial(a, m + 1, r);

		merge(a, l, m, r);
	}


}






void print_array(int a[], int size){

	int i;

	for (i = 0; i < size; i++)
		printf("A[%d] = %d\n", i, a[i]);
	

}




int check_sort(int a[], int size){

	int i, err;

	err = 0;
	for (i = 1; i < size; i++){
		if (a[i - 1] > a[i]){
			err = i;
			break;
		}
	}


	if (err)
		printf("Error at pos %d\n", err);
	else{
		if (size == N)
			printf("Success\n");
	}

	return err;
}


int main(int argc, char *argv[]){

	int i, err, *A;
	double start, stop;

	if (argc > 1)
		if      (! strcmp("qsort", argv[1]) )
			qsort_enable = 1;
		else if (! strcmp("omp", argv[1]))
		{
			//omp_enable   = 1;
			printf("OpenMP currently unavailable, ");
			printf("using merge sort instead\n");
		}


	A = (int *) malloc (N * sizeof (int));
	for (i = 0; i < N; i++)
		A[i] = rand() % 150000;



	start = torc_gettime();

	mergesort_tree_task(A, N, 0);

	stop = torc_gettime();

	printf("Merge sort CPU Time = %.3lf sec\n", stop - start);

	// Check if sorting was performed correctly
	check_sort(A, N);
	

	return (0);
}
