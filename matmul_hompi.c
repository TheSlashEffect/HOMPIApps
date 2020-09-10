#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <omp.h>

#include <torc.h>

#define N 1024      //Matrix size
#define NTASK M*M   //Number of submatrices
#define L 4         //Number of worker threads (optimally equal to number of physical cores,
					//or physical threads if multithreading technology is present)
#define NUM_OF_TESTS 1


//Function & data declarations
int **A, *A_data, **B, *B_data, **C, *C_data;
int *res_data, **res; // Reusable across tasks


int readmat(char *fname, int *mat, int n),
    writemat(char *fname, int *mat, int n);

int M, S; // Must inform every node of these dimensions

void taskexecute(int wid, int *results);
void *threadworker(void *arg);

//Global variable declaration
int taskid = 0;     //Next task's id
int ntask;

#pragma ompix taskdef in(m,s) inout(A_input[1024 * 1024], B_input[1024 * 1024])
void node_instat(int m, int s, int *A_input, int *B_input)
{
	int i;
	M = m;
	S = s;

	A_data = (int *) malloc(N * N * sizeof(int));
	B_data = (int *) malloc(N * N * sizeof(int));

	
	A = (int **) malloc (N * sizeof(int *));
	B = (int **) malloc (N * sizeof(int *));

	for (i = 0; i < N; i++)
	{
		A[i] = (int *) &(A_data[N * i]);
		B[i] = (int *) &(B_data[N * i]);
	}

	for (i = 0; i < N * N; i++)
	{
		A_data[i] = A_input[i];
		B_data[i] = B_input[i];
	}

	res_data = (int *) malloc (S * S * sizeof(int));
	res = (int **) malloc (S * sizeof (int *));

	return;
}

int **A, *A_data, **B, *B_data, **C, *C_data;
int *res_data, **res; // Reusable across tasks



#pragma ompix taskdef
void node_free_mem()
{
	free(A[0]);
	free(A);
	free(B[0]);
	free(B);

	free(res[0]);
	free(res);
}


#pragma ompix taskdef in (wid) out(results[S * S])
void taskexecute(int wid, int *results){

	int i, j, k, x, y, outer_limit, inner_limit, r_i = 0;
	int sum = 0;


	x           = wid / M;
	y           = wid % M;
	outer_limit = (x + 1) * S;	//Computed here once, so 'for' loops don't
	inner_limit = (y + 1) * S;  //have to perform these multiplications
                                //on each iteration.


	for (i = x * S; i < outer_limit; i++){
		for (j = y * S; j < inner_limit; j++){

			for (k = 0, sum = 0; k < N; k++)
				sum += A[i][k] * B[k][j];


			results[r_i++] = sum;
		}
	}


	return;

}
{
	// Write retrieved results to final array
	int i, j, x, y, outer_limit, inner_limit, r_i = 0;
//	printf("Task %d callback\n", wid);
	x           = wid / M;
	y           = wid % M;
	outer_limit = (x + 1) * S;
	inner_limit = (y + 1) * S;

	for (i = x * S; i < outer_limit; i++)

		for (j = y * S; j < inner_limit; j++)

			C[i][j] = results[r_i++];

}




int main(int argc, char *argv[])
{
	int i, task_num, **master_res;
	double start_time, end_time, test_times[NUM_OF_TESTS], avg_time = 0;


	A_data = (int *) malloc (N * N * sizeof(int));
	B_data = (int *) malloc (N * N * sizeof(int));
	C_data = (int *) malloc (N * N * sizeof(int));
	
	A = (int **) malloc (N * sizeof(int *));
	B = (int **) malloc (N * sizeof(int *));
	C = (int **) malloc (N * sizeof(int *));

	for (i = 0; i < N; i++)
	{
		A[i] = (int *) &(A_data[N * i]);
		B[i] = (int *) &(B_data[N * i]);
		C[i] = (int *) &(C_data[N * i]);
	}

	if (argc == 1){
		printf("Submatrix size must be entered in command "
		"line arguments (32, 64, or 256)\n");
		return (1);
	}

	S = atoi(argv[1]);

	//M must be              4, 16, 32
	//Therefore, s must be 256, 64, 32, respectively.

	if (S != 256 && S != 64 && S != 32){
		printf("Acceptable submatrix sizes: 32, 64, 256\n");
		return (1);
	}

	M = N / S;

	ntask  = NTASK;
	// Need this many result arrays
	master_res = (int **) malloc (ntask * sizeof(int *));
	for (i = 0; i < ntask; i++)
		master_res[i] = (int *) malloc (S * S * sizeof(int));

	/* Read matrices from files: "A_file", "B_file"
	 */
	if (readmat("Amat1024", (int *) A_data, N) < 0)
		exit( 1 + printf("file problem\n") );
	if (readmat("Bmat1024", (int *) B_data, N) < 0)
		exit( 1 + printf("file problem\n") );



	// Inform all worker nodes of the decided upon task dimensions
	for (i = 1; i < torc_num_nodes(); i++)
	{
		#pragma ompix task atnode(i)
		node_instat(M, S, A_data, B_data);
	}
	#pragma ompix tasksync

	start_time = torc_gettime();


	while(1){

		task_num = taskid++;

		if (task_num >= ntask)
			break;			//All tasks created


		#pragma ompix task 
		taskexecute(task_num, master_res[task_num]);

	}
	#pragma ompix tasksync


	end_time      = torc_gettime();
	test_times[i] = end_time - start_time;
	avg_time     += test_times[i];


	// No need to wait for them, we can wait right before we finish
	for (i = 1; i < torc_num_nodes(); i++)
	{
		#pragma ompix task atnode(i)
		node_free_mem();
	}

	for (i = 0; i < NUM_OF_TESTS; ++i)
		printf("Test #%d: %lf seconds\n", i + 1, test_times[i]);

	printf("[S = %d] Average time: %f\n", S, avg_time / (double) NUM_OF_TESTS);



	/* Save result in "Cmat1024Results"
	 */
	writemat("Cmat1024Results", (int *) C_data, N);
	
	for (i = 0; i < ntask; i++)
		free(master_res[i]);
	free(master_res);

	// First line frees the data, second one the int * array we used.
	free(A[0]);
	free(A);
	free(B[0]);
	free(B);
	free(C[0]);
	free(C);


	#pragma ompix tasksync
	return (0);
}


/* Utilities to read & write matrices from/to files
 * VVD
 */

#define _mat(i,j) (mat[(i)*n + (j)])


int readmat(char *fname, int *mat, int n){
	FILE *fp;
	int  i, j;

	if ((fp = fopen(fname, "r")) == NULL)
		return (-1);
	for (i = 0; i < n; i++){
		for (j = 0; j < n; j++){
			if (fscanf(fp, "%d", &_mat(i,j)) == EOF){
				fclose(fp);
				return (-1);
			};
		}
	}
	fclose(fp);
	return (0);
}


int writemat(char *fname, int *mat, int n){
	FILE *fp;
	int  i, j;

	if ((fp = fopen(fname, "w")) == NULL)
		return (-1);
	for (i = 0; i < n; i++, fprintf(fp, "\n")){
		for (j = 0; j < n; j++)
			fprintf(fp, " %d", _mat(i, j));
	}
	fclose(fp);
	return (0);
}





