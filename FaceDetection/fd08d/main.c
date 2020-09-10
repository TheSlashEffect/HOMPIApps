#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#if defined(_OPENMP) 
#include <omp.h>
#endif

#include "LpiImage.h"

extern int callbackStillImage(struct lpiImage* image, int *X, int *Y, int *H, int *W, float *O);
extern int callbackFrameImage(struct lpiImage *image, int *X, int *Y, int *H, int *W, float *O);
extern void InitCFF();

extern double my_gettime(void);

#define MAX_IMAGES	256
#define MAX_FACES	64



int max_faces = MAX_FACES;
#include <torc.h>

#pragma ompix taskdef
void torc_instat(){
	printf("Node #%d instantiating CNN...\n", torc_node_id());
	InitCFF();
}



#pragma ompix taskdef inout(par_name[512]) in(seqno)
void torc_remote_imagetask(char *par_name, int seqno){
	
	int i;
	
	struct lpiImage *src;
	char name[512];
	
	int X[MAX_FACES], Y[MAX_FACES], H[MAX_FACES], W[MAX_FACES];
	float O[MAX_FACES];
	int nofFaces;

	printf("\n\nNode #%d\n", torc_node_id());
	
	name[0] = '\0';
	sscanf(par_name, "%s", name);
	
	//printf("\nNode #%d torc task with image = %s\n",
	//	   torc_node_id() ,par_name);
	
	src = lpiImage_loadPGM(par_name);
	
	//printf("About to convolve\n");
	nofFaces = callbackStillImage(src, X, Y, H, W, O);
	
	
	for (i = 0; i < nofFaces; i++){
		printf("FACE[%d]: %d\t%d\t%d\t%d\t%f\n",
			   i, X[i], Y[i], H[i], W[i], O[i]);
	}

	printf("#%d : Torc node %d found %d faces\n",
		   seqno, torc_node_id(), nofFaces);
	lpiImage_lpiReleaseImage(&src);
	
}


int main(int argc, char *argv[])
{
	FILE *fp;
	char buf[512], name[512];
	int count = 0;
	struct lpiImage *src[MAX_IMAGES] = {0};
	int num_threads = 1;
	int nested = 0;
	double t1, t2, t3;
	int i, j, num_nodes, my_node;
	char **name_cpies;

	j = 0;
	
	if (argc == 2)
		num_threads = atoi(argv[1]);

	if (argc == 3) {
		num_threads = atoi(argv[1]);
		nested = atoi(argv[2]);
	}

	InitCFF();
	fp=fopen("list.txt", "r");
	
	// PARALLELIZE HERE
	// Above task is completed.
	// Must clean house and parameterize etc...
	
	name_cpies = (char **) malloc(170 * sizeof (char *));

	// Instantiate convolutional network on each process
	num_nodes = torc_num_nodes();
    for (i = 0; i < num_nodes; i++){
        #pragma ompix task atnode(i)
        torc_instat();
    }
    
	#pragma ompix tasksync
	
	
	t1 = my_gettime();
	
	while( fgets(buf, sizeof(buf), fp)!=NULL )
	{
		
		name[0] = '\0';
		strcpy(name, buf);
		name[strlen(name) - 1 ] = '\0';
		
		count++;

		if (count == MAX_IMAGES) {
			printf("too many images...\n");
			exit(1);
		}

		
		name_cpies[j] = (char *) malloc (512 * sizeof(char));
		strcpy(name_cpies[j], name);
		#pragma ompix task
		torc_remote_imagetask(name_cpies[j], j);
		j++;
		
	}



	t2 = my_gettime();
#if defined(_OPENMP)
	printf("Running with %d threads and nested %s\n", num_threads, (nested)?"ENABLED":"DISABLED");
	omp_set_nested(nested);
	omp_set_num_threads(num_threads);
#endif

	#pragma ompix tasksync

	t3 = my_gettime();

	printf("TIME FOR LOADING IMAGES	= %f\n", t2-t1);
	printf("TIME FOR FACE DETECTION	= %f\n", t3-t2);
	printf("TOTAL EXECUTION TIME	= %f\n", t3-t1);
	
	for (i = 0 ; i < 170; i++)
		free(name_cpies[i]);
	
	free(name_cpies);

	exit(0);
}
