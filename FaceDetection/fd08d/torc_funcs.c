#include <stdio.h>
#include "LpiImage.h"
#include "Convolver.h"

#define MAX_FACES 1000

struct per_image_data_t {
	float*** outputValues;
	struct lpiImage **imgt;
	struct lpiImage **imgt_f;
	struct CConvolver *cconv;


	char gl_FileTitle[256];


	//contain position, scale and score for each target detected after convolve
	int *Xs;
	int *Ys;
	float *outputs;
	int *heights;

	//contain position, scale and score for each cluster center
	int *Xf;
	int *Yf;
	float *outf;
	int *Hf;


	int gl_S1;
	int gl_S2;
	double gl_ST;
	int gl_height;
	int gl_width;

	int allX[MAX_FACES];
	int allY[MAX_FACES];
	int allH[MAX_FACES];
	float allOut[MAX_FACES];



	int vg_deleted[MAX_FACES];
	int discarded[MAX_FACES];
};




#pragma ompix taskdef in(iter, itotal, Sm) inout(img_source_ptr[1],  img_input_ptr[1], facesFound[1], data_ptr[1])
void torc_phase1(int iter, int itotal,
	void *img_source_ptr, void *img_input_ptr,
	void *data_ptr, int Sm[64], int *facesFound
)
{
	int S, SH = 36;
	float SR = 36.0 / 32.0; // ((float) SW / ((float)SH);
	struct per_image_data_t *data;
	struct lpiImage *img_source, *img_input;
	
	img_source = (struct lpiImage *) img_source_ptr;
	img_input  = (struct lpiImage *) img_input_ptr;
	
	data = (struct per_image_data_t *) data_ptr;
	
	
	for(iter = 0; iter < itotal; iter++)
	{
		int i, j;
		int width=img_source->width;
		int height=img_source->height;
		int nbS;

		{
		int h1;
		int w1;

		struct lpiImage *img_tmp;
		unsigned char *p1;
		float *p2;

		int current_width, current_height, total_size;

		float *pp;

		nbS = iter;
		S = Sm[iter];

		//printf("[%d] iter %d\n", omp_get_thread_num(), iter); fflush(0);

#if defined(_OPENMP)
	//	omp_set_lock(&lock);
#endif


		h1 = (int)floor((double)height*((double)SH/(double)S));
		w1 = (int)floor((double)width*((double)SH/(double)S));


		if(h1<36 || w1<32) {
			printf("this should not happen!\n");
			exit(1);
			continue;
		}

		data->imgt[nbS]=lpiImage_lpiCreateImage(w1, h1, sizeof(unsigned char));

			//CONVOLVE
//		{
//			double xt1, xt2;
//			xt1 = my_gettime();
			lpiImage_lpiResize(img_input, data->imgt[nbS]);
//			xt2 = my_gettime();
//			printf("Resize: %lf\n", xt2-xt1);
//		}


		img_tmp=lpiImage_lpiCreateImage(w1, h1, sizeof(float));


		p1=(unsigned char *)data->imgt[nbS]->imageData;
		p2=(float *)img_tmp->imageData;
		for(i=0;i<h1;i++)
			for(j=0;j<w1;j++)
				p2[i*w1+j]=(float)( ((int)p1[i*data->imgt[nbS]->width+j]) - 128)/((float)128.0);

//		{
//			double xt1, xt2;
//			xt1 = my_gettime();
			pp=CConvolver_ConvolveRoughlyStillImage(&data->cconv[nbS], img_tmp, w1, h1, &current_width, &current_height, &total_size, S);
//			xt2 = my_gettime();
//			printf("ConvolveRoughlyStillImage: %lf\n", xt2-xt1);
//		}


			//CHECK THE RESULTS
		for(i=0;i<total_size;i++)
		{
			float output=pp[i];
			int y;
			int x;

			int ycenter;
			int xcenter;

			if (output<=(float)0.0)
				continue;

			y=i/current_width;
			x=i%current_width;

			ycenter = (int)floor((double)(4*y*S)/(double)SH) + (int)floor((double)S/2.0);
			xcenter = (int)floor((double)(4*x*S)/(double)SH) + (int)floor((double)S*SR/2.0);




//			#pragma omp critical 
			{
			data->Xs[*facesFound]=xcenter;
			data->Ys[*facesFound]=ycenter;
			data->heights[*facesFound]=S;
			data->outputs[*facesFound]=output;
			*facesFound++;
			}


		}

			//FREE IMAGE
		lpiImage_lpiReleaseImage(&data->imgt[nbS]);
		CConvolver_DeallocateOutput(&data->cconv[nbS]);

		lpiImage_lpiReleaseImage(&img_tmp);
		//nbS++;


		}
	}

}
