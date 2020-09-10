#general options
PAR ?= yes
PARBATCH ?= no
L2 ?= no
L2T ?= no
CC = ompicc
ORT ?= nat
#CC = suncc
#CC = icc
#CC = gcc
#CC = omcc

ifeq ($(L2), yes)
CFLAGS += -DL2PAR
endif

ifeq ($(L2T), yes)
L2 = task
CFLAGS += -DL2PARTASK
endif

ifeq ($(PARBATCH), yes)
CFLAGS += -DPARBATCH 
# -g -ggdb
endif

ifeq ($(CC), gcc)
CFLAGS += -O3 
ifeq ($(PAR), yes)
CFLAGS +=-fopenmp
endif
LDFLAGS = -static
endif

ifeq ($(CC), icc)
CC = icc
CFLAGS += -O3 -tpp7 -Ob2 -axW -w # -parallel
ifeq ($(PAR), yes)
CFLAGS +=-openmp
endif
LDFLAGS = -static
endif

ifeq ($(CC), suncc)
CFLAGS += -fast
ifeq ($(PAR), yes)
CFLAGS +=-xopenmp=parallel
endif
#LDFLAGS = -dn
endif

ifeq ($(CC), omcc)
CFLAGS += -O3
endif

ifeq ($(CC), nompicc)
ifeq ($(ORT), nat)
ORT = posix
endif
CFLAGS += -O3 --ort=$(ORT)
LDFLAGS = -static
endif


ifeq ($(CC), ompipfcc)
ifeq ($(ORT), nat)
ORT = pthreads
endif
CFLAGS += -O3 --ort=$(ORT)
LDFLAGS = -static
endif

ifeq ($(CC), ompiwscc)
ifeq ($(ORT), nat)
ORT = pthreads
endif
CFLAGS += -O3 --ort=$(ORT)
LDFLAGS = -static
endif

ifeq ($(CC), ompicc)
ifeq ($(ORT), nat)
ORT = torc
endif
CFLAGS += -g -ggdb3 \
-DTORC_SUPPORT \
--nolineno -O3 \
--ort=$(ORT) \
--nolineno -k \
-DL2PARTASK_LO3
#-DL2PARTASK_LO2
#-DL2PARTASK_LO3
#-DL2PARTASK -k --nolineno -g -ggdb3
#LDFLAGS = -static
endif

ifeq ($(CC), mcc)
CFLAGS += -O3 -m32
endif

.SUFFIXES:
.SUFFIXES: .c $(SUFFIXES)
LIBRARIES = -lm


SRCS =	Convolver.c FastConv.c FeatureMap.c NN.c lolac.c mem.c ConvolverFine.c \
	FastConvUtils.c MemUtils.c Neuron.c main.c tanh_lup.c LpiImage.c
OBJS =	Convolver.o FastConv.o FeatureMap.o NN.o lolac.o mem.o ConvolverFine.o \
	FastConvUtils.o MemUtils.o Neuron.o main.o tanh_lup.o LpiImage.o
HDRS =	Convolver.h ConvolverFine.h FastConv.h FastConvUtils.h FeatureMap.h LpiImage.h \
	MemUtils.h NN.h Neuron.h mem.h memDecl.h memValues.h memValuesOld.h tanh_lup.h	Makefile

TARGET = lolac.$(CC).$(ORT).$(L2)
REMOVE = rm -f

all:	$(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS)  -o $(TARGET) $(OBJS) $(LDFLAGS) -lm 

clean:
	$(REMOVE) *.o $(TARGET)
	$(REMOVE) *.il *.da gmon.out core*

.c.o:
	$(CC) $(CFLAGS) -c $<


Convolver.o: Convolver.c $(HDRS)
FastConv.o: FastConv.c $(HDRS)
FeatureMap.o: FeatureMap.c $(HDRS)
NN.o: NN.c $(HDRS)
lolac.o: lolac.c $(HDRS)
mem.o: mem.c $(HDRS)
ConvolverFine.o: ConvolverFine.c $(HDRS)
FastConvUtils.o: FastConvUtils.c $(HDRS)
MemUtils.o: MemUtils.c $(HDRS)
Neuron.o: Neuron.c $(HDRS)
main.o: main.c $(HDRS)
tanh_lup.o: tanh_lup.c $(HDRS)
LpiImage.o: LpiImage.c $(HDRS)

