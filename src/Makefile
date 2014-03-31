CC=gcc
CFLAGS=-Wall -std=c99 -c -g

all: rds.o waveforms.o
	$(CC) -o test rds.o waveforms.o

rds.o: rds.c
	$(CC) $(CFLAGS) rds.c

waveforms.o: waveforms.c waveforms.h
	$(CC) $(CFLAGS) waveforms.c
	
clean:
	rm *.o