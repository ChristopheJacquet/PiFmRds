CC=gcc
CFLAGS=-Wall -std=c99 -c -g

all: rds.o waveforms.o
	$(CC) -o test rds.o waveforms.o

rds.o: rds.c waveforms.h
	$(CC) $(CFLAGS) rds.c

waveforms.o: waveforms.c waveforms.h
	$(CC) $(CFLAGS) waveforms.c

waveforms.h: generate_waveforms.py
	python generate_waveforms.py

clean:
	rm *.o