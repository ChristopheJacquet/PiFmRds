CC = gcc
CFLAGS = -Wall -std=gnu99 -c -g -O2

app: rds.o waveforms.o pi_fm_rds.o
	$(CC) -o pi_fm_rds rds.o waveforms.o pi_fm_rds.o -lm

rds_wav: rds.o waveforms.o rds_wav.o
	$(CC) -o rds_wav rds_wav.o rds.o waveforms.o -lsndfile

rds.o: rds.c waveforms.h
	$(CC) $(CFLAGS) rds.c

waveforms.o: waveforms.c waveforms.h
	$(CC) $(CFLAGS) waveforms.c

pi_fm_rds.o: pi_fm_rds.c
	$(CC) $(CFLAGS) pi_fm_rds.c

rds_wav.o: rds_wav.c
	$(CC) $(CFLAGS) rds_wav.c

waveforms.h: generate_waveforms.py
	python generate_waveforms.py

clean:
	rm *.o
