CC = gcc
CFLAGS = -Wall -std=gnu99 -c -g -O2

app: rds.o waveforms.o pi_fm_rds.o fm_mpx.o
	$(CC) -o pi_fm_rds rds.o waveforms.o pi_fm_rds.o fm_mpx.o -lm -lsndfile

rds_wav: rds.o waveforms.o rds_wav.o fm_mpx.o
	$(CC) -o rds_wav rds_wav.o rds.o waveforms.o fm_mpx.o -lm -lsndfile

rds.o: rds.c waveforms.h
	$(CC) $(CFLAGS) rds.c

waveforms.o: waveforms.c waveforms.h
	$(CC) $(CFLAGS) waveforms.c

pi_fm_rds.o: pi_fm_rds.c
	$(CC) $(CFLAGS) pi_fm_rds.c

rds_wav.o: rds_wav.c
	$(CC) $(CFLAGS) rds_wav.c

fm_mpx.o: fm_mpx.c fm_mpx.h
	$(CC) $(CFLAGS) fm_mpx.c

clean:
	rm *.o
