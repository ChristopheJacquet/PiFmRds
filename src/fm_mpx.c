/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    
    See https://github.com/ChristopheJacquet/PiFmRds
    
    rds_wav.c is a test program that writes a RDS baseband signal to a WAV
    file. It requires libsndfile.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <sndfile.h>
#include <stdlib.h>
#include <math.h>

#include "rds.h"


#define PI 3.141592654


#define FIR_HALF_SIZE 30 
#define FIR_SIZE (2*FIR_HALF_SIZE-1)

#define LENGTH 114000
// TODO: remove constant

// coefficients of the low-pass FIR filter
float low_pass_fir[FIR_HALF_SIZE];


float carrier_38[] = {0.0, 0.8660254037844386, 0.8660254037844388, 1.2246467991473532e-16, -0.8660254037844384, -0.8660254037844386};

float carrier_19[] = {0.0, 0.5, 0.8660254037844386, 1.0, 0.8660254037844388, 0.5, 1.2246467991473532e-16, -0.5, -0.8660254037844384, -1.0, -0.8660254037844386, -0.5};
    
int phase_38 = 0;
int phase_19 = 0;


float downsample_factor;


float rds_buffer[LENGTH] = {0};
float audio_buffer[LENGTH] = {0};
int audio_index = 0;
int audio_len = 0;
float audio_pos;
float out = 0;
float alpha = .03;

float fir_buffer[FIR_SIZE] = {0};
int fir_index = 0;

SNDFILE *inf;

    
    

int fm_mpx_open(char *filename) {
    // Open the input file
    SF_INFO sfinfo;
    if(! (inf = sf_open(filename, SFM_READ, &sfinfo))) {
        fprintf(stderr, "Error: could not open input file %s.\n", filename) ;
        return EXIT_FAILURE; // TODO better error code
    }
    
    int in_samplerate = sfinfo.samplerate;
    downsample_factor = 228000. / in_samplerate;
    
    printf("Input: %d Hz, upsampling factor: %.2f\n", in_samplerate, downsample_factor);
    
    
    // Create the low-pass FIR filter
    float cutoff_freq = 15000 * .8;
    if(in_samplerate/2 < cutoff_freq) cutoff_freq = in_samplerate/2 * .8;
    
    
    
    low_pass_fir[FIR_HALF_SIZE-1] = 2 * cutoff_freq / 228000 /2;
    // Here we divide this coefficient by two because it will be counted twice
    // when applying the filter

    // Only store half of the filter since it is symmetric
    for(int i=1; i<FIR_HALF_SIZE; i++) {
        low_pass_fir[FIR_HALF_SIZE-1-i] = 
            sin(2 * PI * cutoff_freq * i / 228000) / (PI * i)      // sinc
            * (.54 - .46 * cos(2*PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE)));
                                                          // Hamming window
    }
    printf("Created low-pass FIR filter for audio channels, with cutoff at %.1f Hz\n", cutoff_freq);
    
    /*
    for(int i=0; i<FIR_HALF_SIZE; i++) {
        printf("%.5f ", low_pass_fir[i]);
    }
    printf("\n");
    */
    
    audio_pos = downsample_factor;
    
    return 0; // TODO
}


int fm_mpx_get_samples(float *mpx_buffer) { // TODO accept length in argument
    get_rds_samples(rds_buffer, LENGTH);

    for(int i=0; i<LENGTH; i++) {
        if(audio_pos >= downsample_factor) {
            audio_pos -= downsample_factor;
            
            if(audio_len == 0) {
                for(int j=0; j<2; j++) { // one retry
                    audio_len = sf_read_float(inf, audio_buffer, LENGTH);
                    if (audio_len < 0) {
                        fprintf(stderr, "Error reading audio\n");
                        exit(EXIT_FAILURE);
                    }
                    if(audio_len == 0) {
                        sf_seek(inf, 0, SEEK_SET);
                    } else {
                        break;
                    }
                }
                audio_index = 0;
            } else {
                audio_index++;
                audio_len--;
            }
        }

        // Apply FIR low-pass filter
        fir_buffer[fir_index] = audio_buffer[audio_index];
        fir_index++;
        if(fir_index >= FIR_SIZE) fir_index = 0;
        
        
        /* As the FIR filter is symmetric, we do not multiply all 
           the coefficients independently, but two-by-two, thus reducing
           the total number of multiplications by a factor of two
        */
        out = 0;
        int ifbi = fir_index;  // ifbi = increasing FIR Buffer Index
        int dfbi = fir_index;  // dfbi = decreasing FIR Buffer Index
        for(int fi=0; fi<FIR_HALF_SIZE; fi++) {  // fi = Filter Index
            dfbi--;
            if(dfbi < 0) dfbi = FIR_SIZE-1;
            out += low_pass_fir[fi] * (fir_buffer[ifbi] + fir_buffer[dfbi]);
            ifbi++;
            if(ifbi >= FIR_SIZE) ifbi = 0;
        }
        // End of FIR filter
        

        mpx_buffer[i] = (rds_buffer[i] + 
            4*out /* + 
            2 * carrier_38[phase_38] * out + 
            .1*carrier_19[phase_19]*/) / 10;
        
        phase_19++;
        phase_38++;
        if(phase_19 >= 12) phase_19 = 0;
        if(phase_38 >= 6) phase_38 = 0;
        
        audio_pos++;   
        
    }
    
    return 0; // TODO
}


int fm_mpx_close() {
    if(sf_close(inf) ) {
        fprintf(stderr, "Error closing audio file");
    }
    return 0; // TODO
}