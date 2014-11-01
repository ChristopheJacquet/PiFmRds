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
    
    fm_mpx.c: generates an FM multiplex signal containing RDS plus possibly
    monaural or stereo audio.
*/

#include <sndfile.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>

#include "rds.h"


#define PI 3.141592654


#define FIR_HALF_SIZE 30 
#define FIR_SIZE (2*FIR_HALF_SIZE-1)


size_t length;

// coefficients of the low-pass FIR filter
float low_pass_fir[FIR_HALF_SIZE];


float carrier_38[] = {0.0, 0.8660254037844386, 0.8660254037844388, 1.2246467991473532e-16, -0.8660254037844384, -0.8660254037844386};

float carrier_19[] = {0.0, 0.5, 0.8660254037844386, 1.0, 0.8660254037844388, 0.5, 1.2246467991473532e-16, -0.5, -0.8660254037844384, -1.0, -0.8660254037844386, -0.5};
    
int phase_38 = 0;
int phase_19 = 0;


float downsample_factor;


float *audio_buffer;
int audio_index = 0;
int audio_len = 0;
float audio_pos;

float fir_buffer_mono[FIR_SIZE] = {0};
float fir_buffer_stereo[FIR_SIZE] = {0};
int fir_index = 0;
int channels;

SNDFILE *inf;



float *alloc_empty_buffer(size_t length) {
    float *p = malloc(length * sizeof(float));
    if(p == NULL) return NULL;
    
    bzero(p, length * sizeof(float));
    
    return p;
}


int fm_mpx_open(char *filename, size_t len) {
    length = len;

    if(filename != NULL) {
        // Open the input file
        SF_INFO sfinfo;
 
        // stdin or file on the filesystem?
        if(filename[0] == '-') {
            if(! (inf = sf_open_fd(fileno(stdin), SFM_READ, &sfinfo, 0))) {
                fprintf(stderr, "Error: could not open stdin for audio input.\n") ;
                return -1;
            } else {
                printf("Using stdin for audio input.\n");
            }
        } else {
            if(! (inf = sf_open(filename, SFM_READ, &sfinfo))) {
                fprintf(stderr, "Error: could not open input file %s.\n", filename) ;
                return -1;
            } else {
                printf("Using audio file: %s\n", filename);
            }
        }
            
        int in_samplerate = sfinfo.samplerate;
        downsample_factor = 228000. / in_samplerate;
    
        printf("Input: %d Hz, upsampling factor: %.2f\n", in_samplerate, downsample_factor);

        channels = sfinfo.channels;
        if(channels > 1) {
            printf("%d channels, generating stereo multiplex.\n", channels);
        } else {
            printf("1 channel, monophonic operation.\n");
        }
    
    
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
        audio_buffer = alloc_empty_buffer(length * channels);
        if(audio_buffer == NULL) return -1;

    } // end if(filename != NULL)
    else {
        inf = NULL;
        // inf == NULL indicates that there is no audio
    }
    
    return 0;
}


// samples provided by this function are in 0..10: they need to be divided by
// 10 after.
int fm_mpx_get_samples(float *mpx_buffer) {
    get_rds_samples(mpx_buffer, length);

    if(inf  == NULL) return 0; // if there is no audio, stop here
    
    for(int i=0; i<length; i++) {
        if(audio_pos >= downsample_factor) {
            audio_pos -= downsample_factor;
            
            if(audio_len == 0) {
                for(int j=0; j<2; j++) { // one retry
                    audio_len = sf_read_float(inf, audio_buffer, length);
                    if (audio_len < 0) {
                        fprintf(stderr, "Error reading audio\n");
                        return -1;
                    }
                    if(audio_len == 0) {
                        if( sf_seek(inf, 0, SEEK_SET) < 0 ) {
                            fprintf(stderr, "Could not rewind in audio file, terminating\n");
                            return -1;
                        }
                    } else {
                        break;
                    }
                }
                audio_index = 0;
            } else {
                audio_index += channels;
                audio_len -= channels;
            }
        }

        
        // First store the current sample(s) into the FIR filter's ring buffer
        if(channels == 0) {
            fir_buffer_mono[fir_index] = audio_buffer[audio_index];
        } else {
            // In stereo operation, generate sum and difference signals
            fir_buffer_mono[fir_index] = 
                audio_buffer[audio_index] + audio_buffer[audio_index+1];
            fir_buffer_stereo[fir_index] = 
                audio_buffer[audio_index] - audio_buffer[audio_index+1];
        }
        fir_index++;
        if(fir_index >= FIR_SIZE) fir_index = 0;
        
        // Now apply the FIR low-pass filter
        
        /* As the FIR filter is symmetric, we do not multiply all 
           the coefficients independently, but two-by-two, thus reducing
           the total number of multiplications by a factor of two
        */
        float out_mono = 0;
        float out_stereo = 0;
        int ifbi = fir_index;  // ifbi = increasing FIR Buffer Index
        int dfbi = fir_index;  // dfbi = decreasing FIR Buffer Index
        for(int fi=0; fi<FIR_HALF_SIZE; fi++) {  // fi = Filter Index
            dfbi--;
            if(dfbi < 0) dfbi = FIR_SIZE-1;
            out_mono += 
                low_pass_fir[fi] * 
                    (fir_buffer_mono[ifbi] + fir_buffer_mono[dfbi]);
            if(channels > 1) {
                out_stereo += 
                    low_pass_fir[fi] * 
                        (fir_buffer_stereo[ifbi] + fir_buffer_stereo[dfbi]);
            }
            ifbi++;
            if(ifbi >= FIR_SIZE) ifbi = 0;
        }
        // End of FIR filter
        

        mpx_buffer[i] = 
            mpx_buffer[i] +    // RDS data samples are currently in mpx_buffer
            4.05*out_mono;     // Unmodulated monophonic (or stereo-sum) signal
            
        if(channels>1) {
            mpx_buffer[i] +=
                4.05 * carrier_38[phase_38] * out_stereo + // Stereo difference signal
                .9*carrier_19[phase_19];                  // Stereo pilot tone

            phase_19++;
            phase_38++;
            if(phase_19 >= 12) phase_19 = 0;
            if(phase_38 >= 6) phase_38 = 0;
        }
            
        audio_pos++;   
        
    }
    
    return 0;
}


int fm_mpx_close() {
    if(sf_close(inf) ) {
        fprintf(stderr, "Error closing audio file");
    }
    
    if(audio_buffer != NULL) free(audio_buffer);
    
    return 0;
}