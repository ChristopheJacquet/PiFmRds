/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    
    See https://github.com/ChristopheJacquet/PiFmRds

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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "waveforms.h"

#define MAX_TEXT_LENGTH 64
#define GROUP_LENGTH 4

struct {
    char text[MAX_TEXT_LENGTH];
    uint16_t pi;
} rds_params;

/* The RDS error-detection code generator polynomial is
   x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + x^0
*/
#define POLY 0x1B9
#define POLY_DEG 10
#define MSB_BIT 0x8000
#define BLOCK_SIZE 16

#define BITS_PER_GROUP (GROUP_LENGTH * (BLOCK_SIZE+POLY_DEG))
#define SAMPLES_PER_BIT 192
#define FILTER_SIZE (sizeof(waveform_biphase)/sizeof(float))


uint16_t offset_words[] = {0x0FC, 0x198, 0x168, 0x1B4};
// We don't handle offset word C' here for the sake of simplicity

/* Classical CRC computation */
uint16_t crc(uint16_t block) {
    uint16_t crc = 0;
    
    for(int j=0; j<BLOCK_SIZE; j++) {
        int bit = (block & MSB_BIT) != 0;
        block <<= 1;

        int msb = (crc >> (POLY_DEG-1)) & 1;
        crc <<= 1;
        if((msb ^ bit) != 0) {
            crc = crc ^ POLY;
        }
    }
    
    return crc;
}


/* Creates an RDS group. This generates sequences of the form 0B, 0B, 0B, 0B, 2A, etc.
   The pattern is of length 5, the variable 'state' keeps track of where we are in the
   pattern. 'ps_state' and 'rt_state' keep track of where we are in the PS (0B) sequence
   or RT (2A) sequence, respectively.
*/
void get_rds_group(int *buffer) {
    static int state = 0;
    static int ps_state = 0;
    static int rt_state = 0;
    uint16_t blocks[GROUP_LENGTH] = {rds_params.pi, 0, 0, 0};
    
    // Generate block content
    if(state < 4) {
        blocks[1] = 0x0000 | ps_state;
        blocks[2] = 0xCDCD;     // no AF
        blocks[3] = rds_params.text[ps_state*2]<<8 | rds_params.text[ps_state*2+1];
        ps_state++;
        if(ps_state >= 4) ps_state = 0;
    } else { // state == 5
        blocks[1] = 0x2000 | rt_state;
        blocks[2] = rds_params.text[rt_state*4+0]<<8 | rds_params.text[rt_state*4+1];
        blocks[3] = rds_params.text[rt_state*4+2]<<8 | rds_params.text[rt_state*4+3];
        rt_state++;
        if(rt_state >= 16) rt_state = 0;
    }
    
    state++;
    if(state >= 5) state = 0;
    
    // Calculate the checkword for each block and emit the bits
    for(int i=0; i<GROUP_LENGTH; i++) {
        uint16_t block = blocks[i];
        uint16_t check = crc(block) ^ offset_words[i];
        for(int j=0; j<BLOCK_SIZE; j++) {
            *buffer++ = ((block & (1<<(BLOCK_SIZE-1))) != 0);
            block <<= 1;
        }
        for(int j=0; j<POLY_DEG; j++) {
            *buffer++= ((check & (1<<(POLY_DEG-1))) != 0);
            check <<= 1;
        }
    }
}

/* Get a number of RDS samples. This generates the envelope of the waveform using
   pre-generated elementary waveform samples, and then it amplitude-modulates the 
   envelope with a 57 kHz carrier, which is very efficient as 57 kHz is 4 times the
   sample frequency we are working at (228 kHz).
 */
void get_rds_samples(float *buffer, int count) {
    static int bit_buffer[BITS_PER_GROUP];
    static int bit_pos = BITS_PER_GROUP;
    static float sample_buffer[2*FILTER_SIZE] = {0};
    
    static int prev_output = 0;
    static int cur_output = 0;
    static int cur_bit = 0;
    static int sample_count = SAMPLES_PER_BIT;
    static int inverting = 0;
    static int phase = 0;

    static int in_sample_index = 0;
    static int out_sample_index = FILTER_SIZE;
        
    for(int i=0; i<count; i++) {
        if(sample_count >= SAMPLES_PER_BIT) {
            if(bit_pos >= BITS_PER_GROUP) {
                get_rds_group(bit_buffer);
                bit_pos = 0;
            }
            
            // do differential encoding
            cur_bit = bit_buffer[bit_pos];
            prev_output = cur_output;
            cur_output = prev_output ^ cur_bit;
            
            inverting = (cur_output == 1) ? 1 : -1;
            
            float *src = waveform_biphase;
            int idx = in_sample_index;
            for(int j=0; j<FILTER_SIZE; j++) {
                //printf("%d ", j); fflush(stdout);
                if(j<FILTER_SIZE-SAMPLES_PER_BIT) {
                    sample_buffer[idx++] += (*src++) * inverting;
                } else {
                    sample_buffer[idx++] = (*src++) * inverting;
                }
                if(idx >= 2*FILTER_SIZE) idx = 0;
            }
            in_sample_index += SAMPLES_PER_BIT;
            if(in_sample_index >= 2*FILTER_SIZE) in_sample_index -= 2*FILTER_SIZE;
            
            bit_pos++;
            sample_count = 0;
            //printf("%d", cur_bit); fflush(stdout);
        }
        
        float sample = sample_buffer[out_sample_index++];
        if(out_sample_index >= 2*FILTER_SIZE) out_sample_index = 0;
        
        // modulate at 57 kHz
        // use phase for this
        switch(phase) {
            case 0:
            case 2: sample = 0; break;
            case 1: break;
            case 3: sample = -sample; break;
        }
        phase++;
        if(phase >= 4) phase = 0;
        
        *buffer++ = sample;
        sample_count++;
    }
}

void set_rds_params(uint16_t pi_code, char *text) {
    rds_params.pi = pi_code;
    
    strncpy(rds_params.text, text, 64);
}
