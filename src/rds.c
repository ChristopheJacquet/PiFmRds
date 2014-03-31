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

void get_rds_samples(float *buffer, int count) {
    static int bit_buffer[BITS_PER_GROUP];
    static int bit_pos = BITS_PER_GROUP;
    
    //static int prev_output = 0;
    static int prev_bit = 0;
    static int cur_bit = 0;
    //static int cur_output = 0;
    static int next_bit = 0;
    static int sample_pos = SAMPLES_PER_BIT;
    
    for(int i=0; i<count; i++) {
        if(sample_pos >= SAMPLES_PER_BIT) {
            if(bit_pos >= BITS_PER_GROUP) {
                get_rds_group(bit_buffer);
                bit_pos = 0;
            }
            prev_bit = cur_bit;
            cur_bit = next_bit;
            next_bit = bit_buffer[bit_pos];
            //prev_output = cur_output;
            //cur_output = prev_output ^ cur_bit;
            
            bit_pos++;
            //printf("%d", cur_bit); fflush(stdout);
            sample_pos = 0;
        }
        
        float sample = symbol_samples[prev_bit][cur_bit][next_bit][sample_pos];
        *buffer++ = sample;
        printf("%c", (((int)(sample*100))));
        sample_pos++;
    }
}

int main(int argc, char **argv) {
    rds_params.pi = 0x1234;
    
    strncpy(rds_params.text, "Hello", 64);
    float buffer[128000];
    
    get_rds_samples(buffer, 100000);
}