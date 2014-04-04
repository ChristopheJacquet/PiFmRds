#include <stdio.h>

#include "rds.h"

#define LENGTH 100000

/* Simple test program */
int main(int argc, char **argv) {
    set_rds_params(0x1234, "Hello!  World of the RaspberryPi!");
    
    float buffer[LENGTH];
    int count = 0;
    
    for(int j=0; j<20; j++) {
        get_rds_samples(buffer, LENGTH);

        fprintf(stderr, "Iteration %d count=%d\n", j, count);

        for(int i=0; i<LENGTH; i++) {
            printf("%c", (((int)(buffer[i]*50))));
            count++;
        }
    }
}