#include <stdio.h>

#include "rds.h"

#define LENGTH 99840

/* Simple test program */
int main(int argc, char **argv) {
    set_rds_params(0x1234, "Hello");
    
    float buffer[LENGTH];
    
    get_rds_samples(buffer, LENGTH);
    
    for(int j=0; j<50; j++) {
        for(int i=0; i<LENGTH; i++) {
            printf("%c", (((int)(buffer[i]*70))));
        }
    }
}