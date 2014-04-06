/*
 * PiFmRds - FM/RDS transmitter for the Raspberry Pi
 * Copyright (C) 2014 Christophe Jacquet, F8FTK
 * Copyright (C) 2012 Richard Hirst
 * Copyright (C) 2012 Oliver Mattos and Oskar Weigl
 *
 * See https://github.com/ChristopheJacquet/PiFmRds
 *
 * PI-FM-RDS: RaspberryPi FM transmitter, with RDS. 
 *
 * This file contains the VHF FM modulator. All credit goes to the original
 * authors, Oliver Mattos and Oskar Weigl for the original idea, and to
 * Richard Hirst for using the Pi's DMA engine, which reduced CPU usage
 * dramatically.
 *
 * I (Christophe Jacquet) have adapted their idea to transmitting samples
 * at 228 kHz, allowing to build the 57 kHz subcarrier for RDS BPSK data.
 *
 * Never use this to transmit VHF-FM data through an antenna, as it is
 * illegal in most countries. This code is for testing purposes only.
 * Always connect a shielded transmission line from the RaspberryPi directly
 * to a radio receiver, so as *not* to emit radio waves.
 *
 * ---------------------------------------------------------------------------
 * These are the comments from Richard Hirst's version:
 *
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 * 
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"


#define NUM_SAMPLES        50000
#define NUM_CBS            (NUM_SAMPLES * 2)

#define BCM2708_DMA_NO_WIDE_BURSTS    (1<<26)
#define BCM2708_DMA_WAIT_RESP        (1<<3)
#define BCM2708_DMA_D_DREQ        (1<<6)
#define BCM2708_DMA_PER_MAP(x)        ((x)<<16)
#define BCM2708_DMA_END            (1<<1)
#define BCM2708_DMA_RESET        (1<<31)
#define BCM2708_DMA_INT            (1<<2)

#define DMA_CS            (0x00/4)
#define DMA_CONBLK_AD        (0x04/4)
#define DMA_DEBUG        (0x20/4)

#define DMA_BASE        0x20007000
#define DMA_LEN            0x24
#define PWM_BASE        0x2020C000
#define PWM_LEN            0x28
#define CLK_BASE            0x20101000
#define CLK_LEN            0xA8
#define GPIO_BASE        0x20200000
#define GPIO_LEN        0xB4


#define PWM_CTL            (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL        40
#define PWMCLK_DIV        41

#define GPCLK_CNTL        (0x70/4)
#define GPCLK_DIV        (0x74/4)

#define PWMCTL_MODE1        (1<<1)
#define PWMCTL_PWEN1        (1<<0)
#define PWMCTL_CLRF        (1<<6)
#define PWMCTL_USEF1        (1<<5)

#define PWMDMAC_ENAB        (1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing..
#define PWMDMAC_THRSHLD        ((15<<8)|(15<<0))

#define GPFSEL0            (0x00/4)

#define PLLFREQ            500000000.    // PLLD is running at 500MHz        ////

// The deviation specifies how wide the signal is. Use 25.0 for WBFM
// (broadcast radio) and about 3.5 for NBFM (walkie-talkie style radio)
#define DEVIATION        25.0


typedef struct {
    uint32_t info, src, dst, length,
         stride, next, pad[2];
} dma_cb_t;

typedef struct {
    uint8_t *virtaddr;
    uint32_t physaddr;
} page_map_t;

page_map_t *page_map;

static uint8_t *virtbase;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

struct control_data_s {
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
};

#define PAGE_SIZE    4096
#define PAGE_SHIFT    12
#define NUM_PAGES    ((sizeof(struct control_data_s) + PAGE_SIZE - 1) >> PAGE_SHIFT)

static struct control_data_s *ctl;

static void
udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}

static void
terminate(int dummy)
{
    if (dma_reg) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
    }
    exit(1);
}

static void
fatal(char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

static uint32_t
mem_virt_to_phys(void *virt)
{
    uint32_t offset = (uint8_t *)virt - virtbase;

    return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static uint32_t
mem_phys_to_virt(uint32_t phys)
{
    uint32_t pg_offset = phys & (PAGE_SIZE - 1);
    uint32_t pg_addr = phys - pg_offset;
    int i;

    for (i = 0; i < NUM_PAGES; i++) {
        if (page_map[i].physaddr == pg_addr) {
            return (uint32_t)virtbase + i * PAGE_SIZE + pg_offset;
        }
    }
    fatal("Failed to reverse map phys addr %08x\n", phys);

    return 0;
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m\n");
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED)
        fatal("Failed to map peripheral at 0x%08x: %m\n", base);
    close(fd);

    return vaddr;
}


#define SUBSIZE 1
#define RDS_DATA_SIZE 5000
#define DATA_SIZE 10000


int
main(int argc, char **argv)
{
    int i, fd, pid;
    char pagemap_fn[64];

    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }
        
    dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

    virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
            MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
            -1, 0);
    if (virtbase == MAP_FAILED)
        fatal("Failed to mmap physical pages: %m\n");
    if ((unsigned long)virtbase & (PAGE_SIZE-1))
        fatal("Virtual address is not page aligned\n");
    printf("Virtual memory mapped at %p\n", virtbase);
    page_map = malloc(NUM_PAGES * sizeof(*page_map));
    if (page_map == 0)
        fatal("Failed to malloc page_map: %m\n");
    pid = getpid();
    sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
    fd = open(pagemap_fn, O_RDONLY);
    if (fd < 0)
        fatal("Failed to open %s: %m\n", pagemap_fn);
    if (lseek(fd, (unsigned long)virtbase >> 9, SEEK_SET) != (unsigned long)virtbase >> 9)
        fatal("Failed to seek on %s: %m\n", pagemap_fn);
//    printf("Page map:\n");
    for (i = 0; i < NUM_PAGES; i++) {
        uint64_t pfn;
        page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
        // Following line forces page to be allocated
        page_map[i].virtaddr[0] = 0;
        if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
            fatal("Failed to read %s: %m\n", pagemap_fn);
        if (((pfn >> 55)&0xfbf) != 0x10c)  // pagemap bits: https://www.kernel.org/doc/Documentation/vm/pagemap.txt
            fatal("Page %d not present (pfn 0x%016llx)\n", i, pfn);
        page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
//        printf("  %2d: %8p ==> 0x%08x [0x%016llx]\n", i, page_map[i].virtaddr, page_map[i].physaddr, pfn);
    }

    // GPIO4 needs to be ALT FUNC 0 to otuput the clock
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);

    // Program GPCLK to use MASH setting 1, so fractional dividers work
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;

    ctl = (struct control_data_s *)virtbase;
    dma_cb_t *cbp = ctl->cb;
    uint32_t phys_sample_dst = 0x7e101074;
    uint32_t phys_pwm_fifo_addr = 0x7e20c000 + 0x18;


    uint32_t carrier_freq = 107900000;

    // Calculate the frequency control word
    // The fractional part is stored in the lower 12 bits
    uint32_t freq_ctl = ((float)(PLLFREQ / carrier_freq)) * ( 1 << 12 );


    for (i = 0; i < NUM_SAMPLES; i++) {
        ctl->sample[i] = 0x5a << 24 | freq_ctl;    // Silence
        // Write a frequency sample
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP;
        cbp->src = mem_virt_to_phys(ctl->sample + i);
        cbp->dst = phys_sample_dst;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
        // Delay
        cbp->info = BCM2708_DMA_NO_WIDE_BURSTS | BCM2708_DMA_WAIT_RESP | BCM2708_DMA_D_DREQ | BCM2708_DMA_PER_MAP(5);
        cbp->src = mem_virt_to_phys(virtbase);
        cbp->dst = phys_pwm_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    cbp--;
    cbp->next = mem_virt_to_phys(virtbase);

    // Here we define the rate at which we want to update the GPCLK control 
    // register.
    //
    // Set the range to 2 bits. PLLD is at 500 MHz, therefore to get 228 kHz
    // we need a divisor of 500000 / 2 / 228 = 1096.491228
    //
    // This is 1096 + 2012*2^-12
    //
    // However the fractional part may have to be adjusted to take the actual
    // frequency of your Pi's oscillator into account. For example on my Pi,
    // the fractional part should be 1916 instead of 2012 to get exactly 
    // 228 kHz. However RDS decoding is still okay even at 2012.

    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source=PLLD and disable
    udelay(100);
    // theorically : 1096 + 2012*2^-12
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (1096<<12) | 2012; // 1916 on my RaspberryPi
    udelay(100);
    clk_reg[PWMCLK_CNTL] = 0x5A000216;              // Source=PLLD and enable + MASH filter 1
    udelay(100);
    pwm_reg[PWM_RNG1] = 2;
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
    udelay(10);
    

    // Initialise the DMA
    dma_reg[DMA_CS] = BCM2708_DMA_RESET;
    udelay(10);
    dma_reg[DMA_CS] = BCM2708_DMA_INT | BCM2708_DMA_END;
    dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
    dma_reg[DMA_DEBUG] = 7; // clear debug error flags
    dma_reg[DMA_CS] = 0x10880001;    // go, mid priority, wait for outstanding writes

    
    // Try to read audio samples from a .wav file
    SNDFILE *sf = NULL;
    SF_INFO info;
    
    float data[DATA_SIZE];
    int data_len = 0;
        
    if (argc > 1) {
        sf = sf_open(argv[1],SFM_READ,&info);
        if(sf == NULL)
            fatal("Failed to read .wav file\n");
        if(info.samplerate != 228000)
            fatal("Sample rate must be 228 kHz\n");
        
        data_len = 0;        
    }



    uint32_t last_cb = (uint32_t)ctl->cb;
    int data_index = 0;

    float rds_data[RDS_DATA_SIZE];
    int rds_index = sizeof(rds_data);
    
    char ps[9] = {0};
    set_rds_pi(0x2345);
    set_rds_rt("RPi-Live - Live RDS transmission from the Raspberry Pi!");
    uint16_t count = 0;
    uint16_t count2 = 0;
    
    printf("Starting to transmit on %3.1f MHz.\n", carrier_freq/1e6);

    for (;;) {
        if(count == 512) {
            snprintf(ps, 9, "%08d", count2);
            set_rds_ps(ps);
            count2++;
        }
        if(count == 1024) {
            set_rds_ps("RPi-Live");
            count = 0;
        }
        count++;
        
        
        usleep(5000);

        uint32_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
        int last_sample = (last_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
        int this_sample = (cur_cb - (uint32_t)virtbase) / (sizeof(dma_cb_t) * 2);
        int free_slots = this_sample - last_sample;

        if (free_slots < 0)
            free_slots += NUM_SAMPLES;

        while (free_slots >= SUBSIZE) {
            // generate RDS samples if necessary
            if(rds_index >= RDS_DATA_SIZE) {
                get_rds_samples(rds_data, RDS_DATA_SIZE);
                rds_index = 0;
            }
            
            // read samples in the wav file if necessary
            if(sf && data_len == 0) {
                for(int j=0; j<2; j++) { // one retry
                    data_len = sf_read_float(sf, data, DATA_SIZE);
                    if (data_len < 0)
                        fatal("Error reading data: %m\n");
                    if(data_len == 0) {
                        sf_seek(sf, 0, SEEK_SET);
                    } else {
                        break;
                    }
                }
                data_index = 0;
            }
            
            
        
            //float dval = (float)(data[data_index])/65536.0 * DEVIATION;
            float dval = rds_data[rds_index] * (DEVIATION / 10.);
            rds_index++;

            // add modulation from .wav?
            if(sf && data_len > 0) {
                dval += data[data_index] * DEVIATION/2;
                data_index++;
                data_len--;
            }

            int intval = (int)((floor)(dval));
            //int frac = (int)((dval - (float)intval) * SUBSIZE);


            ctl->sample[last_sample++] = (0x5A << 24 | freq_ctl) + intval; //(frac > j ? intval + 1 : intval);
            if (last_sample == NUM_SAMPLES)
                last_sample = 0;

            free_slots -= SUBSIZE;
        }
        last_cb = (uint32_t)virtbase + last_sample * sizeof(dma_cb_t) * 2;
    }

    terminate(0);

    return 0;
}

