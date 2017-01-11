/*
 * PiFmRds - FM/RDS transmitter for the Raspberry Pi
 * Copyright (C) 2014, 2015 Christophe Jacquet, F8FTK
 * Copyright (C) 2012, 2015 Richard Hirst
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
 * To make it work on the Raspberry Pi 2, I used a fix by Richard Hirst
 * (again) to request memory using Broadcom's mailbox interface. This fix
 * was published for ServoBlaster here:
 * https://www.raspberrypi.org/forums/viewtopic.php?p=699651#p699651
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
 * This code uses DMA channel 5 and the PWM hardware, with no regard for
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
#include "fm_mpx.h"
#include "control_pipe.h"

#include "mailbox.h"
#define MBFILE            DEVICE_FILE_NAME    /* From mailbox.h */

#if (RASPI)==1
#define PERIPH_VIRT_BASE 0x20000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0x40000000
#define MEM_FLAG 0x0c
#elif (RASPI)==2
#define PERIPH_VIRT_BASE 0x3f000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

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

#define DMA_NUMBER         5
#define DMA_BASE_OFFSET        0x00007000
#define DMA_LEN            0xe24
#define PWM_BASE_OFFSET        0x0020C000
#define PWM_LEN            0x28
#define CLK_BASE_OFFSET            0x00101000
#define CLK_LEN            0x1300
#define GPIO_BASE_OFFSET    0x00200000
#define GPIO_LEN        0x100

#define DMA_VIRT_BASE        (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE        (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE        (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE        (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE        (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE        (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE        (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE        (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)


#define PWM_CTL            (0x00/4)
#define PWM_DMAC        (0x08/4)
#define PWM_RNG1        (0x10/4)
#define PWM_FIFO        (0x18/4)

#define PWMCLK_CNTL        40
#define PWMCLK_DIV        41

#define CM_GP0DIV (0x7e101074)
#define CM_PLLCFRAC  (0x7e102220)

#define CORECLK_CNTL      (0x08/4)
#define CORECLK_DIV       (0x0c/4)
#define GPCLK_CNTL        (0x70/4)
#define GPCLK_DIV         (0x74/4)
#define EMMCCLK_CNTL     (0x1C0/4)
#define EMMCCLK_DIV      (0x1C4/4)
#define PLLC_CTRL       (0x1120/4)
#define PLLC_FRAC       (0x1220/4)

#define PWMCTL_MODE1        (1<<1)
#define PWMCTL_PWEN1        (1<<0)
#define PWMCTL_CLRF        (1<<6)
#define PWMCTL_USEF1        (1<<5)

#define PWMDMAC_ENAB        (1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing.
#define PWMDMAC_THRSHLD        ((15<<8)|(15<<0))

#define GPFSEL0            (0x00/4)

#define PLLFREQ            500000000.    // PLLD is running at 500MHz

// The deviation specifies how wide the signal is. 
// Use 75kHz for WBFM (broadcast radio) 
// and about 2.5kHz for NBFM (walkie-talkie style radio)
#define DEVIATION        75000


typedef struct {
    uint32_t info, src, dst, length,
         stride, next, pad[2];
} dma_cb_t;

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)


static struct {
    int handle;            /* From mbox_open() */
    unsigned mem_ref;    /* From mem_alloc() */
    unsigned bus_addr;    /* From mem_lock() */
    uint8_t *virt_addr;    /* From mapmem() */
} mbox;
    


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
print_clock_tree(void)
{

   if( clk_reg==NULL ) return;
#define PLLA_CTRL (0x1100/4)
#define PLLA_FRAC (0x1200/4)
#define PLLA_DSI0 (0x1300/4)
#define PLLA_CORE (0x1400/4)
#define PLLA_PER  (0x1500/4)
#define PLLA_CCP2 (0x1600/4)

#define PLLB_CTRL  (0x11e0/4)
#define PLLB_FRAC  (0x12e0/4)
#define PLLB_ARM   (0x13e0/4)
#define PLLB_SP0   (0x14e0/4)
#define PLLB_SP1   (0x15e0/4)
#define PLLB_SP2   (0x16e0/4)

#define PLLC_CTRL  (0x1120/4)
#define PLLC_FRAC  (0x1220/4)
#define PLLC_CORE2 (0x1320/4)
#define PLLC_CORE1 (0x1420/4)
#define PLLC_PER   (0x1520/4)
#define PLLC_CORE0 (0x1620/4)

#define PLLD_CTRL (0x1140/4)
#define PLLD_FRAC (0x1240/4)
#define PLLD_DSI0 (0x1340/4)
#define PLLD_CORE (0x1440/4)
#define PLLD_PER  (0x1540/4)
#define PLLD_DSI1 (0x1640/4)

#define PLLH_CTRL (0x1160/4)
#define PLLH_FRAC (0x1260/4)
#define PLLH_AUX  (0x1360/4)
#define PLLH_RCAL (0x1460/4)
#define PLLH_PIX  (0x1560/4)
#define PLLH_STS  (0x1660/4)

#define XOSC_CTRL (0x1190/4)
   printf("PLLC_DIG0=%08x\n",clk_reg[(0x1020/4)]);
   printf("PLLC_DIG1=%08x\n",clk_reg[(0x1024/4)]);
   printf("PLLC_DIG2=%08x\n",clk_reg[(0x1028/4)]);
   printf("PLLC_DIG3=%08x\n",clk_reg[(0x102c/4)]);
   printf("PLLC_ANA0=%08x\n",clk_reg[(0x1030/4)]);
   printf("PLLC_ANA1=%08x\n",clk_reg[(0x1034/4)]);
   printf("PLLC_ANA2=%08x\n",clk_reg[(0x1038/4)]);
   printf("PLLC_ANA3=%08x\n",clk_reg[(0x103c/4)]);
   printf("PLLC_DIG0R=%08x\n",clk_reg[(0x1820/4)]);
   printf("PLLC_DIG1R=%08x\n",clk_reg[(0x1824/4)]);
   printf("PLLC_DIG2R=%08x\n",clk_reg[(0x1828/4)]);
   printf("PLLC_DIG3R=%08x\n",clk_reg[(0x182c/4)]);

   printf("GNRIC CTL=%08x DIV=%8x  ",clk_reg[ 0],clk_reg[ 1]);
   printf("VPU   CTL=%08x DIV=%8x\n",clk_reg[ 2],clk_reg[ 3]);
   printf("SYS   CTL=%08x DIV=%8x  ",clk_reg[ 4],clk_reg[ 5]);
   printf("PERIA CTL=%08x DIV=%8x\n",clk_reg[ 6],clk_reg[ 7]);
   printf("PERII CTL=%08x DIV=%8x  ",clk_reg[ 8],clk_reg[ 9]);
   printf("H264  CTL=%08x DIV=%8x\n",clk_reg[10],clk_reg[11]);
   printf("ISP   CTL=%08x DIV=%8x  ",clk_reg[12],clk_reg[13]);
   printf("V3D   CTL=%08x DIV=%8x\n",clk_reg[14],clk_reg[15]);

   printf("CAM0  CTL=%08x DIV=%8x  ",clk_reg[16],clk_reg[17]);
   printf("CAM1  CTL=%08x DIV=%8x\n",clk_reg[18],clk_reg[19]);
   printf("CCP2  CTL=%08x DIV=%8x  ",clk_reg[20],clk_reg[21]);
   printf("DSI0E CTL=%08x DIV=%8x\n",clk_reg[22],clk_reg[23]);
   printf("DSI0P CTL=%08x DIV=%8x  ",clk_reg[24],clk_reg[25]);
   printf("DPI   CTL=%08x DIV=%8x\n",clk_reg[26],clk_reg[27]);
   printf("GP0   CTL=%08x DIV=%8x  ",clk_reg[28],clk_reg[29]);
   printf("GP1   CTL=%08x DIV=%8x\n",clk_reg[30],clk_reg[31]);

   printf("GP2   CTL=%08x DIV=%8x  ",clk_reg[32],clk_reg[33]);
   printf("HSM   CTL=%08x DIV=%8x\n",clk_reg[34],clk_reg[35]);
   printf("OTP   CTL=%08x DIV=%8x  ",clk_reg[36],clk_reg[37]);
   printf("PCM   CTL=%08x DIV=%8x\n",clk_reg[38],clk_reg[39]);
   printf("PWM   CTL=%08x DIV=%8x  ",clk_reg[40],clk_reg[41]);
   printf("SLIM  CTL=%08x DIV=%8x\n",clk_reg[42],clk_reg[43]);
   printf("SMI   CTL=%08x DIV=%8x  ",clk_reg[44],clk_reg[45]);
   printf("SMPS  CTL=%08x DIV=%8x\n",clk_reg[46],clk_reg[47]);

   printf("TCNT  CTL=%08x DIV=%8x  ",clk_reg[48],clk_reg[49]);
   printf("TEC   CTL=%08x DIV=%8x\n",clk_reg[50],clk_reg[51]);
   printf("TD0   CTL=%08x DIV=%8x  ",clk_reg[52],clk_reg[53]);
   printf("TD1   CTL=%08x DIV=%8x\n",clk_reg[54],clk_reg[55]);

   printf("TSENS CTL=%08x DIV=%8x  ",clk_reg[56],clk_reg[57]);
   printf("TIMER CTL=%08x DIV=%8x\n",clk_reg[58],clk_reg[59]);
   printf("UART  CTL=%08x DIV=%8x  ",clk_reg[60],clk_reg[61]);
   printf("VEC   CTL=%08x DIV=%8x\n",clk_reg[62],clk_reg[63]);

   printf("PULSE CTL=%08x DIV=%8x  ",clk_reg[100],clk_reg[101]);
   printf("PLLT  CTL=%08x DIV=????????\n",clk_reg[76]);

   printf("DSI1E CTL=%08x DIV=%8x  ",clk_reg[86],clk_reg[87]);
   printf("DSI1P CTL=%08x DIV=%8x\n",clk_reg[88],clk_reg[89]);
   printf("AVE0  CTL=%08x DIV=%8x\n",clk_reg[90],clk_reg[91]);

   printf("SDC   CTL=%08x DIV=%8x  ",clk_reg[106],clk_reg[107]);
   printf("ARM   CTL=%08x DIV=%8x\n",clk_reg[108],clk_reg[109]);
   printf("AVE0  CTL=%08x DIV=%8x  ",clk_reg[110],clk_reg[111]);
   printf("EMMC  CTL=%08x DIV=%8x\n",clk_reg[112],clk_reg[113]);

   // Sometimes calculated frequencies are off by a factor of 2
   // ANA1 bit 14 may indicate that a /2 prescaler is active
   printf("PLLA PDIV=%d NDIV=%d FRAC=%d  ",(clk_reg[PLLA_CTRL]>>16) ,clk_reg[PLLA_CTRL]&0x3ff, clk_reg[PLLA_FRAC] );
   printf(" %f MHz\n",19.2* ((float)(clk_reg[PLLA_CTRL]&0x3ff) + ((float)clk_reg[PLLA_FRAC])/((float)(1<<20))) );
   printf("DSI0=%d CORE=%d PER=%d CCP2=%d\n\n",clk_reg[PLLA_DSI0],clk_reg[PLLA_CORE],clk_reg[PLLA_PER],clk_reg[PLLA_CCP2]);


   printf("PLLB PDIV=%d NDIV=%d FRAC=%d  ",(clk_reg[PLLB_CTRL]>>16) ,clk_reg[PLLB_CTRL]&0x3ff, clk_reg[PLLB_FRAC] );
   printf(" %f MHz\n",19.2* ((float)(clk_reg[PLLB_CTRL]&0x3ff) + ((float)clk_reg[PLLB_FRAC])/((float)(1<<20))) );
   printf("ARM=%d SP0=%d SP1=%d SP2=%d\n\n",clk_reg[PLLB_ARM],clk_reg[PLLB_SP0],clk_reg[PLLB_SP1],clk_reg[PLLB_SP2]);

   printf("PLLC PDIV=%d NDIV=%d FRAC=%d  ",(clk_reg[PLLC_CTRL]>>16) ,clk_reg[PLLC_CTRL]&0x3ff, clk_reg[PLLC_FRAC] );
   printf(" %f MHz\n",19.2* ((float)(clk_reg[PLLC_CTRL]&0x3ff) + ((float)clk_reg[PLLC_FRAC])/((float)(1<<20))) );
   printf("CORE2=%d CORE1=%d PER=%d CORE0=%d\n\n",clk_reg[PLLC_CORE2],clk_reg[PLLC_CORE1],clk_reg[PLLC_PER],clk_reg[PLLC_CORE0]);

   printf("PLLD PDIV=%d NDIV=%d FRAC=%d  ",(clk_reg[PLLD_CTRL]>>16) ,clk_reg[PLLD_CTRL]&0x3ff, clk_reg[PLLD_FRAC] );
   printf(" %f MHz\n",19.2* ((float)(clk_reg[PLLD_CTRL]&0x3ff) + ((float)clk_reg[PLLD_FRAC])/((float)(1<<20))) );
   printf("DSI0=%d CORE=%d PER=%d DSI1=%d\n\n",clk_reg[PLLD_DSI0],clk_reg[PLLD_CORE],clk_reg[PLLD_PER],clk_reg[PLLD_DSI1]);

   printf("PLLH PDIV=%d NDIV=%d FRAC=%d  ",(clk_reg[PLLH_CTRL]>>16) ,clk_reg[PLLH_CTRL]&0x3ff, clk_reg[PLLH_FRAC] );
   printf(" %f MHz\n",19.2* ((float)(clk_reg[PLLH_CTRL]&0x3ff) + ((float)clk_reg[PLLH_FRAC])/((float)(1<<20))) );
   printf("AUX=%d RCAL=%d PIX=%d STS=%d\n\n",clk_reg[PLLH_AUX],clk_reg[PLLH_RCAL],clk_reg[PLLH_PIX],clk_reg[PLLH_STS]);


}

static void
udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}

static void
terminate(int num)
{
    // Stop outputting and generating the clock.
    if (clk_reg && gpio_reg && mbox.virt_addr) {
        // Set GPIO4 to be an output (instead of ALT FUNC 0, which is the clock).
        gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (1 << 12);

        // Disable the clock generator.
        clk_reg[GPCLK_CNTL] = 0x5A;
    }

    if (dma_reg && mbox.virt_addr) {
        dma_reg[DMA_CS] = BCM2708_DMA_RESET;
        udelay(10);
    }
    
    fm_mpx_close();
    close_control_pipe();

    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, NUM_PAGES * 4096);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
    }

    print_clock_tree();

    printf("Terminating: cleanly deactivated the DMA engine and killed the carrier.\n");
    
    exit(num);
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
    uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

    return mbox.bus_addr + offset;
}

static uint32_t
mem_phys_to_virt(uint32_t phys)
{
    return phys - (uint32_t)mbox.bus_addr + (uint32_t)mbox.virt_addr;
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m.\n");
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED)
        fatal("Failed to map peripheral at 0x%08x: %m.\n", base);
    close(fd);

    return vaddr;
}



#define SUBSIZE 1
#define DATA_SIZE 5000


int tx(uint32_t carrier_freq, uint32_t divider, char *audio_file, uint16_t pi, char *ps, char *rt, float ppm, char *control_pipe) {
    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (int i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }
        
    dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
    dma_reg = dma_reg+((0x100/sizeof(int))*(DMA_NUMBER));
    pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);

    // Use the mailbox interface to the VC to ask for physical memory.
    mbox.handle = mbox_open();
    if (mbox.handle < 0)
        fatal("Failed to open mailbox. Check kernel support for vcio / BCM2708 mailbox.\n");
    printf("Allocating physical memory: size = %d     ", NUM_PAGES * 4096);
    if(! (mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * 4096, 4096, MEM_FLAG))) {
        fatal("Could not allocate memory.\n");
    }
    // TODO: How do we know that succeeded?
    printf("mem_ref = %u     ", mbox.mem_ref);
    if(! (mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
        fatal("Could not lock memory.\n");
    }
    printf("bus_addr = %x     ", mbox.bus_addr);
    if(! (mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * 4096))) {
        fatal("Could not map memory.\n");
    }
    printf("virt_addr = %p\n", mbox.virt_addr);
    

    uint32_t freq_ctl;
    if( divider ) // PLL modulation
    {
       // switch the core over to PLLA
       clk_reg[CORECLK_DIV]  = (0x5a<<24) | (4<<12) ; // core div 4
       udelay(100);
       clk_reg[CORECLK_CNTL] = (0x5a<<24) | (1<<4) | (4); // run, src=PLLA

       // switch the EMMC over to PLLD
       // FIXME should also adjust divider to keep same frequency
       // or possibly not if the gpu decides to change clocks due to low voltage
       // TODO  put clocks back to their original state when we are done?
       int clktmp;
       clktmp = clk_reg[EMMCCLK_CNTL];
       clk_reg[EMMCCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24) ; // clear run
       udelay(100);
       clk_reg[EMMCCLK_CNTL] = (0xF00&clktmp) | (0x5a<<24) | (6); // src=PLLD
       udelay(100);
       clk_reg[EMMCCLK_CNTL] = (0xF00&clktmp) | (0x5a<<24) | (1<<4) | (6); // run , src=PLLD

       // Adjust PLLC frequency
       freq_ctl = (unsigned int)(((carrier_freq*divider)/19.2e6)*((double)(1<<20))) ; // todo PPM
       clk_reg[PLLC_CTRL] = (0x5a<<24) | (0x21<<12) | (freq_ctl>>20 ); // integer part
       freq_ctl&=0xFFFFF;
       clk_reg[PLLC_FRAC] = (0x5a<<24) | (freq_ctl&0xFFFFC) ; // fractional part
       udelay(1000);

       // Program GPCLK integer division
       clktmp = clk_reg[GPCLK_CNTL];
       clk_reg[GPCLK_CNTL] = (0xF0F&clktmp) | (0x5a<<24) ; // clear run
       udelay(100);
       clk_reg[GPCLK_DIV]  = (0x5a<<24) | (divider<<12); 
       udelay(100);
       clk_reg[GPCLK_CNTL] = (0x5a<<24) | (5); // src=PLLC
       udelay(100);
       clk_reg[GPCLK_CNTL] = (0x5a<<24) | (1<<4) | (5); // run , src=PLLC




    }
    else  // MASH modulation
    {
       // Program GPCLK to use MASH setting 1, so fractional dividers work
       clk_reg[GPCLK_CNTL] = 0x5A << 24 | 6;  // src 6 = PLLD
       udelay(100);
       clk_reg[GPCLK_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;


       // Calculate the frequency control word
       // The fractional part is stored in the lower 12 bits
       freq_ctl = ((float)(PLLFREQ / carrier_freq)) * ( 1 << 12 ); //PLLD=500MHz
    }

    // GPIO4 needs to be ALT FUNC 0 to output the clock
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12);


    ctl = (struct control_data_s *) mbox.virt_addr;
    dma_cb_t *cbp = ctl->cb;
    uint32_t phys_sample_dst = divider ? CM_PLLCFRAC : CM_GP0DIV;
    uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;

    for (int i = 0; i < NUM_SAMPLES; i++) {
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
        cbp->src = mem_virt_to_phys(mbox.virt_addr);
        cbp->dst = phys_pwm_fifo_addr;
        cbp->length = 4;
        cbp->stride = 0;
        cbp->next = mem_virt_to_phys(cbp + 1);
        cbp++;
    }
    cbp--;
    cbp->next = mem_virt_to_phys(mbox.virt_addr);

    // Here we define the rate at which we want to update the GPCLK control 
    // register.
    //
    // Set the range to 2 bits. PLLD is at 500 MHz, therefore to get 228 kHz
    // we need a divisor of 500000 / 2 / 228 = 1096.491228
    //
    // This is 1096 + 2012*2^-12 theoretically
    //
    // However the fractional part may have to be adjusted to take the actual
    // frequency of your Pi's oscillator into account. For example on my Pi,
    // the fractional part should be 1916 instead of 2012 to get exactly 
    // 228 kHz. However RDS decoding is still okay even at 2012.
    //
    // So we use the 'ppm' parameter to compensate for the oscillator error

    float srdivider = (500000./(2*228*(1.+ppm/1.e6)));
    uint32_t idivider = (uint32_t) srdivider;
    uint32_t fdivider = (uint32_t) ((srdivider - idivider)*pow(2, 12));
    
    printf("ppm corr is %.4f, divider is %.4f (%d + %d*2^-12) [nominal 1096.4912].\n", 
                ppm, srdivider, idivider, fdivider);

    pwm_reg[PWM_CTL] = 0;
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source=PLLD and disable
    udelay(100);
    // theorically : 1096 + 2012*2^-12
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (idivider<<12) | fdivider;
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

    
    uint32_t last_cb = (uint32_t)ctl->cb;

    // Data structures for baseband data
    float data[DATA_SIZE];
    int data_len = 0;
    int data_index = 0;

    // Initialize the baseband generator
    if(fm_mpx_open(audio_file, DATA_SIZE) < 0) return 1;
    
    // Initialize the RDS modulator
    char myps[9] = {0};
    set_rds_pi(pi);
    set_rds_rt(rt);
    uint16_t count = 0;
    uint16_t count2 = 0;
    int varying_ps = 0;
    
    if(ps) {
        set_rds_ps(ps);
        printf("PI: %04X, PS: \"%s\".\n", pi, ps);
    } else {
        printf("PI: %04X, PS: <Varying>.\n", pi);
        varying_ps = 1;
    }
    printf("RT: \"%s\"\n", rt);
    
    // Initialize the control pipe reader
    if(control_pipe) {
        if(open_control_pipe(control_pipe) == 0) {
            printf("Reading control commands on %s.\n", control_pipe);
        } else {
            printf("Failed to open control pipe: %s.\n", control_pipe);
            control_pipe = NULL;
        }
    }
    
    
    printf("Starting to transmit on %3.1f MHz.\n", carrier_freq/1e6);


    float deviation_scale_factor;
    if( divider ) // PLL modulation
    {   // note samples are [-10:10]
        deviation_scale_factor=  0.1 * (divider*DEVIATION/  (19.2e6/((double)(1<<20))) ) ; // todo PPM
    }
    else // MASH modulation
    {   
        double normdivider=((double)PLLFREQ / (double)carrier_freq) * ( 1 << 12 ); //PLLD=500MHz
        double highdivider=((double)PLLFREQ / (((double)carrier_freq)+(DEVIATION))) * ( 1 << 12 ); 
        deviation_scale_factor= 0.1 * (highdivider-normdivider);
        // Scaling factor for mash is negative because 
        // a higher division ratio equals a lower frequency. 
    }
    //printf("deviation_scale_factor = %f \n", deviation_scale_factor);


    for (;;) {
        // Default (varying) PS
        if(varying_ps) {
            if(count == 512) {
                snprintf(myps, 9, "%08d", count2);
                set_rds_ps(myps);
                count2++;
            }
            if(count == 1024) {
                set_rds_ps("RPi-Live");
                count = 0;
            }
            count++;
        }
        
        if(control_pipe && poll_control_pipe() == CONTROL_PIPE_PS_SET) {
            varying_ps = 0;
        }
        
        usleep(5000);

        uint32_t cur_cb = mem_phys_to_virt(dma_reg[DMA_CONBLK_AD]);
        int last_sample = (last_cb - (uint32_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int this_sample = (cur_cb - (uint32_t)mbox.virt_addr) / (sizeof(dma_cb_t) * 2);
        int free_slots = this_sample - last_sample;
        if (free_slots < 0)
            free_slots += NUM_SAMPLES;

        while (free_slots >= SUBSIZE) {
            // get more baseband samples if necessary
            if(data_len == 0) {
                if( fm_mpx_get_samples(data) < 0 ) {
                    terminate(0);
                }
                data_len = DATA_SIZE;
                data_index = 0;
            }
            
            float dval = data[data_index]*deviation_scale_factor;
            int intval;
            if( divider ) // PLL modulation
            {
              intval = ( (int)((floor)(dval)) & ~0x3 ) ; 
              // clock settings from boot code seem to always leave 2 lsb clear 
            }
            else // MASH modulation
            { 
               intval = (int)((floor)(dval)) ;
            }
            data_index++;
            data_len--;



            ctl->sample[last_sample++] = ( 0x5A << 24 | freq_ctl) + intval;
            if (last_sample == NUM_SAMPLES)
                last_sample = 0;

            free_slots -= SUBSIZE;
        }
        last_cb = (uint32_t)mbox.virt_addr + last_sample * sizeof(dma_cb_t) * 2;
    }

    return 0;
}


int main(int argc, char **argv) {
    char *audio_file = NULL;
    char *control_pipe = NULL;
    uint32_t carrier_freq = 107900000;
    char *ps = NULL;
    char *rt = "PiFmRds: live FM-RDS transmission from the RaspberryPi";
    uint16_t pi = 0x1234;
    uint32_t mash = -1;
    float ppm = 0;
    
    
    // Parse command-line arguments
    for(int i=1; i<argc; i++) {
        char *arg = argv[i];
        char *param = NULL;
        
        if(arg[0] == '-' && i+1 < argc) param = argv[i+1];
        
        if((strcmp("-wav", arg)==0 || strcmp("-audio", arg)==0) && param != NULL) {
            i++;
            audio_file = param;
        } else if(strcmp("-freq", arg)==0 && param != NULL) {
            i++;
            carrier_freq = 1e6 * atof(param);
            if(carrier_freq < 76e6 || carrier_freq > 108e6)
                fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9, between 76 and 108.\n");
        } else if(strcmp("-pi", arg)==0 && param != NULL) {
            i++;
            pi = (uint16_t) strtol(param, NULL, 16);
        } else if(strcmp("-ps", arg)==0 && param != NULL) {
            i++;
            ps = param;
        } else if(strcmp("-rt", arg)==0 && param != NULL) {
            i++;
            rt = param;
        } else if(strcmp("-ppm", arg)==0 && param != NULL) {
            i++;
            ppm = atof(param);
        } else if(strcmp("-ctl", arg)==0 && param != NULL) {
            i++;
            control_pipe = param;
        } else if(strcmp("-mash", arg)==0 ) {
            i++;
            mash=1;
        } else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds [-freq freq] [-audio file] [-ppm ppm_error] [-pi pi_code]\n"
            "                  [-ps ps_text] [-rt rt_text] [-ctl control_pipe] [-mash]\n", arg);
        }
    }


    // Choose an integer divider for GPCLK0
    // 
    // There may be improvements possible to this algorithm. 
    double xtal_freq_recip=1.0/19.2e6; // todo PPM correction 
    int best_divider=0;

    // Optional loop to print tuning solutions for all FM frequencies
    //for( carrier_freq=76000000 ; carrier_freq<108000000 ; carrier_freq+=100000 )
    {
      int solution_count=0;
      printf("carrier:%3.2f ",carrier_freq/1e6);
      int divider,min_int_multiplier,max_int_multiplier, fom, int_multiplier, best_fom=0;
      double frac_multiplier;
      best_divider=0;
      for( divider=2;divider<20;divider+=1)
      {
        if( carrier_freq*divider <  760e6 ) continue; // widest accepted frequency range
        if( carrier_freq*divider > 1300e6 ) break;

        max_int_multiplier=((int)((double)(carrier_freq+10+DEVIATION)*divider*xtal_freq_recip));
        min_int_multiplier=((int)((double)(carrier_freq-10-DEVIATION)*divider*xtal_freq_recip));
        if( min_int_multiplier!=max_int_multiplier ) continue; // don't cross integer boundary

        solution_count++;  // if we make it here the solution is acceptable, 
        fom=0;             // but we want a good solution

        if( carrier_freq*divider >  900e6 ) fom++; // prefer freqs closer to 1000
        if( carrier_freq*divider < 1100e6 ) fom++;
        if( carrier_freq*divider >  800e6 ) fom++; // accepted frequency range
        if( carrier_freq*divider < 1200e6 ) fom++;
        

        frac_multiplier=((double)(carrier_freq)*divider*xtal_freq_recip);
        int_multiplier = (int) frac_multiplier;
        frac_multiplier = frac_multiplier - int_multiplier;
        if( (frac_multiplier>0.2) && (frac_multiplier<0.8) ) fom++; // prefer mulipliers away from integer boundaries 


        //if( divider%2 == 1 ) fom+=2; // prefer odd dividers
        // Even and odd dividers could have different harmonic content,
        // but the latest measurements have shown no significant difference.


        //printf(" multiplier:%f divider:%d VCO: %4.1fMHz\n",carrier_freq*divider*xtal_freq_recip,divider,(double)carrier_freq*divider/1e6);
        if( fom > best_fom )
        {
            best_fom=fom;
            best_divider=divider;
        }
      }
      printf(" multiplier:%f divider:%d VCO: %4.1fMHz\n",carrier_freq*best_divider*xtal_freq_recip,best_divider,(double)carrier_freq*best_divider/1e6);
    
      if( solution_count==0) 
            printf("No tuning solution found. (76.8 and 96.0 are not supported.)\n");

    }// Optional loop to print tuning solutions

    if( best_divider==0) return -1 ; 

    if( mash!=-1 )
    {
       best_divider=0;  // if divider=0,  use MASH divider instead of PLL modulation
    }
    int errcode = tx(carrier_freq, best_divider, audio_file, pi, ps, rt, ppm, control_pipe);
    
    terminate(errcode);
}
