/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <limits.h>

#include <pico/stdlib.h>
#include <hardware/flash.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>

#include <math.h>
#include <gsl/gsl_fit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include "sht3x.h"
#include <ssd1306.h>

#include "nec_transmit.h"

// https://stackoverflow.com/a/9194117
// round_up_to2(x,y) rounds x up to the nearest y, if y is a power of 2
#define round_up_to2(x,y) (((x) + (y) - 1) & -(y))

// round_down_to2(x,y) rounds x down to the nearest y, if y is a power of 2
#define round_down_to2(x,y) ((x) & -(y))

// round_up_to(x,y) rounds x up to the nearest y, y is not necessarily a power of 2
#define round_up_to(x,y) ((((x) + (y) - 1) / y) * y)

// round_down_to(x,y) rounds x down to the nearest y, y is not necessarily a power of 2
#define round_down_to(x,y) (((x) / (y)) * (y))

#define FLASH_BINARY_END (round_up_to2(__flash_binary_end, FLASH_SECTOR_SIZE))

// 16 MB RP2040
#define FLASH_WRITE_RIGHT 0xffffff
#define FLASH_READ_RIGHT (FLASH_WRITE_RIGHT + XIP_BASE)

#define FLASH_WRITE_LEFT (FLASH_BINARY_END - XIP_BASE)
#define FLASH_READ_LEFT FLASH_BINARY_END

typedef struct entry {
        bool unused; // first bit must be 0 for a used entry
        bool valid; // allow marking invalid, if I cannot write (the rest?) or erase
        short RH[2];
        short T[2];
} entry;
// sizeof(entry) ...
// store data at different times
// compression:
// and also learn https://raw.githack.com/facebook/zstd/release/doc/zstd_manual.html
// first does zstd compile?
// https://spin.atomicobject.com/2013/03/14/heatshrink-embedded-data-compression/
// https://github.com/dblalock/sprintz looks the best so far

typedef struct { uint8_t body[FLASH_SECTOR_SIZE]; } sector_t;

typedef struct { uint8_t body[FLASH_PAGE_SIZE]; } page_t;

// ring buffer
// https://archived.moe/diy/thread/2440483/#2445511
// keep in mind writing addresses start at 0, while reading addresses
// (by dereferencing these pointers) start at XIP_BASE
//
// testing options:
// ceedling
// https://github.com/silentbicycle/greatest, https://github.com/silentbicycle/theft are a bit more minimal
// alternative runtime verification with copilot, which does not support pointers by design....
// so in this case I need to convert pointers into long for copilot
//
// invariants:
// s_L <= s <= s_R;
// s <= (long)p <= (long)(s+1). not really true when starting a new sector the very last page of the previous sector is available
// is_erased( s == s_L ? s_R : s - 1);
typedef struct {
        sector_t *s; // active sector in flash
        sector_t *s_L; // left sector in flash
        sector_t *s_R; // right sector in flash
        page_t *p, *p_L, *p_R; // active, left, right page in flash
        entry *e, *e_L, *e_R; // active, left and right bounds in page_buffer
        char page_gap;
        // a copy of the currently active page. Filled from indexes high to
        // low. The very last entry isn't used unless FLASH_PAGE_SIZE %
        // sizeof(entry) == 0, ie. page_gap == 0
        entry page_buffer[1+FLASH_PAGE_SIZE/sizeof(entry)];
} ringbuffer; 

// cycle_decr(rb, v) decrements (pointer) rb->v,
// except if it goes below rb->v_L, in which case
// it is replaced by rb->v_R
#define cycle_decr(rb, v) \
  do { rb->v--; \
       if (rb->v < rb->v ## _L) rb->v = rb->v ## _R; \
      } while (0)

#define is_right(rb, v) \
  ( (rb)->v == (rb)->v ## _R )

bool is_erased(sector_t *s) {
        for (int i; i < FLASH_SECTOR_SIZE; i++)
                if (s->body[i] != 0xff) return false;
        return true;
}

void init_rb(ringbuffer* rb) {
        extern long __flash_binary_end;

        rb->s_R = (sector_t*) round_down_to2(FLASH_READ_RIGHT, FLASH_SECTOR_SIZE);
        rb->s_R--; // possibly 
        rb->s_L = (sector_t*) FLASH_READ_LEFT; 

        rb->p_R = (page_t*) rb->s_R;
        rb->p_R += FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE - 1;
        rb->p_L = (page_t*) FLASH_READ_LEFT;

        rb->p = rb->p_R;

        // rb->p becomes the rightmost page where the leftmost entry.unused == true
        // or it stays p_R
        for (page_t* p = rb->p; p >= rb->p_L ; p--)
                if (p->body[0] & 0x1) {
                        rb->p = p;
                        break;
                }

        rb->s = (sector_t*) (round_down_to2((long)rb->p, FLASH_SECTOR_SIZE));

        // erase the next sector ahead
        sector_t *s_next = rb->s == rb->s_L ? rb->s_R : rb->s-1;
        if (!is_erased(s_next))
                flash_range_erase((long)s_next - XIP_BASE, FLASH_SECTOR_SIZE);


        char k = FLASH_PAGE_SIZE / sizeof(entry);
        rb->e_R = &rb->page_buffer[k];
        rb->e_L = &rb->page_buffer[0];
        rb->page_gap = FLASH_PAGE_SIZE % sizeof(entry);

        // fill all of page_buffer from flash
        entry* rbe = rb->e_L;
        for (entry* e = (entry*) rb->p; e <= rb->e_R ; )
                *rbe++ = *e++;

        // what if the current sector was not erased correctly?
        // I could copy the current sector to memory
        // then erase it
        // then write it back
        // or I could just accept that a sectors worth of data is invalid?
        // it would be better to know when data is invalid
        // so I need another bit
}

bool entry_eq(entry* x, entry* y) {
        return x->T[0] == y->T[0] &&
               x->T[1] == y->T[1] &&
               x->RH[0] == y->RH[0] &&
               x->RH[1] == y->RH[1];
}

void rb_advance(ringbuffer *rb) {
        // advance entry, page, sector pointers
        cycle_decr(rb, e);
        if (is_right(rb, e)) { // new page
                cycle_decr(rb, p);
                if (is_right(rb, p) || (long)rb->p == (long)rb->s) { // new sector
                        cycle_decr(rb, s); // s-- or s = s_R
                        flash_range_erase( (long)rb->s - XIP_BASE, FLASH_SECTOR_SIZE);
                }
        };
}

void insert_rb(ringbuffer* rb, entry* en) {
        // add the entry to flash
        *rb->e = *en;
        flash_range_program(((long)rb->p) - XIP_BASE, (uint8_t*)rb->page_buffer, FLASH_PAGE_SIZE);

        // now check written data?
        // I don't have a pointer into the flash indicating the current entry
        // I only have page
        struct entry* e_flash = (entry*)rb->p;
        e_flash += rb->e - rb->e_L;

        if (!entry_eq(e_flash, en)) printf("error");
        // recover by marking en.valid=false
        // I could call insert_rb
        //

}

#define SHT3X_ADDR 0x45

// test isnan?

// Stull 1947 coefficients from NIST webbook https://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185
double antoinePw(double T) {
        double A = 4.6543, B = 1435.264, C = -64.848 + 273.15;
        return pow(10 , (A - (B / (T + C))));
}

// g water / m3
double absHumidity(double T, double RH) {
        return 18 * RH * 1e5 * antoinePw(T) / 8.314 / (T + 273.15);
}
// TODO:
// next I fit some model to the absHumidities
// https://www.gnu.org/software/gsl/doc/html/lls.html#robust-linear-regression
// gsl_multifit_robust_est
// but I want the reverse, which to report a distribution of x (time) which we
// expect the RH to reach the desired level?
// There is also the question as to how much history to include
// which is to be decided based on a GCV type of score?
// in other words, run several regressions missing the last point, each with different amounts of history (up to the maximum)
// and then pick the one that best predicts?
// perhaps I should collect data dump it to usb and then have it on my computer
//

// two terminals one with:
// minicom -b 115200 -o -D /dev/ttyACM0 -C output
// while (true); do make && sudo picotool load -f hello_usb.uf2; inotifywait -e modify hello_usb.c; done

#define foreachBus(action) for(int i=0; i<2; i++) { \
        sensirion_i2c_select_bus(i);\
        action;\
}

// "nec_transmit.h" single function with lazy initialization
void ir_shutdown(void) {
     static int ir_tx_sm;
     static bool initialized = false;
     if (!initialized) ir_tx_sm = nec_tx_init(pio0, 28);
     initialized = true;
     pio_sm_put(pio0, ir_tx_sm, nec_encode_frame(0x0, 0x0));
}

int main() {
    stdio_init_all();

    gsl_rng * rng = gsl_rng_alloc(gsl_rng_default);

    bool light = 0;
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    int anyKey;
    do {
      printf("press space to blink IR, any other key starts recording\n");
      anyKey = getchar_timeout_us(1000000);
      sleep_ms(10);

      if (anyKey == ' ') ir_shutdown();

    } while (anyKey == PICO_ERROR_TIMEOUT || anyKey == ' ');

    srand(to_us_since_boot(get_absolute_time()));

    sensirion_i2c_init();
    
    int ok[2];
    do {
            foreachBus(ok[i] = sht3x_probe(SHT3X_ADDR));
            for (int i=0; i < 2 ; i++)
                    if (ok[i] != STATUS_OK) printf("sensor on bus %d failed status %d\n", i, ok[i]);
    } while (ok[0] != STATUS_OK || ok[1] != STATUS_OK);


    // t in milliseconds
    // mode is 0 LPM, 1 MPM, 2 HPM
    // j is for seeing about directly adjacent measurements
    // ret0, ret1 are what sht3x_measure returns (should be 0)
    // T0,T1 are in celsius
    // RH1,RH2 are percent relative humidity ie. on [0.000,100.000]
    // w1 w2 are g/m3 water vapor
    printf("T0, T1, RH0, RH1\n");
    while (true){

    int32_t temp[2], humidity[2];
    foreachBus(sht3x_measure_blocking_read(SHT3X_ADDR, temp+i, humidity+i));
    printf("%0.3f, %0.3f, %0.3f, %0.3f\n",
                    temp[0] / 1000.0f, temp[1] /1000.0f,
                    humidity[0] / 1000.0f, humidity[1]  /1000.0f);
    sleep_ms(1000);
    }

    // printf("\n\nt,mode,j,ret0,ret1,T0,T1,RH0,RH1,w1,w2\n");
    // while (true) {
    //   foreachBus(sht3x_set_power_mode(SHT3X_MEAS_MODE_HPM));
    //   foreachBus(sht3x_measure(SHT3X_ADDR));
    //   sleep_ms(15); // 4 or 6 for LPM MPM
    //   uint32_t time = to_ms_since_boot(get_absolute_time());
    //   int32_t temp[2], humidity[2];
    //   int8_t rets[2];
    //   foreachBus(rets[i] = sht3x_read(SHT3X_ADDR, temp+i, humidity+i));
    //   printf("%d, %d, %d, %0.3f, %0.3f, %0.3f, %0.3f, %0.4f, %0.4f\n",
    //                   time,
    //                   rets[0], rets[1],
    //                   temp[0] / 1000.0f, temp[1] / 1000.0f,
    //                   humidity[0] / 1000.0f, humidity[1] / 1000.0f,
    //                   absHumidity(temp[0]/1000.0f, humidity[0]/100000.0f),
    //                   absHumidity(temp[1]/1000.0f, humidity[1]/100000.0f));
    //  sleep_ms(1000);
    // }
    return 0;
}
