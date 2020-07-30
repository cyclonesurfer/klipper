// ADXL345 sensor support
//
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>
#include <pigpiod_if2.h>
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <time.h>
#include "accel_values.h" // struct accel_values
#include "compiler.h" // __visible
#include "pyhelper.h" // errorf

#define SPI_CHANNEL 0
#define SPI_FREQUENCY 1600000
#define SPI_FLAGS 3 // POL & PHA

// ADXL345 register addresses
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39

// Multi-byte transfer
#define MULTI_BYTE 0x40

// Read bit
#define READ_BIT 0x80

// ADXL345 register values
#define DATA_RATE_3200 0xF
#define DATA_RANGE_16G_FULL 0xB
#define MEASURE_MODE 0x8
#define STANDBY_MODE 0x0
#define BYPASS_MODE 0x0
#define STREAMING_MODE 0x80
#define FIFO_ENTRIES_MASK 0x3F

#define SCALE_MULTIPLIER (0.004 * 9806.65) // 4mg/LSB * Earth gravity
#define READ_RATE 3200

#define FIFO_MIN_BUF 16

struct adxl345 {
    int pi, handle;
};

int
adxl345_write_reg(struct adxl345 *acc, char reg, char reg_val)
{
    char buf[2];
    buf[0] = reg | MULTI_BYTE;
    buf[1] = reg_val;
    int res = spi_write(acc->pi, acc->handle, buf, 2);
    if (res != 2)
        return res;
    return 0;
}

int
adxl345_read_reg(struct adxl345 *acc, char reg, char *reg_val)
{
    char buf[2];
    buf[0] = reg | READ_BIT | MULTI_BYTE;
    int res = spi_xfer(acc->pi, acc->handle, buf, buf, 2);
    if (res != 2)
        return res;
    *reg_val = buf[1];
    return 0;
}

int
adxl345_read(struct adxl345 *acc, double *ax, double *ay, double *az)
{
    char buf[7];
    buf[0] = DATAX0 | READ_BIT | MULTI_BYTE;
    int res = spi_xfer(acc->pi, acc->handle, buf, buf, 7);
    if (res != 7)
        return res;
    *ax = SCALE_MULTIPLIER * (int16_t)(buf[1] | (buf[2] << 8));
    *ay = SCALE_MULTIPLIER * (int16_t)(buf[3] | (buf[4] << 8));
    *az = SCALE_MULTIPLIER * (int16_t)(buf[5] | (buf[6] << 8));
    return 0;
}

int
adxl345_read_fifo(struct adxl345 *acc, int n,
        double *ax, double *ay, double *az)
{
    int i, res;
    for (i = 0; i < n; ++i)
        if ((res = adxl345_read(acc, ax + i, ay + i, az + i)) != 0)
            return res;
    return 0;
}

struct accel_values * __visible
adxl345_measure(struct adxl345 *acc, double duration)
{
    int n = ceil(duration * READ_RATE), res;
    if (n <= 0)
        return NULL;
    struct accel_values *values = accel_values_alloc(n);
    double delay = 1. / READ_RATE;

    if ((res = adxl345_write_reg(acc, POWER_CTL, MEASURE_MODE)) != 0)
        goto transfer_error;
    if ((res = adxl345_write_reg(acc, FIFO_CTL, STREAMING_MODE)) != 0)
        goto transfer_error;

    double ax, ay, az;
    char reg_val;
    // Cold read
    do {
        if ((res = adxl345_read_reg(acc, FIFO_STATUS, &reg_val)) != 0)
            goto transfer_error;
        reg_val &= FIFO_ENTRIES_MASK;
    } while (reg_val == 0);
    for (; reg_val > 0; --reg_val)
        if ((res = adxl345_read_fifo(acc, 1, &ax, &ay, &az)) != 0)
            goto transfer_error;

    // Actual reads
    int i;
    for (i = 0; i < n;) {
        double t = time_time();
        if ((res = adxl345_read_reg(acc, FIFO_STATUS, &reg_val)) != 0)
            goto transfer_error;
        reg_val &= FIFO_ENTRIES_MASK;
        if (reg_val >= n - i || reg_val >= FIFO_MIN_BUF) {
            if (reg_val > n - i) reg_val = n - i;
            if ((res = adxl345_read_fifo(acc, reg_val, values->ax + i
                            , values->ay + i, values->az + i)) != 0)
                goto transfer_error;
            for (; reg_val > 0; ++i, --reg_val)
                values->t[i] = delay * i;
        }
        double sleep = delay * (1
                + (n - i < FIFO_MIN_BUF ? n - i : FIFO_MIN_BUF))
            - (time_time() - t);
        if (sleep > 0)
            time_sleep(sleep);
    }

    if ((res = adxl345_write_reg(acc, FIFO_CTL, BYPASS_MODE)) != 0)
        goto transfer_error;
    if ((res = adxl345_write_reg(acc, POWER_CTL, STANDBY_MODE)) != 0)
        goto transfer_error;
    return values;

transfer_error:
    errorf("SPI transmissions failure: %d", res);
    adxl345_write_reg(acc, FIFO_CTL, BYPASS_MODE);
    adxl345_write_reg(acc, POWER_CTL, STANDBY_MODE);
    accel_values_free(values);
    return NULL;
}

struct adxl345 * __visible
adxl345_init()
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        errorf("PIGPIO initalization failed: %d", pi);
        return NULL;
    }
    int handle = spi_open(pi, SPI_CHANNEL, SPI_FREQUENCY, SPI_FLAGS);
    if (handle < 0) {
        pigpio_stop(pi);
        errorf("SPI initalization failed: %d", handle);
        return NULL;
    }
    struct adxl345 *acc = malloc(sizeof(*acc));
    memset(acc, 0, sizeof(*acc));
    acc->pi = pi;
    acc->handle = handle;

    int res;
    if ((res = adxl345_write_reg(acc, BW_RATE, DATA_RATE_3200)) != 0)
        goto init_error;
    if ((res = adxl345_write_reg(acc, DATA_FORMAT, DATA_RANGE_16G_FULL)) != 0)
        goto init_error;

    // Read a few cold values
    accel_values_free(adxl345_measure(acc, 0.1));
    return acc;

init_error:
    errorf("SPI transmissions failure: %d", res);
    spi_close(pi, handle);
    pigpio_stop(pi);
    return NULL;
}

void __visible
adxl345_free(struct adxl345 *acc)
{
    if (acc == NULL)
        return;
    spi_close(acc->pi, acc->handle);
    pigpio_stop(acc->pi);
    free(acc);
}
