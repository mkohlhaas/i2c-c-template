#pragma once

#define VERBOSE 0

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
  bool     connected;    // Set to 1 when connected
  int      port;         // port
  char     model[16];    // model string
  char     serial[9];    // Serial number of USB device
  uint64_t uptime;       // time since boot (seconds)
  float    voltage_v;    // USB voltage (Volts)
  float    current_ma;   // device current (mA)
  float    temp_celsius; // temperature (C)
  char     mode;         // I2C 'I' or bitbang 'B' mode
  uint8_t  sda;          // SDA state, 0 or 1
  uint8_t  scl;          // SCL state, 0 or 1
  uint8_t  speed;        // I2C line speed (in kHz)
  uint8_t  pullups;      // pullup state (6 bits, 1=enabled)
  uint8_t  ccitt_crc;    // Hardware CCITT CRC
  uint8_t  e_ccitt_crc;  // Host CCITT CRC, should match
} I2CDriver;

int     i2c_start (I2CDriver *sd, uint8_t dev, uint8_t op);
void    i2c_stop (I2CDriver *sd);
void    i2c_read (I2CDriver *sd, uint8_t bytes[], size_t nn);
bool    i2c_write (I2CDriver *sd, const uint8_t bytes[], size_t nn);
bool    i2c_connect (I2CDriver *sd, const char *portname);
void    i2c_disconnect (I2CDriver *sd);
void    i2c_capture (I2CDriver *sd);
void    i2c_update_status (I2CDriver *sd);
void    i2c_monitor (I2CDriver *sd, bool enable);
uint8_t i2c_reset (I2CDriver *sd);
void    i2c_scan (I2CDriver *sd, uint8_t devices[128]);
