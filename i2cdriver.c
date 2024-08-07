#include "i2cdriver.h"
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// ****************************   Serial port  ********************************

static int
openTerminalToFT230 (const char *portname)
{

  int fd = open (portname, O_RDWR | O_NOCTTY);
  if (fd == -1)
    {
      perror (portname);
      return -1;
    }
  struct termios term_settings;
  tcgetattr (fd, &term_settings);

  cfsetispeed (&term_settings, B1000000);
  cfsetospeed (&term_settings, B1000000);

  cfmakeraw (&term_settings);
  term_settings.c_cc[VMIN] = 1;
  if (tcsetattr (fd, TCSANOW, &term_settings) != 0)
    {
      perror ("Serial settings");
      return -1;
    }

  return fd;
}

static size_t
readFromFT230 (int fd, uint8_t *b, size_t s)
{
  ssize_t n;
  size_t  t = 0;
  while (t < s)
    {
      n = read (fd, b + t, s);
      if (n > 0)
        {
          t += n;
        }
    }

#if VERBOSE
  printf (" READ %zu %zd: ", s, n);
  for (size_t i = 0; i < s; i++)
    {
      printf ("%02x ", 0xff & b[i]);
    }
  printf ("\n");
#endif

  return s;
}

static void
writeToFT230 (int fd, const uint8_t *b, size_t s)
{
  write (fd, b, s);

#if VERBOSE
  printf ("WRITE %zu: ", s);
  for (size_t i = 0; i < s; i++)
    {
      printf ("%02x ", 0xff & b[i]);
    }
  printf ("\n");
#endif
}

static void
closeSerialPort (int hSerial)
{
  close (hSerial);
}

// ******************************  CCITT CRC  *********************************

static const uint16_t crc_table[256]
    = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
        0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a,
        0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
        0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
        0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87,
        0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
        0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290,
        0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e,
        0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f,
        0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
        0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83,
        0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
        0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

static void
crc_update (I2CDriver *sd, const uint8_t *data, size_t data_len)
{
  unsigned int tbl_idx;
  uint16_t     crc = sd->e_ccitt_crc;

  while (data_len--)
    {
      tbl_idx = ((crc >> 8) ^ *data) & 0xff;
      crc     = (crc_table[tbl_idx] ^ (crc << 8)) & 0xffff;
      data++;
    }
  sd->e_ccitt_crc = crc;
}

// ******************************  I2CDriver  *********************************

// Connect to `FT230` chip via USB.
// Returns `true` on successful connection.
bool
i2c_connect (I2CDriver *sd, const char *portname)
{
  *sd      = (I2CDriver){};
  sd->port = openTerminalToFT230 (portname);
  if (sd->port == -1)
    {
      return false;
    }

  // Use echo command. Test all uint8_ts.
  uint8_t const tests[] = {
    'A',
    '\r',
    '\n',
    0xff,
  };

  for (unsigned long i = 0; i < sizeof tests; i++)
    {
      uint8_t tx[] = {
        'e',
        tests[i],
      };
      writeToFT230 (sd->port, tx, sizeof tx);
      uint8_t rx;
      readFromFT230 (sd->port, &rx, sizeof rx);
      if (rx != tests[i])
        {
          return false;
        }
    }

  i2c_update_status (sd);
  sd->connected   = true;
  sd->e_ccitt_crc = sd->ccitt_crc;
  return true;
}

void
i2c_disconnect (I2CDriver *sd)
{
  if (sd->connected)
    {
      closeSerialPort (sd->port);
    }
  *sd = (I2CDriver){};
}

static void
charCommand (I2CDriver *sd, char c)
{
  writeToFT230 (sd->port, (uint8_t *)&c, 1);
}

static bool
i2c_ack (I2CDriver *sd)
{
  uint8_t a;
  readFromFT230 (sd->port, &a, sizeof a);
  return (a & 1) != 0;
}

void
i2c_update_status (I2CDriver *sd)
{
  uint8_t response_size = 80;
  uint8_t readbuffer[response_size + 1];
  int     bytesRead;

  charCommand (sd, '?');
  bytesRead = readFromFT230 (sd->port, readbuffer, response_size);

#ifdef VERBOSE
  // TODO: add log functions
  // show bytes, readbuffer
  // fprintf(stderr, "%d Bytes were read: %.*s\n", bytesRead, bytesRead, readbuffer);
#endif

  readbuffer[bytesRead] = 0;
  sscanf ((char *)readbuffer, "[%15s %8s %" SCNu64 "%f %f %f %c %" SCNu8 "%" SCNu8 "%" SCNu8 "%" SCNu8 "%" SCNu8 "]",
          sd->model, sd->serial, &sd->uptime, &sd->voltage_v, &sd->current_ma, &sd->temp_celsius, &sd->mode, &sd->sda,
          &sd->scl, &sd->speed, &sd->pullups, &sd->ccitt_crc);
}

void
i2c_scan (I2CDriver *sd, uint8_t devices[128])
{
  charCommand (sd, 'd');
  readFromFT230 (sd->port, devices + 8, 112);
}

uint8_t
i2c_reset (I2CDriver *sd)
{
  charCommand (sd, 'x');
  uint8_t a[1];
  if (readFromFT230 (sd->port, a, 1) != 1)
    {
      return 0;
    }
  return a[0];
}

int
i2c_start (I2CDriver *sd, uint8_t dev, uint8_t rw)
{
  uint8_t start[2] = {
    's',
    (dev << 1) | rw,
  };
  writeToFT230 (sd->port, start, sizeof start);
  return i2c_ack (sd);
}

void
i2c_stop (I2CDriver *sd)
{
  charCommand (sd, 'p');
}

bool
i2c_write (I2CDriver *sd, const uint8_t bytes[], size_t nn)
{
  bool succesful_write = true;

  for (size_t i = 0; i < nn; i += 64)
    {
      size_t  len     = ((nn - i) < 64) ? (nn - i) : 64;
      uint8_t cmd[65] = { (uint8_t)(0xc0 + len - 1) };
      memcpy (cmd + 1, bytes + i, len);
      writeToFT230 (sd->port, cmd, 1 + len);
      succesful_write &= i2c_ack (sd);
    }
  if (succesful_write)
    {
      crc_update (sd, bytes, nn);
    }
  return succesful_write;
}

void
i2c_read (I2CDriver *sd, uint8_t bytes[], size_t nn)
{
  size_t i;

  for (i = 0; i < nn; i += 64)
    {
      size_t  rest   = nn - 1;
      size_t  len    = rest < 64 ? rest : 64;
      uint8_t cmd[1] = {
        0x80 + len - 1,
      };
      writeToFT230 (sd->port, cmd, 1);
      readFromFT230 (sd->port, bytes + i, len);
      crc_update (sd, bytes + i, len);
    }
}

void
i2c_monitor (I2CDriver *sd, bool enable)
{
  charCommand (sd, enable ? 'm' : ' ');
}

void
i2c_capture (I2CDriver *sd)
{
  printf ("Capture started\n");
  charCommand (sd, 'c');
  uint8_t byte;
  int     starting = 0;
  int     nbits    = 0;
  int     bits     = 0;

  while (true)
    {
      readFromFT230 (sd->port, &byte, 1);
      for (int i = 0; i < 2; i++)
        {
          int symbol = (i == 0) ? (byte >> 4) : (byte & 0xf);
          switch (symbol)
            {
            case 0:
              break;
            case 1:
              starting = 1;
              break;
            case 2:
              printf ("STOP\n");
              starting = 1;
              break;
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
              bits = (bits << 3) | (symbol & 7);
              nbits += 3;
              if (nbits == 9)
                {
                  int b8 = (bits >> 1), ack = !(bits & 1);
                  if (starting)
                    {
                      starting = 0;
                      printf ("START %02x %s", b8 >> 1, (b8 & 1) ? "READ" : "WRITE");
                    }
                  else
                    {
                      printf ("BYTE %02x", b8);
                    }
                  printf (" %s\n", ack ? "ACK" : "NAK");
                  nbits = 0;
                  bits  = 0;
                }
            }
        }
    }
}
