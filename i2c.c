#include "i2cdriver.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

int
i2c_commands (I2CDriver *sd, int argc, char *argv[])
{
  for (int i = 0; i < argc; i++)
    {
      char *token = argv[i];
      if (strlen (token) != 1)
        {
          goto badcommand;
        }
      switch (token[0])
        {
        case 'i':
          i2c_getstatus (sd);
          printf ("uptime %" SCNu64 "  %.3f V  %.0f mA  %.1f C SDA=%d SCL=%d speed=%d kHz\n", sd->uptime, sd->voltage_v,
                  sd->current_ma, sd->temp_celsius, sd->sda, sd->scl, sd->speed);
          break;

        case 'x':
          {
            uint8_t sda_scl = i2c_reset (sd);
            printf ("Bus reset. SDA = %d, SCL = %d\n", 1 & (sda_scl >> 1), 1 & sda_scl);
          }
          break;

        case 'd':
          {
            uint8_t devices[128];
            int     i;

            i2c_scan (sd, devices);
            printf ("\n");
            for (i = 8; i < 0x78; i++)
              {
                if (devices[i] == '1')
                  {
                    printf ("%02x  ", i);
                  }
                else
                  {
                    printf ("--  ");
                  }
                if ((i % 8) == 7)
                  {
                    printf ("\n");
                  }
              }
            printf ("\n");
          }
          break;

        case 'w':
          {
            token            = argv[++i];
            unsigned int dev = strtol (token, NULL, 0);

            token = argv[++i];
            uint8_t bytes[8192];
            char   *endptr = token;
            size_t  nn     = 0;
            while (nn < sizeof (bytes))
              {
                bytes[nn++] = strtol (endptr, &endptr, 0);
                if (*endptr == '\0')
                  {
                    break;
                  }
                if (*endptr != ',')
                  {
                    fprintf (stderr, "Invalid bytes '%s'\n", token);
                    return 1;
                  }
                endptr++;
              }

            i2c_start (sd, dev, 0);
            i2c_write (sd, bytes, nn);
          }
          break;

        case 'r':
          {
            token            = argv[++i];
            unsigned int dev = strtol (token, NULL, 0);

            token      = argv[++i];
            size_t  nn = strtol (token, NULL, 0);
            uint8_t bytes[8192];

            i2c_start (sd, dev, 1);
            i2c_read (sd, bytes, nn);
            i2c_stop (sd);

            size_t i;
            for (i = 0; i < nn; i++)
              {
                printf ("%s0x%02x", i ? "," : "", 0xff & bytes[i]);
              }
            printf ("\n");
          }
          break;

        case 'p':
          i2c_stop (sd);
          break;

        case 'm':
          {
            char line[100];

            i2c_monitor (sd, 1);
            printf ("[Hit return to exit monitor mode]\n");
            fgets (line, sizeof (line) - 1, stdin);
            i2c_monitor (sd, 0);
          }
          break;

        case 'c':
          {
            i2c_capture (sd);
          }
          break;

        default:
        badcommand:
          fprintf (stderr, "Bad command '%s'\n", token);
          fprintf (stderr, "\n");
          fprintf (stderr, "Commands are:");
          fprintf (stderr, "\n");
          fprintf (stderr, "  i              display status information (uptime, voltage, current, temperature)\n");
          fprintf (stderr, "  x              I2C bus reset\n");
          fprintf (stderr, "  d              device scan\n");
          fprintf (stderr, "  w dev <bytes>  write bytes to I2C device dev\n");
          fprintf (stderr, "  p              send a STOP\n");
          fprintf (stderr, "  r dev N        read N bytes from I2C device dev, then STOP\n");
          fprintf (stderr, "  m              enter I2C bus monitor mode\n");
          fprintf (stderr, "  c              enter I2C bus capture mode\n");
          fprintf (stderr, "\n");

          return EXIT_FAILURE;
        }
    }

  return EXIT_SUCCESS;
}
int
main (int argc, char *argv[])
{
  I2CDriver i2c;
  if (argc < 2)
    {
      printf ("Usage: i2ccl <PORTNAME> <commands>\n");
      exit (EXIT_FAILURE);
    }
  else
    {
      i2c_connect (&i2c, argv[1]);
      if (!i2c.connected)
        {
          exit (EXIT_FAILURE);
        }
      return i2c_commands (&i2c, argc - 2, argv + 2);
    }
}
