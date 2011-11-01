/**
 * Test program for interface library to medipix Labview system.
 * 
 * Matthew Pearson
 * Nov 2011
 */

#include "medipix_low.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) 
{
  double value = 0;
  int status = 0;

  printf("Test program for medipix_low interface library.\n");

  if ((status = mpxConnect("172.23.244.34", 14000, 14001)) != MPX_OK) {
    printf("ERROR Connecting. status: %d\n", status);
  }

  if ((status = mpxGet("NUMFRAMESTOACQUIRE", &value)) != MPX_OK) {
    printf("ERROR. status: %d\n", status);
  }
  printf("NUMFRAMESTOACQUIRE: %f\n", value);

  if ((status = mpxSet("NUMFRAMESTOACQUIRE", 2)) != MPX_OK) {
    printf("ERROR. status: %d\n", status);
  }

  if ((status = mpxGet("NUMFRAMESTOACQUIRE", &value)) != MPX_OK) {
    printf("ERROR. status: %d\n", status);
  }
  printf("NUMFRAMESTOACQUIRE: %f\n", value);

  if ((status = mpxDisconnect()) != MPX_OK) {
    printf("ERROR Disconnecting . status: %d\n", status);
  }

  if ((status = mpxGet("NUMFRAMESTOACQUIRE", &value)) != MPX_OK) {
    printf("ERROR. status: %d\n", status);
  }

  sleep(10);

  return EXIT_SUCCESS;
}
