/**
 * Interface library to medipix Labview system.
 * 
 * Matthew Pearson
 * Nov 2011
 */

#include "medipix_low.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
//#include <sys/wait.h>
#include <netinet/in.h>
#include <string.h>
//#include <signal.h>
#include <errno.h>
//#include <unistd.h>

/**
 * Set a value for the specified command.
 * @arg command - command name
 * @arg value - value to set (double)
 * @return int - error code
 */
int mpxSet(const char *command, double value) 
{
  printf("mpxSet. command: %s, value: %f\n", command, value);
  return MPX_OK;
}



int mpxGet(const char *command, double *value) 
{
  printf("mpxGet. command: %s\n", command);
  return MPX_OK;
}

int mpxData(unsigned int *data)
{

  return MPX_OK;
}

int mpxConnect(const char *host, int commandPort, int dataPort)
{
  static int connected = 0;
  int fd = 0;
  struct sockaddr_in server_addr;

  char *function = "mpxConnect";

  printf("mpxSet. host: %s, command port: %d, data port: %d\n", host, commandPort, dataPort);

  if (!connected) {
    /*Create a TCP socket*/
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      perror(function);
      return MPX_CONN;
    }
    
    /*Create and initialise a socket address structure*/
    memset(&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(host);
    server_addr.sin_port = htons(commandPort);
    
    /*Connect to the server.*/
    if ((connect(fd, (struct sockaddr*) &server_addr, sizeof(server_addr))) < 0) {
      perror(function);
      return MPX_CONN;
    }
    
    connected = 1;
  }
  
  return MPX_OK;
}

int mpxDisconnect(void) 
{
  printf("mpxDisconnect.\n");

  return MPX_OK;
}

/**
 * Return the error string for this error code.
 * @arg int - error number
 * @char pointer - pointer to char array. Must be at least MPX_MAXLINE bytes.
 */
int mpxError(int error, char *errMsg)
{
  if (error == MPX_OK) {
    strncpy(errMsg, "OK", MPX_MAXLINE);
  } else if (error == MPX_ERROR) {
    strncpy(errMsg, "Unknown Error", MPX_MAXLINE);
  } else if (error == MPX_CMD) {
    strncpy(errMsg, "Unknown Command", MPX_MAXLINE);
  } else if (error == MPX_PARAM) {
    strncpy(errMsg, "Param Out Of Range", MPX_MAXLINE);
  } else if (error == MPX_CONN) {
    strncpy(errMsg, "Not Connected To Detector", MPX_MAXLINE);
  } else {
    strncpy(errMsg, "Unknown Error Code", MPX_MAXLINE);
  }
  return MPX_OK;
}



