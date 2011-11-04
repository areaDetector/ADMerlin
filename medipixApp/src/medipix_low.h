/**
 * Interface library to medipix Labview system.
 * 
 * Matthew Pearson
 * Nov 2011
 */

#ifndef MPX_LOW_H
#define MPX_LOW_H

/* Fixed protocol names*/
#define MPX_HEADER "MPX"
#define MPX_SET "SET"
#define MPX_GET "GET"
#define MPX_COMMAND "CMD"

#define MPX_MAXLINE 256
#define MPX_DATAFRAME 128000

#define MPX_OK 0    /*Ok*/
#define MPX_ERROR 1 /*Unknown Error*/
#define MPX_CMD 2   /*Command not known.*/
#define MPX_PARAM 3 /*Param out of range.*/
#define MPX_CONN 100  /*Not connected to detector.*/
#define MPX_WRITE 101 /*Error writing to socket.*/
#define MPX_READ 102 /*Error reading from socket.*/
#define MPX_LEN 103 /*Length of command and value too long, or NULL*/
#define MPX_DATA 110 /*Data formatting error.*/

/* Function prototypes*/
extern int mpxSet(const char *command, const char *value);
extern int mpxGet(const char *command, char *value);
extern int mpxCmd(const char *command);
extern int mpxData(unsigned int *data);
extern int mpxConnect(const char *host, int commandPort, int dataPort);
extern int mpxIsConnected(int *conn);
extern int mpxDisconnect(void); 
extern int mpxError(int error, char *errMsg);

/* local static function prototypes*/
static int mpxWriteRead(const char *buff, char *response);
static int mpxRead(char *input);
static int mpxWrite(const char *buff);

//static void * mpxDataThread(void *data_fd);
//static int mpxReadData(int data_fd) 

#endif /* MPX_LOW_H */
