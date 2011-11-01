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
#define MPX_TERM "\r\n"

#define MPX_MAXLINE 256

#define MPX_OK 0    /*Ok*/
#define MPX_ERROR 1 /*Unknown Error*/
#define MPX_CMD 2   /*Command not known.*/
#define MPX_PARAM 3 /*Param out of range.*/
#define MPX_CONN 4  /*Not connected to detector.*/

/* Function prototypes*/
extern int mpxSet(const char *command, double value);
extern int mpxGet(const char *command, double *value);
extern int mpxData(unsigned int *data);
extern int mpxConnect(const char *host, int commandPort, int dataPort);
extern int mpxDisconnect(void); 
extern int mpxError(int error, char *errMsg);


#endif /* MPX_LOW_H */
