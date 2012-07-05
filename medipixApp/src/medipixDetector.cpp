/* medipixDetector.cpp
 *
 * This is a driver for a Medipix 3 detector chip.
 *
 * The driver is designed to communicate with the chip via the matching Labview controller over TCP/IP
 *
 * Author: Giles Knap
 *         Diamond Light Source Ltd.
 *
 * Created:  Jan 06 2011
 *

 * Original Source from pilatusDetector by Mark Rivers
 *
 */

#define DEBUG 1

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <time.h>

// #include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>

#include "ADDriver.h"

#include "medipix_low.h"

/** Messages to/from Labview command channel */
#define MAX_MESSAGE_SIZE 256 
#define MAX_FILENAME_LEN 256
#define MAX_BAD_PIXELS 100
/** Time to poll when reading from Labview */
#define ASYN_POLL_TIME .01 
#define Labview_DEFAULT_TIMEOUT 2.0
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

/** Trigger modes */
typedef enum
{
    TMInternal,
    TMExternalEnable,
    TMExternalTriggerHigh,
    TMExternalTriggerLow,
    TMExternalTriggerRising,
} medipixTriggerMode;/** Trigger modes */

/** Medipix Individual Trigger types */

#define TMTrigInternal  "0"
#define TMTrigRising 	"1"
#define TMTrigFalling 	"2"


/** data header types */
typedef enum
{
    MPXDataHeaderNone,
    MPXDataHeader12,
    MPXDataHeader24,
    MPXAcquisitionHeader,
    MPXUnknownHeader
} medipixDataHeader;

static const char *driverName = "medipixDetector";

#define medipixDelayTimeString      "DELAY_TIME"
#define medipixThreshold0String     "THRESHOLD0"
#define medipixThreshold1String     "THRESHOLD1"
#define medipixOperatingEnergyString "OPERATINGENERGY"

#define medipixThresholdApplyString "THRESHOLD_APPLY"
#define medipixThresholdAutoApplyString "THRESHOLD_AUTO_APPLY"
#define medipixArmedString          "ARMED"

#define medipixmedpixThresholdScanString "THRESHOLDSCAN"
#define medipixStartThresholdScanString	"THRESHOLDSTART"
#define medipixStopThresholdScanString	"THRESHOLDSTOP"
#define medipixStepThresholdScanString	"THRESHOLDSTEP"
#define medipixStartThresholdScanningString	"STARTTHRESHOLDSCANNING"
#define medipixCounterDepthString	"COUNTERDEPTH"
#define medipixResetString "RESET"

/** Driver for Dectris medipix pixel array detectors using their Labview server over TCP/IP socket */
class medipixDetector: public ADDriver
{
public:
	medipixDetector(const char *portName, const char *LabviewCmdPort,
			const char *LabviewDataPort, int maxSizeX, int maxSizeY,
			int maxBuffers, size_t maxMemory, int priority, int stackSize);

	/* These are the methods that we override from ADDriver */
	virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
	virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
	virtual asynStatus writeOctet(asynUser *pasynUser, const char *value,
			size_t nChars, size_t *nActual);
	void report(FILE *fp, int details);
	void medipixTask(); /* This should be private but is called from C so must be public */
	void medipixStatus(); /* This should be private but is called from C so must be public */

protected:
	int medipixDelayTime;
#define FIRST_medipix_PARAM medipixDelayTime
	int medipixThreshold0;
	int medipixThreshold1;
	int medipixOperatingEnergy;
	int medipixThresholdApply;
	int medipixThresholdAutoApply;
	int medipixArmed;
	int medipixThresholdScan;
	int medipixStartThresholdScan;
	int medipixStopThresholdScan;
	int medipixStepThresholdScan;
	int medipixStartThresholdScanning;
	int medipixTvxVersion;
	int medipixCounterDepth;
	int medipixReset;

#define LAST_medipix_PARAM medipixReset

private:
	/* These are the methods that are new to this class */
	void abortAcquisition();
	asynStatus setAcquireParams();
	asynStatus getThreshold();
    asynStatus updateThresholdScanParms();
    void parseDataFrame(NDArray* pImage, const char* header);
    medipixDataHeader parseDataHeader(const char* header);

	/* The labview communication primitives */
	asynStatus mpxGet(char* valueId, double timeout);
	asynStatus mpxSet(char* valueId, char* value, double timeout);
	asynStatus mpxCommand(char* commandId, double timeout);
	asynStatus mpxWrite(double timeout);
	asynStatus mpxReadCmd(char* cmdType, char* cmdName, double timeout);
	asynStatus mpxWriteRead(char* cmdType, char* cmdName ,double timeout);
	asynStatus mpxRead(asynUser* pasynUser, char* bodyBuf, int bufSize, int* bytesRead, double timeout);

	/* Our data */
	int imagesRemaining;
	NDArray *pFlatField;
	int multipleFileNumber;
	asynUser *pasynLabViewCmd;
	asynUser *pasynLabViewData;
	double averageFlatField;

	bool startingUp;  // used to avoid very chatty initialisation

	/* input and output from the labView controller     */
	char toLabview[MPX_MAXLINE];
	char fromLabview[MPX_MAXLINE];
	char fromLabviewHeader[MPX_MAXLINE];
	char fromLabviewBody[MPX_MAXLINE];
	char fromLabviewValue[MPX_MAXLINE];
	int fromLabviewError;

	char LabviewCommandPortName[20];
	char LabviewDataPortName[20];
};

#define NUM_medipix_PARAMS (&LAST_medipix_PARAM - &FIRST_medipix_PARAM + 1)

// #######################################################################################
// ##################### Labview communications primitives ###############################
// #######################################################################################

asynStatus medipixDetector::mpxSet(char* valueId, char* value, double timeout)
{
	int buff_len = 0;
	int msg_len = 0;
	asynStatus status;
	char *tok = NULL;

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;

	if ((valueId == NULL))
		return asynError;

	// Build up command to be sent.
	// length is header + length specifier (10 decimal digits) + "SET" + length of variable name + 3 commas
	buff_len = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + strlen(MPX_SET) + strlen(value)
			+ strlen(valueId) + 4;
	if (buff_len > MPX_MAXLINE)
		return asynError;

	// the message length specifier contains the count of characters including the ',' after itself
	// i.e. total length minus the header length (including 1 comma)
	msg_len = buff_len - MPX_MSG_LEN_DIGITS - strlen(MPX_HEADER) - 1;

	sprintf(toLabview, "%s,%010u,%s,%s,%s", MPX_HEADER, msg_len, MPX_SET, valueId, value);

	if ((status = mpxWriteRead(MPX_SET, valueId, timeout)) != asynSuccess)
	{
		return status;
	}

	// items in the response are comma delimited
	tok = strtok(fromLabviewBody, ",");
	tok = strtok(NULL, ",");

	// 3rd Item is Error Number
	tok = strtok(NULL, ",");
	if (tok == NULL)
		return asynError;
	fromLabviewError = atoi(tok);

	if (fromLabviewError != MPX_OK)
		return asynError;

	return asynSuccess;
}

asynStatus medipixDetector::mpxCommand(char* commandId, double timeout)
{
	int buff_len = 0;
	int msg_len = 0;
	asynStatus status;
	char *tok = NULL;

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;

	if ((commandId == NULL))
		return asynError;

	// Build up command to be sent.
	// length is header + length specifier (10 decimal digits) + "SET" + length of variable name + 3 commas
	buff_len = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + strlen(MPX_CMD)
			+ strlen(commandId) + 3;
	if (buff_len > MPX_MAXLINE)
		return asynError;

	// the message length specifier contains the count of characters including the ',' after itself
	// i.e. total length minus the header length (including 1 comma)
	msg_len = buff_len - MPX_MSG_LEN_DIGITS - strlen(MPX_HEADER) - 1;

	sprintf(toLabview, "%s,%010u,%s,%s", MPX_HEADER, msg_len, MPX_CMD, commandId);

	if ((status = mpxWriteRead(MPX_CMD, commandId, timeout)) != asynSuccess)
	{
		return status;
	}

	// items in the response are comma delimited
	tok = strtok(fromLabviewBody, ",");
	tok = strtok(NULL, ",");

	// 3rd Item is Error Number
	tok = strtok(NULL, ",");
	if (tok == NULL)
		return asynError;
	fromLabviewError = atoi(tok);

	if (fromLabviewError != MPX_OK)
		return asynError;
	return asynSuccess;
}

/**
 * Get the specified named value from Labview
 */
asynStatus medipixDetector::mpxGet(char* valueId, double timeout)
{
	int buff_len = 0;
	int msg_len = 0;
	asynStatus status;
	char *tok = NULL;

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;

	if ((valueId == NULL))
	{
		return asynError;
	}

	// Build up command to be sent.
	// length is header + length specifier (10 decimal digits) + "GET" + length of variable name + 3 commas
	buff_len = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + strlen(MPX_GET)
			+ strlen(valueId) + 3;
	if (buff_len > MPX_MAXLINE)
	{
		return asynError;
	}
	// the message length specifier contains the count of characters including the ',' after itself
	// i.e. total length minus the header length (including 1 comma)
	msg_len = buff_len - MPX_MSG_LEN_DIGITS - strlen(MPX_HEADER) - 1;

	sprintf(toLabview, "%s,%010u,%s,%s", MPX_HEADER, msg_len, MPX_GET, valueId);

	if ((status = mpxWriteRead(MPX_GET, valueId, timeout)) != asynSuccess)
	{
		return status;
	}

	// items in the response are comma delimited
	tok = strtok(fromLabviewBody, ",");
	tok = strtok(NULL, ",");

	// 3rd Item is Value
	tok = strtok(NULL, ",");
	if (tok == NULL)
		return asynError;

	strncpy(fromLabviewValue, tok, MPX_MAXLINE);

	// 4th Item is Error Number
	tok = strtok(NULL, ",");
	if (tok == NULL)
		return asynError;
	fromLabviewError = atoi(tok);

	if (fromLabviewError != MPX_OK)
		return asynError;

	return asynSuccess;
}

/**
 * Send a request to labview
 */
asynStatus medipixDetector::mpxWrite(double timeout)
{
	char *functionName = "mpxWrite";
	size_t nwrite;
	asynStatus status;

#ifdef DEBUG
	printf("mpxWrite: Request: %s\n", toLabview);
#endif

	// pasynOctetSyncIO->flush(this->pasynLabViewCmd);
	status = pasynOctetSyncIO->write(this->pasynLabViewCmd, this->toLabview,
			strlen(this->toLabview), timeout, &nwrite);
	// make sure buffers are written out for short messages
	// pasynOctetSyncIO->flush(this->pasynLabViewCmd);

	if (status)
		asynPrint(this->pasynLabViewCmd, ASYN_TRACE_ERROR,
				"%s:%s, status=%d, sent\n%s\n", driverName, functionName,
				status, this->toLabview);

	setStringParam(ADStringToServer, this->toLabview);

	return asynSuccess;
}

/**
 * Reads in a raw MPX frame from a pasynOctetSyncIO handle
 *
 * This function skips any leading data, looking for the pattern
 * MPX,0000000000,
 *
 * Where
 * 		0000000000 = the no. of bytes in body of the
 * 			frame in decimal (inclusive of comma after 000000000)
 * Reads the rest of the body into the passed bodyBuf
 *
 */
asynStatus medipixDetector::mpxRead(asynUser* pasynUser, char* bodyBuf, int bufSize,
		int* bytesRead, double timeout)
{
	size_t nread = 0;
	asynStatus status = asynSuccess;
	int eomReason;
	const char *functionName = "mpxRead";
	int headerSize = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + 2;
	int mpxLen = strlen(MPX_HEADER);
	int bodySize;
	int readCount = 0;
	int leadingJunk = 0;
	int headerChar = 0;

	char headerStr[] = MPX_HEADER;
	char* tok;
	char header[MPX_MAXLINE];


	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;
	// clear previous contents of buffer in case of error
	bodyBuf[0] = 0;
	*bytesRead = 0;

	// look for MPX in the stream, throw away any preceding data
	// this is to re-synch with server after an error or reboot
	while(headerChar < mpxLen)
	{
		status = pasynOctetSyncIO->read(pasynUser, header + headerChar, 1,
					timeout, &nread, &eomReason);
		if(status != asynSuccess)
			return status;

		if(header[headerChar] == headerStr[headerChar])
		{
			headerChar++;
		}
		else
		{
			leadingJunk++;
			headerChar = 0;
		}
	}

	if(leadingJunk > 0)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s, status=%d %d bytes of leading garbage discarded before header %s \n",
				driverName, functionName, status, leadingJunk,
				this->fromLabview);
	}

	// read the rest of the header block including message length
	readCount = 0;
    do
    {
        status = pasynOctetSyncIO->read(pasynUser, header + mpxLen + readCount,
                    (headerSize - mpxLen) - readCount,
                    timeout, &nread, &eomReason);
        if(status == asynSuccess)
            readCount += nread;
    } while (nread != 0 && readCount < (headerSize - mpxLen) && status == asynSuccess);

	if (status != asynSuccess)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s, timeout=%f, status=%d received %d bytes\n%s\n",
				driverName, functionName, timeout, status, readCount,
				this->fromLabview);
	}
	else
	{
	    if(readCount != (headerSize - mpxLen))
            return asynError;

		// terminate the response for string handling
		header[readCount + mpxLen] = (char) NULL;
		strncpy(fromLabviewHeader, header, MPX_MAXLINE);

	#ifdef DEBUG
		printf("mpxRead: Response Header: %s\n", header);
	#endif

		// parse the header
		tok = strtok(header, ","); // this first element already verified above

		tok = strtok(NULL, ",");
		if (tok == NULL)
			return asynError;

		// subtract one from bodySize since we already read the 1st comma
		bodySize = atoi(tok) - 1;

		if (bodySize == 0 || bodySize >= bufSize)
			return asynError;

		// now read the rest of the message (the body)
	    readCount = 0;
	    do
		{
			status = pasynOctetSyncIO->read(pasynUser, bodyBuf + readCount,
			    bodySize - readCount,
				timeout, &nread, &eomReason);
			if(status == asynSuccess)
			    readCount += nread;
		} while (nread != 0 && readCount < bodySize && status == asynSuccess);

		if(readCount < bodySize)
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR,
					"%s:%s, timeout=%f, status=%d received %d bytes in MPX command body, expected %d\n",
					driverName, functionName, timeout, status, readCount, bodySize);
			fromLabviewError = MPX_ERR_LEN;
			return status = asynSuccess ? asynError : status;
		}

		*bytesRead = readCount;
	}

	fromLabviewError = MPX_OK;
	return status;
}

/**
 * Reads in the MPX command header and body from labview
 * verifies the header and places the body in this->fromLabviewBody
 * for parsing by the caller
 */
asynStatus medipixDetector::mpxReadCmd(char* cmdType, char* cmdName ,double timeout)
{
	char* functionName = "mpxReadCmd";
	int nread = 0;
	asynStatus status = asynSuccess;
	char buff[MPX_MAXLINE];
	char* tok;

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;
	// clear out previous values for the member variables for response text
	fromLabview[0] = 0;
	fromLabviewBody[0] = 0;
	fromLabviewHeader[0] = 0;
	fromLabviewValue[0] = 0;

	// repeat read of responses until correct response is received or no more
	// data is available - this approach re-syncs client and server after an error or reboot
	// at either end
	while(status == asynSuccess)
	{
		status = mpxRead(this->pasynLabViewCmd, buff, MPX_MAXLINE, &nread, timeout);

		if (status == asynSuccess)
		{
			// terminate the response for string handling
			buff[nread] = (char) NULL;
			// update the member variables with relevant parts of the response
			strncpy(fromLabviewBody, buff, MPX_MAXLINE);
			strncpy(fromLabview, fromLabviewHeader, MPX_MAXLINE);
			strncat(fromLabview, fromLabviewBody, MPX_MAXLINE);
			// Set output string so it can get back to EPICS
			setStringParam(ADStringFromServer, this->fromLabview);

			// items in the response are comma delimited -
			// 1st item is the command type
			tok = strtok(buff, ",");
			if (!(tok == NULL || strncmp(cmdType, tok, MPX_MAXLINE)))
			{
				// 2nd item is command (or variable) name which should be echoed back
				tok = strtok(NULL, ",");
				if (!(tok == NULL || strncmp(cmdName, tok, MPX_MAXLINE)))
					break; // success - exit the while loop
			}
		}

		// if we get here then the expected response was not received
		// report an error and retry

		asynPrint(this->pasynLabViewCmd, ASYN_TRACE_ERROR,
				"%s:%s error, status=%d unexpected response from labview: '%s%s'\n", driverName,
				functionName, status, fromLabviewHeader, fromLabviewBody);

		fromLabviewHeader[0] = 0; // clear for consistent error reporting
        fromLabviewBody[0] = 0;
        fromLabview[0] = 0;
	}

#ifdef DEBUG
	printf("mpxReadCmd: Full Response: %s\n", fromLabview);
#endif

	fromLabviewError = MPX_OK;
	return status;
}

asynStatus medipixDetector::mpxWriteRead(char* cmdType, char* cmdName ,double timeout)
{
	asynStatus status;

	// this->lock(); // make sure commands from different threads are not interleaved
	// removed above because I do not believe you can nest locks and the following unlock
	// would therefore free the AsynPort Thread when called from WriteInt32 for example

	if ((status = mpxWrite(timeout)) != asynSuccess)
	{
		return status;
	}

	if ((status = mpxReadCmd(cmdType, cmdName ,timeout)) != asynSuccess)
	{
		return status;
	}

	// removed - see above comment
	// this->unlock();

	return asynSuccess;
}

// #######################################################################################
// ##################### END OF Labview communications primitives ########################
// #######################################################################################

asynStatus medipixDetector::setAcquireParams()
{
	int ival;
	double dval;
	double dval2;
	int triggerMode;
	char value[MPX_MAXLINE];
	asynStatus status;
//	char *substr = NULL;
//	int pixelCutOff = 0;

	// avoid chatty startup which keeps setting these values
	if(startingUp)
	    return asynSuccess;

	status = getIntegerParam(ADTriggerMode, &triggerMode);
	if (status != asynSuccess)
		triggerMode = TMInternal;
	// medipix individually controls how start and stop triggers are read
	// here we translate the chosen trigger mode into a combination of start
	// and stop modes
	switch(triggerMode)
	{
	case TMInternal :
		this->mpxSet(MPXVAR_TRIGGERSTART, TMTrigInternal, Labview_DEFAULT_TIMEOUT);
		this->mpxSet(MPXVAR_TRIGGERSTOP, TMTrigInternal, Labview_DEFAULT_TIMEOUT);
		break;
	case TMExternalEnable :
		this->mpxSet(MPXVAR_TRIGGERSTART, TMTrigRising, Labview_DEFAULT_TIMEOUT);
		this->mpxSet(MPXVAR_TRIGGERSTOP, TMTrigFalling, Labview_DEFAULT_TIMEOUT);
		break;
	case TMExternalTriggerLow :
		this->mpxSet(MPXVAR_TRIGGERSTART, TMTrigFalling, Labview_DEFAULT_TIMEOUT);
		this->mpxSet(MPXVAR_TRIGGERSTOP, TMTrigInternal, Labview_DEFAULT_TIMEOUT);
		break;
    case TMExternalTriggerHigh :
        this->mpxSet(MPXVAR_TRIGGERSTART, TMTrigRising, Labview_DEFAULT_TIMEOUT);
        this->mpxSet(MPXVAR_TRIGGERSTOP, TMTrigInternal, Labview_DEFAULT_TIMEOUT);
        break;
    case TMExternalTriggerRising :
        this->mpxSet(MPXVAR_TRIGGERSTART, TMTrigRising, Labview_DEFAULT_TIMEOUT);
        this->mpxSet(MPXVAR_TRIGGERSTOP, TMTrigRising, Labview_DEFAULT_TIMEOUT);
		break;
	}

	status = getIntegerParam(ADNumImages, &ival);
	if ((status != asynSuccess) || (ival < 1))
	{
		ival = 1;
		setIntegerParam(ADNumImages, ival);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%d", ival);
	this->mpxSet(MPXVAR_NUMFRAMESTOACQUIRE, value, Labview_DEFAULT_TIMEOUT);

	status = getIntegerParam(ADNumExposures, &ival);
	if ((status != asynSuccess) || (ival < 1))
	{
		ival = 1;
		setIntegerParam(ADNumExposures, ival);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%d", ival);
	this->mpxSet(MPXVAR_NUMFRAMESPERTRIGGER, value, Labview_DEFAULT_TIMEOUT);

	status = getIntegerParam(medipixCounterDepth, &ival);
	if ((status != asynSuccess) || (ival != 12 && ival != 24)) // currently limited to 12/24 bit
	{
		ival = 12;
		setIntegerParam(medipixCounterDepth, ival);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%d", ival);
	this->mpxSet(MPXVAR_COUNTERDEPTH, value, Labview_DEFAULT_TIMEOUT);

	status = getDoubleParam(ADAcquireTime, &dval);
	if ((status != asynSuccess) || (dval < 0.))
	{
		dval = 1.;
		setDoubleParam(ADAcquireTime, dval);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%f", dval*1000); // translated into millisec
	this->mpxSet(MPXVAR_ACQUISITIONTIME, value, Labview_DEFAULT_TIMEOUT);

	status = getDoubleParam(ADAcquirePeriod, &dval2);
	if ((status != asynSuccess) || (dval2 < 0.))
	{
		dval = 2.;
		setDoubleParam(ADAcquirePeriod, dval2);
	}
	else if(dval2 - dval < 0)
	{
		dval2 = dval + 0.00085; // this is hard coded readback time but only used in error condition, see below
		setDoubleParam(ADAcquirePeriod, dval2);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%f", dval2*1000); // translated into millisec
	this->mpxSet(MPXVAR_ACQUISITIONPERIOD, value, Labview_DEFAULT_TIMEOUT);
	// read the acquire period back from the server so that it can insert
	// the readback time if necessary
	this->mpxGet(MPXVAR_ACQUISITIONPERIOD, Labview_DEFAULT_TIMEOUT);
	if(status == asynSuccess)
		setDoubleParam(ADAcquirePeriod, atof(fromLabviewValue)/1000); // translated into secs

	return (asynSuccess);

}

asynStatus medipixDetector::getThreshold()
{
	int status;

    if(startingUp)
        return asynSuccess;

	/* Read back the actual setting, in case we are out of bounds.*/
	status = mpxGet(MPXVAR_THRESHOLD0, Labview_DEFAULT_TIMEOUT);
	if(status == asynSuccess)
		setDoubleParam(medipixThreshold0, atof(fromLabviewValue));
	status = mpxGet(MPXVAR_THRESHOLD1, Labview_DEFAULT_TIMEOUT);
	if(status == asynSuccess)
		setDoubleParam(medipixThreshold1, atof(fromLabviewValue));
    status = mpxGet(MPXVAR_OPERATINGENERGY, Labview_DEFAULT_TIMEOUT);
    if(status == asynSuccess)
        setDoubleParam(medipixOperatingEnergy, atof(fromLabviewValue));

	callParamCallbacks();

	return (asynSuccess);
}

asynStatus medipixDetector::updateThresholdScanParms()
{
    asynStatus status = asynSuccess;
    char valueStr[MPX_MAXLINE];
    int thresholdScan;
    double start, stop, step;

    if(startingUp)
        return asynSuccess;

    getDoubleParam(medipixStartThresholdScan, &start);
    getDoubleParam(medipixStopThresholdScan, &stop);
    getDoubleParam(medipixStepThresholdScan, &step);
    getIntegerParam(medipixThresholdScan, &thresholdScan);


    epicsSnprintf(valueStr, MPX_MAXLINE, "%f",start);
    status = mpxSet(MPXVAR_THSTART, valueStr, Labview_DEFAULT_TIMEOUT);

    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%f", stop);
        status = mpxSet(MPXVAR_THSTOP, valueStr, Labview_DEFAULT_TIMEOUT);
    }
    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%f", step);
        status = mpxSet(MPXVAR_THSTEP, valueStr, Labview_DEFAULT_TIMEOUT);
    }
    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%d", thresholdScan);
        status = mpxSet(MPXVAR_THSSCAN, valueStr, Labview_DEFAULT_TIMEOUT);
    }

    /* Read back the actual setting, in case we are out of bounds.*/
    status = mpxGet(MPXVAR_THSTART, Labview_DEFAULT_TIMEOUT);
    if(status == asynSuccess)
        setDoubleParam(medipixStartThresholdScan, atof(fromLabviewValue));
    status = mpxGet(MPXVAR_THSTEP, Labview_DEFAULT_TIMEOUT);
    if(status == asynSuccess)
        setDoubleParam(medipixStepThresholdScan, atof(fromLabviewValue));
    status = mpxGet(MPXVAR_THSTOP, Labview_DEFAULT_TIMEOUT);
    if(status == asynSuccess)
        setDoubleParam(medipixStopThresholdScan, atof(fromLabviewValue));
    status = mpxGet(MPXVAR_THSSCAN, Labview_DEFAULT_TIMEOUT);
    if(status == asynSuccess)
        setIntegerParam(medipixThresholdScan, atoi(fromLabviewValue));


    return status;
}

static void medipixTaskC(void *drvPvt)
{
	medipixDetector *pPvt = (medipixDetector *) drvPvt;

	pPvt->medipixTask();
}

// returns true if the header is a data header
// parses the data header and adds appropriate attributes to pImage
medipixDataHeader medipixDetector::parseDataHeader(const char* header)
{
    char buff[MPX_MSG_DATATYPE_LEN];
    medipixDataHeader headerType = MPXUnknownHeader;

    strncpy(buff, header, MPX_MSG_DATATYPE_LEN);

    if(!strncmp(buff, MPX_DATA_12,MPX_MSG_DATATYPE_LEN))
            headerType = MPXDataHeader12;
    else if(!strncmp(buff, MPX_DATA_24,MPX_MSG_DATATYPE_LEN))
            headerType =  MPXDataHeader24;
    else if(!strncmp(buff, MPX_DATA_ACQ_HDR,MPX_MSG_DATATYPE_LEN))
        headerType =  MPXAcquisitionHeader;

    return headerType;
}

// returns true if the header is a data header
// parses the data header and adds appropriate attributes to pImage
void  medipixDetector::parseDataFrame(NDArray* pImage, const char* header)
{
    char buff[MPX_IMG_HDR_LEN+1];
    unsigned long lVal;
    double dVal;
    int iVal, dacNum;
    char dacName[10];
    char* tok;

    // make a copy since strtok is destructive
    strncpy(buff, header, MPX_IMG_HDR_LEN);
    buff[MPX_IMG_HDR_LEN+1] = 0;

    #ifdef DEBUG
        printf("Image frame Header: %s\n\n", buff);
    #endif

    tok = strtok(buff,",");
    tok = strtok(NULL,",");  // skip the (HDR already parsed)
    if(tok != NULL)
    {
        iVal = atoi(tok);
        pImage->pAttributeList->add("Frame Number","", NDAttrInt32, &iVal);
    }
    tok = strtok(NULL, ",");
    if(tok != NULL)
    {
        iVal = atoi(tok);
        pImage->pAttributeList->add("Counter Number","", NDAttrInt32, &iVal);
    }
    tok = strtok(NULL, ",");
    if(tok != NULL)
    {
        time_t rawtime;
        unsigned long msecs;


        // Covert string representation to EPICS Time and store in attributes as
        // a unsigned long
        // format is 2012-02-01 11:26:00.000

/*
 * NOTE it has been decided that this driver will provide a timestamp and will ignore the value
 * passed from medipix - this is because the FPGA does not have access to a clock while processing
 * and hence all frames in a given acquisition are reported as starting at the same microsecond
 *
        msecsstr = strptime(tok, "%Y-%m-%d %H:%M:%S.", &t);
        if(msecsstr != NULL)
            msecs = atol(msecsstr);
*/

        rawtime = time (NULL);
        lVal = (unsigned long) rawtime;
        msecs = 0;

        pImage->pAttributeList->add("Start Time UTC seconds","", NDAttrUInt32, &lVal);
        pImage->pAttributeList->add("Start Time millisecs","", NDAttrUInt32, &msecs);
    }
    tok = strtok(NULL, ",");
    if(tok != NULL)
    {
        dVal = atof(tok);
        pImage->pAttributeList->add("Duration","", NDAttrFloat64, &dVal);
    }
    tok = strtok(NULL, ",");
    if(tok != NULL)
    {
        dVal = atof(tok);
        pImage->pAttributeList->add("Threshold 0","", NDAttrFloat64, &dVal);
    }
    tok = strtok(NULL, ",");
    if(tok != NULL)
    {
        dVal = atof(tok);
        pImage->pAttributeList->add("Threshold 1","", NDAttrFloat64, &dVal);
    }
    for(dacNum = 1; dacNum <=100; dacNum++ && tok != NULL)
    {
        tok = strtok(NULL, ",");
        if(tok != NULL)
        {
            iVal = atoi(tok);
            sprintf(dacName,"DAC %03d", dacNum);
            pImage->pAttributeList->add(dacName,"", NDAttrInt32, &iVal);
        }
    }
}

/** helper functions for endien conversion
 *
 */
inline void endian_swap(unsigned short& x)
{
    x = (x>>8) |
        (x<<8);
}

inline void endian_swap(unsigned int& x)
{
    x = (x>>24) |
        ((x<<8) & 0x00FF0000) |
        ((x>>8) & 0x0000FF00) |
        (x<<24);
}
/** This thread controls acquisition, reads image files to get the image data, and
 * does the callbacks to send it to higher layers
 * It is totally decoupled from the command thread and simply waits for data
 * frames to be sent on the data channel (TCP) regardless of the state in the command
 * thread and TCP channel */
void medipixDetector::medipixTask()
{
	int status = asynSuccess;
	int imageCounter;
	int counterDepth;
	NDArray *pImage;
	epicsTimeStamp startTime;
	const char *functionName = "medipixTask";
	int dims[2];
	int arrayCallbacks;
	int nread;
	char *bigBuff;
	char aquisitionHeader[MPX_ACQUISITION_HEADER_LEN+1];

	this->lock();

	// allocate a buffer for reading in images from labview over network
	bigBuff = (char*) calloc(MPX_IMG_FRAME_LEN24, 1);


	/* Loop forever */
	while (1)
	{
        // Get the current time - note this is the time we start listening and may be up
	    // to 10 seconds earlier than frame received - oh well !
        epicsTimeGetCurrent(&startTime);

        // Acquire an image from the data channel
        memset(bigBuff, 0, MPX_IMG_FRAME_LEN);

        /* We release the mutex when waiting because this takes a long time and
         * we need to allow abort operations to get through */
        this->unlock();

        // wait for the next data frame packet - this function spends most of its time here
        status = mpxRead(this->pasynLabViewData, bigBuff, MPX_IMG_FRAME_LEN24, &nread, 10);
        this->lock();

        /* If there was an error jump to bottom of loop */
        if (status)
        {
            if (status == asynTimeout)
                status = asynSuccess;   // timeouts are expected
            else
            {
                asynPrint(this->pasynLabViewData, ASYN_TRACE_ERROR,
                        "%s:%s: error in Labview data channel response, status=%d\n", driverName,
                        functionName, status);
                setStringParam(ADStatusMessage,
                        "Error in Labview data channel response");
                // wait before trying again - otherwise socket error creates a tight loop
                epicsThreadSleep(.5);
            }
            continue;
        }

        // if we get here we have successfully received a data frame
        #ifdef DEBUG
            printf("\n\nReceived image frame of %d bytes\n", nread);
        #endif

        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

        if (arrayCallbacks)
        {
        	getIntegerParam(medipixCounterDepth, &counterDepth);
            getIntegerParam(NDArrayCounter, &imageCounter);
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            /* Call the callbacks to update any changes */
            callParamCallbacks();

            medipixDataHeader header = parseDataHeader(bigBuff);

            /* Get an image buffer from the pool */
            getIntegerParam(ADMaxSizeX, &dims[0]);
            getIntegerParam(ADMaxSizeY, &dims[1]);
            if(header == MPXDataHeader12)
            {
                #ifdef DEBUG
                    printf("12bit Array\n");
                #endif

                pImage = this->pNDArrayPool->alloc(2, dims, NDUInt16, 0, NULL);

                // copy the data into NDArray, switching to little endien and Inverting in the Y axis
                epicsUInt16 *pData, *pSrc;
                int x,y;
                for ( y = 0; y < dims[1]; y++)
                {
                    for (   x = 0,
                            pData = (epicsUInt16 *) pImage->pData + y * dims[0],
                            pSrc = (epicsUInt16 *) (bigBuff + MPX_IMG_HDR_LEN) + (dims[1] - y) * dims[0];
                            x < dims[0];
                            x++, pData++, pSrc++)
                    {
                        *pData = *pSrc;
                        endian_swap(*pData);
                    }
                }
            }
            else if(header == MPXDataHeader24)
            {
                #ifdef DEBUG
                    printf("24bit Array\n");
                #endif

				pImage = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);

				epicsUInt32 *pData, *pSrc;
                int x,y;
                for ( y = 0; y < dims[1]; y++)
                {
                    for (   x = 0,
                            pData = (epicsUInt32 *) pImage->pData + y * dims[0],
                            pSrc = (epicsUInt32 *) (bigBuff + MPX_IMG_HDR_LEN) + (dims[1] - y) * dims[0];
                            x < dims[0];
                            x++, pData++, pSrc++)
                    {
                        *pData = *pSrc;
                        endian_swap(*pData);
                    }
                }
            }

            if(header == MPXAcquisitionHeader)
            {
                // this is an acquisition header
                strncpy(aquisitionHeader, bigBuff, MPX_ACQUISITION_HEADER_LEN);
                aquisitionHeader[MPX_ACQUISITION_HEADER_LEN] = 0;

                #ifdef DEBUG
                    printf("Acquisition Header:\n%s\n\n", aquisitionHeader);
                #endif
            }
            else if(header == MPXDataHeader12 || header == MPXDataHeader24)
            {
                // parse the header and apply attributes to the NDArray
                parseDataFrame(pImage, bigBuff);

                /* Put the frame number and time stamp into the buffer */
                pImage->uniqueId = imageCounter;
                pImage->timeStamp = startTime.secPastEpoch
                        + startTime.nsec / 1.e9;

                // string attributes are global in HDF5 plugin so the most recent acquisition header is
                // applied to all files
                pImage->pAttributeList->add("Acquisition Header","", NDAttrString, aquisitionHeader);

                /* Get any attributes that have been defined for this driver */
                this->getAttributes(pImage->pAttributeList);

                /* Call the NDArray callback */
                /* Must release the lock here, or we can get into a deadlock, because we can
                 * block on the plugin lock, and the plugin can be calling us */
                this->unlock();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: calling NDArray callback\n", driverName,
                        functionName);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
                /* Free the image buffer */
                pImage->release();
            }
        }

		/* Call the callbacks to update any changes */
		callParamCallbacks();
	}
	// release the image buffer (in reality this does not get called - I need a thread shutdown signal)
	free(bigBuff);
}

static void medipixStatusC(void *drvPvt)
{
	medipixDetector *pPvt = (medipixDetector *) drvPvt;

	pPvt->medipixStatus();
}

/** This thread periodically read the detector status (temperature, humidity, etc.)
 It does not run if we are acquiring data, to avoid polling Labview when taking data.*/
void medipixDetector::medipixStatus()
{
	int result = asynSuccess;
	int status = 0;
	int statusCode;
	int idleMessage = 0;

	// let the startup script complete before attempting I/O
	epicsThreadSleep(4);
	startingUp = 0;


	// make sure important grouped variables are set to agree with
	// IOCs auto saved values
	setAcquireParams();
	updateThresholdScanParms();
	getThreshold();

    result = mpxGet(MPXVAR_GETSOFTWAREVERSION, Labview_DEFAULT_TIMEOUT);
    statusCode = atoi(this->fromLabviewValue);

    // initial status
    setIntegerParam(ADStatus, ADStatusIdle);

	while (1)
	{
        epicsThreadSleep(.5);
		this->lock();
        getIntegerParam(ADStatus, &status);

        if(status == ADStatusIdle)
        {
            if (!idleMessage)
            {
                setStringParam(ADStatusMessage, "Waiting for acquire command");
                callParamCallbacks();
                idleMessage = 1;
            }
        }
        else
        {
            result = mpxGet(MPXVAR_DETECTORSTATUS, Labview_DEFAULT_TIMEOUT);
            statusCode = atoi(this->fromLabviewValue);

            if (result == asynSuccess && this->fromLabviewError == MPX_OK)
            {
                // todo implement full set of status codes
                if(statusCode == 0)
                {
                    // detector has reported idle state
                    setIntegerParam(ADStatus, ADStatusIdle);
                    setIntegerParam(ADAcquire, 0);
                    idleMessage = 0;
                }
            }
            else
            {
                setStringParam(ADStatusMessage, "Labview communication error");
                // abort any acquire state
                setIntegerParam(ADStatus, ADStatusError);
                setIntegerParam(ADAcquire, 0);
            }
            callParamCallbacks();
        }
        unlock();
	}

}


/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters, including ADAcquire, ADTriggerMode, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus medipixDetector::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
	int function = pasynUser->reason;
	int adstatus;
	asynStatus status = asynSuccess;
	const char *functionName = "writeInt32";

	status = setIntegerParam(function, value);

    if (function == medipixReset)
    {
        mpxCommand(MPXCMD_RESET, Labview_DEFAULT_TIMEOUT);

        // I cannot successfully reconnect to the server after a reset
        // the only solution found so far is to restart the ioc
        exit(0);
    }
    else if (function == ADAcquire)
    {
		getIntegerParam(ADStatus, &adstatus);
		if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError))
		{
            setIntegerParam(ADStatus, ADStatusAcquire);
            setStringParam(ADStatusMessage, "Acquiring...");
		    mpxCommand(MPXCMD_STARTACQUISITION, Labview_DEFAULT_TIMEOUT);
		    // this is required since Merlin has a (variable) delay before clearing idle status
		    // This driver relies on the detector's idle status to report its own status
		    epicsThreadSleep(.6);
		}
		if (!value && (adstatus == ADStatusAcquire))
		{
            setIntegerParam(ADStatus, ADStatusIdle);
		    mpxCommand(MPXCMD_STOPACQUISITION, Labview_DEFAULT_TIMEOUT);
		}
	}
	else if (function == medipixStartThresholdScanning)
	{
	    getIntegerParam(ADStatus, &adstatus);
	    if(value && (adstatus == ADStatusIdle || adstatus == ADStatusError))
	    {
	        setIntegerParam(ADStatus, ADStatusAcquire);
            setStringParam(ADStatusMessage, "Performing Threshold Scan...");
	        status = mpxCommand(MPXCMD_THSCAN, Labview_DEFAULT_TIMEOUT);
	    }
	    else if ((adstatus == ADStatusAcquire))
	    {
	        // abort a threshold scan
            setIntegerParam(ADStatus, ADStatusIdle);
            mpxCommand(MPXCMD_STOPACQUISITION, Labview_DEFAULT_TIMEOUT);
	    }

	}
	else if ((function == ADTriggerMode) || (function == ADNumImages)
			|| (function == ADNumExposures) || (function == medipixCounterDepth))
	{
		setAcquireParams();
	}
	else if (function == medipixThresholdApply)
	{
	    // TODO what is the relevance of this command to medipix?
		getThreshold();
	}
	else
	{
		// function numbers are assigned sequentially via createParam in the constructor and hence
		// any function numbers lower than our first function is handled by a (the) super class
		if (function < FIRST_medipix_PARAM
			) status = ADDriver::writeInt32(pasynUser, value);
	}

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();

	if (status)
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s: error, status=%d function=%d, value=%d\n", driverName,
				functionName, status, function, value);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: function=%d, value=%d\n", driverName, functionName,
				function, value);
	return status;
}

/** Called when asyn clients call pasynFloat64->write().
 * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus medipixDetector::writeFloat64(asynUser *pasynUser,
		epicsFloat64 value)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *functionName = "writeFloat64";
	char value_str[MPX_MAXLINE];
	double oldValue;

	/* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
	 * status at the end, but that's OK */
	getDoubleParam(function, &oldValue);
	status = setDoubleParam(function, value);

	/* Changing any of the following parameters requires recomputing the base image */
    if (function == medipixThreshold0)
    {
        epicsSnprintf(value_str, MPX_MAXLINE, "%f", value);
        status = mpxSet(MPXVAR_THRESHOLD0, value_str, Labview_DEFAULT_TIMEOUT);
        getThreshold();
    }
    else if (function == medipixThreshold1)
    {
        epicsSnprintf(value_str, MPX_MAXLINE, "%f", value);
        status = mpxSet(MPXVAR_THRESHOLD1, value_str, Labview_DEFAULT_TIMEOUT);
        getThreshold();
    }
    else if (function == medipixOperatingEnergy)
    {
        epicsSnprintf(value_str, MPX_MAXLINE, "%f", value);
        status = mpxSet(MPXVAR_OPERATINGENERGY, value_str, Labview_DEFAULT_TIMEOUT);
        getThreshold();
    }
    else if ((function == ADAcquireTime) || (function == ADAcquirePeriod)
            /* || (function == medipixDelayTime) */)
    {
        setAcquireParams();
    }
	else if ((function == medipixStartThresholdScan) || (function == medipixStopThresholdScan)
	        || (function == medipixStepThresholdScan) )
	{
	    updateThresholdScanParms();
	}
	else
	{
		/* If this parameter belongs to a base class call its method */
		if (function < FIRST_medipix_PARAM
			) status = ADDriver::writeFloat64(pasynUser, value);
	}

	if (status)
	{
		/* Something went wrong so we set the old value back */
		setDoubleParam(function, oldValue);
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s error, status=%d function=%d, value=%f\n", driverName,
				functionName, status, function, value);
	}
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: function=%d, value=%f\n", driverName, functionName,
				function, value);

	/* Do callbacks so higher layers see any changes */
	callParamCallbacks();
	return status;
}

/** Called when asyn clients call pasynOctet->write().
 * This function performs actions for some parameters, including medipixBadPixelFile, ADFilePath, etc.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Address of the string to write.
 * \param[in] nChars Number of characters to write.
 * \param[out] nActual Number of characters actually written. */
asynStatus medipixDetector::writeOctet(asynUser *pasynUser, const char *value,
		size_t nChars, size_t *nActual)
{
	int function = pasynUser->reason;
	asynStatus status = asynSuccess;
	const char *functionName = "writeOctet";

	return asynSuccess;

	/* Set the parameter in the parameter library. */
	status = (asynStatus) setStringParam(function, (char *) value);

	if (function == NDFilePath)
	{
	    // not required for medipix
	}
	else
	{
		/* If this parameter belongs to a base class call its method */
		if (function < FIRST_medipix_PARAM)
		    status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
	}

	/* Do callbacks so higher layers see any changes */
	status = (asynStatus) callParamCallbacks();

	if (status)
		epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
				"%s:%s: status=%d, function=%d, value=%s", driverName,
				functionName, status, function, value);
	else
		asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
				"%s:%s: function=%d, value=%s\n", driverName, functionName,
				function, value);
	*nActual = nChars;
	return status;
}


/** Report status of the driver.
 * Prints details about the driver if details>0.
 * It then calls the ADDriver::report() method.
 * \param[in] fp File pointed passed by caller where the output is written to.
 * \param[in] details If >0 then driver details are printed.
 */
void medipixDetector::report(FILE *fp, int details)
{

	fprintf(fp, "medipix detector %s\n", this->portName);
	if (details > 0)
	{
		int nx, ny, dataType;
		getIntegerParam(ADSizeX, &nx);
		getIntegerParam(ADSizeY, &ny);
		getIntegerParam(NDDataType, &dataType);
		fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
		fprintf(fp, "  Data type:         %d\n", dataType);
	}
	/* Invoke the base class method */
	ADDriver::report(fp, details);
}

extern "C" int medipixDetectorConfig(const char *portName,
		const char *LabviewCommandPort, const char *LabviewDataPort,
		int maxSizeX, int maxSizeY, int maxBuffers, size_t maxMemory,
		int priority, int stackSize)
{
	new medipixDetector(portName, LabviewCommandPort, LabviewDataPort, maxSizeX,
			maxSizeY, maxBuffers, maxMemory, priority, stackSize);
	return (asynSuccess);
}

/** Constructor for medipix driver; most parameters are simply passed to ADDriver::ADDriver.
 * After calling the base class constructor this method creates a thread to collect the detector data,
 * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] LabviewPort The name of the asyn port previously created with drvAsynIPPortConfigure to
 *            communicate with Labview.
 * \param[in] maxSizeX The size of the medipix detector in the X direction.
 * \param[in] maxSizeY The size of the medipix detector in the Y direction.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
 * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
 *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
 * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
 */
medipixDetector::medipixDetector(const char *portName,
		const char *LabviewCommandPort, const char *LabviewDataPort,
		int maxSizeX, int maxSizeY, int maxBuffers, size_t maxMemory,
		int priority, int stackSize)

:
		ADDriver(portName, 1, NUM_medipix_PARAMS, maxBuffers, maxMemory, 0, 0, /* No interfaces beyond those set in ADDriver.cpp */
		ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
		priority, stackSize), imagesRemaining(0)

{
	int status = asynSuccess;
	const char *functionName = "medipixDetector";
	int dims[2];

	startingUp = 1;
    strcpy(LabviewCommandPortName, LabviewCommandPort);
    strcpy(LabviewDataPortName, LabviewDataPort);

	/* Allocate the raw buffer we use to read image files.  Only do this once */
	dims[0] = maxSizeX;
	dims[1] = maxSizeY;
	/* Allocate the raw buffer we use for flat fields. */
	this->pFlatField = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);

	// medipix is upside down by area detector standards
	// this does not work - I need to invert using my own memory copy function
	this->ADReverseY = 1;

	/* Connect to Labview */
	status = pasynOctetSyncIO->connect(LabviewCommandPort, 0,
			&this->pasynLabViewCmd, NULL);
	status = pasynOctetSyncIO->connect(LabviewDataPort, 0,
			&this->pasynLabViewData, NULL);

	createParam(medipixDelayTimeString, asynParamFloat64, &medipixDelayTime);
	createParam(medipixThreshold0String, asynParamFloat64, &medipixThreshold0);
	createParam(medipixThreshold1String, asynParamFloat64, &medipixThreshold1);
	createParam(medipixOperatingEnergyString, asynParamFloat64, &medipixOperatingEnergy);
	createParam(medipixThresholdApplyString, asynParamInt32, &medipixThresholdApply);
	createParam(medipixThresholdAutoApplyString, asynParamInt32, &medipixThresholdAutoApply);
	createParam(medipixArmedString, asynParamInt32, &medipixArmed);

	createParam(medipixmedpixThresholdScanString, asynParamInt32, &medipixThresholdScan);
	createParam(medipixStartThresholdScanString, asynParamFloat64, &medipixStartThresholdScan);
	createParam(medipixStopThresholdScanString, asynParamFloat64, &medipixStopThresholdScan);
	createParam(medipixStepThresholdScanString, asynParamFloat64, &medipixStepThresholdScan);
	createParam(medipixStartThresholdScanningString, asynParamInt32, &medipixStartThresholdScanning);
	createParam(medipixCounterDepthString, asynParamInt32, &medipixCounterDepth);
	createParam(medipixResetString, asynParamInt32, &medipixReset);

	/* Set some default values for parameters */
	status = setStringParam(ADManufacturer, "Medipix Consortium");
	status |= setStringParam(ADModel, "medipix");
	status |= setIntegerParam(ADMaxSizeX, maxSizeX);
	status |= setIntegerParam(ADMaxSizeY, maxSizeY);
	status |= setIntegerParam(ADSizeX, maxSizeX);
	status |= setIntegerParam(ADSizeX, maxSizeX);
	status |= setIntegerParam(ADSizeY, maxSizeY);
	status |= setIntegerParam(NDArraySizeX, maxSizeX);
	status |= setIntegerParam(NDArraySizeY, maxSizeY);
	status |= setIntegerParam(NDArraySize, 0);
	status |= setIntegerParam(NDDataType, NDUInt32);
	status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADTriggerMode, TMInternal);

    // attempt to clear the spurious error on startup (failed - not sure where this is coming from?)
    //      Medipix1TestFileName devAsynOctet: writeIt requested 0 but sent 10780660 bytes
    status |= setStringParam(NDFileName, "image.bmp");

	if (status)
	{
		printf("%s: unable to set camera parameters\n", functionName);
		return;
	}

	/* Create the thread that updates the images */
	status = (epicsThreadCreate("medipixDetTask", epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			(EPICSTHREADFUNC) medipixTaskC, this) == NULL);
	if (status)
	{
		printf("%s:%s epicsThreadCreate failure for image task\n", driverName,
				functionName);
		return;
	}

	/* Create the thread that monitors detector status (temperature, humidity, etc). */
	status = (epicsThreadCreate("medipixStatusTask", epicsThreadPriorityMedium,
			epicsThreadGetStackSize(epicsThreadStackMedium),
			(EPICSTHREADFUNC) medipixStatusC, this) == NULL);
	if (status)
	{
		printf("%s:%s epicsThreadCreate failure for status task\n", driverName,
				functionName);
		return;
	}

}

/* Code for iocsh registration */
static const iocshArg medipixDetectorConfigArg0 =
{ "Port name", iocshArgString };
static const iocshArg medipixDetectorConfigArg1 =
{ "Labview cmd port", iocshArgString };
static const iocshArg medipixDetectorConfigArg2 =
{ "Labview data port", iocshArgString };
static const iocshArg medipixDetectorConfigArg3 =
{ "maxSizeX", iocshArgInt };
static const iocshArg medipixDetectorConfigArg4 =
{ "maxSizeY", iocshArgInt };
static const iocshArg medipixDetectorConfigArg5 =
{ "maxBuffers", iocshArgInt };
static const iocshArg medipixDetectorConfigArg6 =
{ "maxMemory", iocshArgInt };
static const iocshArg medipixDetectorConfigArg7 =
{ "priority", iocshArgInt };
static const iocshArg medipixDetectorConfigArg8 =
{ "stackSize", iocshArgInt };
static const iocshArg * const medipixDetectorConfigArgs[] =
{ &medipixDetectorConfigArg0, &medipixDetectorConfigArg1,
		&medipixDetectorConfigArg2, &medipixDetectorConfigArg3,
		&medipixDetectorConfigArg4, &medipixDetectorConfigArg5,
		&medipixDetectorConfigArg6, &medipixDetectorConfigArg7,
		&medipixDetectorConfigArg8 };
static const iocshFuncDef configmedipixDetector =
{ "medipixDetectorConfig", 8, medipixDetectorConfigArgs };
static void configmedipixDetectorCallFunc(const iocshArgBuf *args)
{
	medipixDetectorConfig(args[0].sval, args[1].sval, args[2].sval,
			args[3].ival, args[4].ival, args[5].ival, args[6].ival,
			args[7].ival, args[8].ival);
}

static void medipixDetectorRegister(void)
{

	iocshRegister(&configmedipixDetector, configmedipixDetectorCallFunc);
}

extern "C"
{
epicsExportRegistrar(medipixDetectorRegister);
}

