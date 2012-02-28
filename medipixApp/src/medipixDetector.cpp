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

// TODO remove for production
#define DEBUG 1

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include <epicsTime.h>
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
#define Labview_DEFAULT_TIMEOUT 0.3
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

/** Trigger modes */
typedef enum
{
    TMInternal,
    TMExternalEnable,
    TMExternalTrigger,
    TMMultipleExternalTrigger,
    TMAlignment
} medipixTriggerMode;/** Trigger modes */

typedef enum
{
    NoAcquisition,
    AcquireImage,
    ThresholdScan
} medipixAcquisitionMode;

/** Bad pixel structure for medipix detector */
typedef struct
{
	int badIndex;
	int replaceIndex;
} badPixel;

static const char *gainStrings[] =
{ "lowG", "midG", "highG", "uhighG" };

static const char *driverName = "medipixDetector";

#define medipixDelayTimeString      "DELAY_TIME"
#define medipixThreshold0String     "THRESHOLD0"
#define medipixThreshold1String     "THRESHOLD1"
#define medipixOperatingEnergyString "OPERATINGENERGY"
#define medipixThresholdApplyString "THRESHOLD_APPLY"
#define medipixThresholdAutoApplyString "THRESHOLD_AUTO_APPLY"
#define medipixArmedString          "ARMED"
#define medipixImageFileTmotString  "IMAGE_FILE_TMOT"
#define medipixBadPixelFileString   "BAD_PIXEL_FILE"
#define medipixNumBadPixelsString   "NUM_BAD_PIXELS"
#define medipixFlatFieldFileString  "FLAT_FIELD_FILE"
#define medipixMinFlatFieldString   "MIN_FLAT_FIELD"
#define medipixFlatFieldValidString "FLAT_FIELD_VALID"
#define medipixGapFillString        "GAP_FILL"
#define medipixWavelengthString     "WAVELENGTH"
#define medipixEnergyLowString      "ENERGY_LOW"
#define medipixEnergyHighString     "ENERGY_HIGH"
#define medipixDetDistString        "DET_DIST"
#define medipixDetVOffsetString     "DET_VOFFSET"
#define medipixBeamXString          "BEAM_X"
#define medipixBeamYString          "BEAM_Y"
#define medipixFluxString           "FLUX"
#define medipixFilterTransmString   "FILTER_TRANSM"
#define medipixStartAngleString     "START_ANGLE"
#define medipixAngleIncrString      "ANGLE_INCR"
#define medipixDet2thetaString      "DET_2THETA"
#define medipixPolarizationString   "POLARIZATION"
#define medipixAlphaString          "ALPHA"
#define medipixKappaString          "KAPPA"
#define medipixPhiString            "PHI"
#define medipixChiString            "CHI"
#define medipixOscillAxisString     "OSCILL_AXIS"
#define medipixNumOscillString      "NUM_OSCILL"
#define medipixPixelCutOffString    "PIXEL_CUTOFF"
#define medipixThTemp0String        "TH_TEMP_0"
#define medipixThTemp1String        "TH_TEMP_1"
#define medipixThTemp2String        "TH_TEMP_2"
#define medipixThHumid0String       "TH_HUMID_0"
#define medipixThHumid1String       "TH_HUMID_1"
#define medipixThHumid2String       "TH_HUMID_2"
#define medipixTvxVersionString      "TVXVERSION"
#define medipixStartThresholdScanString	"THRESHOLDSTART"
#define medipixStopThresholdScanString	"THRESHOLDSTOP"
#define medipixStepThresholdScanString	"THRESHOLDSTEP"
#define medipixStartThresholdScanningString	"STARTTHRESHOLDSCANNING"

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
	int medipixImageFileTmot;
	int medipixBadPixelFile;
	int medipixNumBadPixels;
	int medipixFlatFieldFile;
	int medipixMinFlatField;
	int medipixFlatFieldValid;
	int medipixGapFill;
	int medipixWavelength;
	int medipixEnergyLow;
	int medipixEnergyHigh;
	int medipixDetDist;
	int medipixDetVOffset;
	int medipixBeamX;
	int medipixBeamY;
	int medipixFlux;
	int medipixFilterTransm;
	int medipixStartAngle;
	int medipixAngleIncr;
	int medipixDet2theta;
	int medipixPolarization;
	int medipixAlpha;
	int medipixKappa;
	int medipixPhi;
	int medipixChi;
	int medipixOscillAxis;
	int medipixNumOscill;
	int medipixPixelCutOff;
	int medipixThTemp0;
	int medipixThTemp1;
	int medipixThTemp2;
	int medipixThHumid0;
	int medipixThHumid1;
	int medipixThHumid2;
	int medipixStartThresholdScan;
	int medipixStopThresholdScan;
	int medipixStepThresholdScan;
	int medipixStartThresholdScanning;
	int medipixTvxVersion;
#define LAST_medipix_PARAM medipixTvxVersion

private:
	/* These are the methods that are new to this class */
	void abortAcquisition();
	asynStatus setAcquireParams();
	asynStatus setThreshold();
    asynStatus updateThresholdScanParms();

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
	epicsEventId startEventId;
	epicsEventId stopEventId;
	NDArray *pFlatField;
	int multipleFileNumber;
	asynUser *pasynLabViewCmd;
	asynUser *pasynLabViewData;
	double averageFlatField;
	medipixAcquisitionMode acquisitionMode;

	/* input and output from the labView controller     */
	char toLabview[MPX_MAXLINE];
	char fromLabview[MPX_MAXLINE];
	char fromLabviewHeader[MPX_MAXLINE];
	char fromLabviewBody[MPX_MAXLINE];
	char fromLabviewValue[MPX_MAXLINE];
	char fromLabviewImgHdr[MPX_IMG_HDR_LEN+1];
	int fromLabviewError;
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

	pasynOctetSyncIO->flush(this->pasynLabViewCmd);
	status = pasynOctetSyncIO->write(this->pasynLabViewCmd, this->toLabview,
			strlen(this->toLabview), timeout, &nwrite);
	// make sure buffers are written out for short messages
	pasynOctetSyncIO->flush(this->pasynLabViewCmd);

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
	size_t headerSize = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + 2;
	unsigned int mpxLen = strlen(MPX_HEADER);
	int bodySize;
	int readCount = 0;
	bool leadingJunk = 0;
	unsigned int headerChar = 0;

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
			leadingJunk = 1;
			headerChar = 0;
		}
	}

	if(leadingJunk)
		asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
				"%s:%s, status=%d received %d bytes - Leading garbage discarded \n",
				driverName, functionName, status, nread,
				this->fromLabview);

	// read the rest of the header block including message length
	status = pasynOctetSyncIO->read(pasynUser, header + mpxLen,
			headerSize - mpxLen,
			timeout, &nread, &eomReason);

	nread += mpxLen;

	if (status != asynSuccess)
	{
		asynPrint(pasynUser, ASYN_TRACE_ERROR,
				"%s:%s, timeout=%f, status=%d received %d bytes\n%s\n",
				driverName, functionName, timeout, status, nread,
				this->fromLabview);
	}
	else
	{
		// terminate the response for string handling
		header[nread] = (char) NULL;
		strncpy(fromLabviewHeader, header, MPX_MAXLINE);

	#ifdef DEBUG
		printf("mpxRead: Response Header: %s\n", header);
	#endif

		if (nread != headerSize)
			return asynError;

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
		do
		{
			status = pasynOctetSyncIO->read(pasynUser, bodyBuf, bodySize - readCount,
				timeout, &nread, &eomReason);
			readCount += nread;
		} while (nread != 0 && readCount < bodySize);

		if(readCount < bodySize)
		{
			asynPrint(pasynUser, ASYN_TRACE_ERROR,
					"%s:%s, timeout=%f, status=%d received %d bytes in MPX header, expected %d\n",
					driverName, functionName, timeout, status, *bytesRead, bodySize);
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
	}

#ifdef DEBUG
	printf("mpxRead: Full Response: %s\n", fromLabview);
#endif

	fromLabviewError = MPX_OK;
	return status;
}

asynStatus medipixDetector::mpxWriteRead(char* cmdType, char* cmdName ,double timeout)
{
	asynStatus status;

	if ((status = mpxWrite(timeout)) != asynSuccess)
	{
		return status;
	}

	if ((status = mpxReadCmd(cmdType, cmdName ,timeout)) != asynSuccess)
	{
		return status;
	}

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

	status = getIntegerParam(ADTriggerMode, &triggerMode);
	if (status != asynSuccess)
		triggerMode = TMInternal;

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
	else if(dval2 - dval < 0.00085) // hardcoded value for readback time TODO parameterize
	{
		dval2 = dval + 0.00085;
		setDoubleParam(ADAcquirePeriod, dval2);
	}
	epicsSnprintf(value, MPX_MAXLINE, "%f", dval2*1000); // translated into millisec
	this->mpxSet(MPXVAR_ACQUISITIOINPERIOD, value, Labview_DEFAULT_TIMEOUT);

//	status = getDoubleParam(medipixDelayTime, &dval);
//	if ((status != asynSuccess) || (dval < 0.))
//	{
//		dval = 0.;
//		setDoubleParam(medipixDelayTime, dval);
//	}
//	epicsSnprintf(value, MPX_MAXLINE, "%d", ival);
//	this->mpxSet(MPXVAR_???, value, Labview_DEFAULT_TIMEOUT);

//		status = getIntegerParam(medipixGapFill, &ival);
//	if ((status != asynSuccess) || (ival < -2) || (ival > 0))
//	{
//		ival = -2;
//		setIntegerParam(medipixGapFill, ival);
//	}
//	/* -2 is used to indicate that GapFill is not supported because it is a single element detector */
//	if (ival != -2)
//	{
//		epicsSnprintf(this->toLabview, sizeof(this->toLabview), "gapfill %d",
//				ival);
//		writeReadLabview(Labview_DEFAULT_TIMEOUT);
//	}
//
//	/*Read back the pixel count rate cut off value.*/
//	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "Tau");
//	status = writeReadLabview(5.0);
//
//	/*Response contains the string "cutoff = 1221026 counts"*/
//	if (!status)
//	{
//		if ((substr = strstr(this->fromLabview, "cutoff")) != NULL)
//		{
//			sscanf(substr, "cutoff = %d counts", &pixelCutOff);
//			setIntegerParam(medipixPixelCutOff, pixelCutOff);
//		}
//	}

	return (asynSuccess);

}

asynStatus medipixDetector::setThreshold()
{
	int status;
	double threshold0, threshold1, energy;
	char value[MPX_MAXLINE];


	getDoubleParam(medipixThreshold0, &threshold0);
	getDoubleParam(medipixThreshold1, &threshold1);
	getDoubleParam(medipixOperatingEnergy, &energy);

	/* Set the status to waiting so we can be notified when it has finished */
	setIntegerParam(ADStatus, ADStatusWaiting);
	setStringParam(ADStatusMessage, "Setting threshold");
	callParamCallbacks();

	epicsSnprintf(value, MPX_MAXLINE, "%f", threshold0);
	status = mpxSet(MPXVAR_THRESHOLD0, value, Labview_DEFAULT_TIMEOUT);
	if(status == asynSuccess)
	{
		epicsSnprintf(value, MPX_MAXLINE, "%f", threshold1);
		status = mpxSet(MPXVAR_THRESHOLD1, value, Labview_DEFAULT_TIMEOUT);
	}
	if(status == asynSuccess)
	{
		epicsSnprintf(value, MPX_MAXLINE, "%f", energy);
		status = mpxSet(MPXVAR_OPERATINGENERGY, value, Labview_DEFAULT_TIMEOUT);
	}

	if (status)
		setIntegerParam(ADStatus, ADStatusError);
	else
		setIntegerParam(ADStatus, ADStatusIdle);

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


	// TODO is this required for medipix
	/* The SetThreshold command resets numimages to 1 and gapfill to 0, so re-send current
	 * acquisition parameters */
	//setAcquireParams();

	callParamCallbacks();

	return (asynSuccess);
}

static void medipixTaskC(void *drvPvt)
{
	medipixDetector *pPvt = (medipixDetector *) drvPvt;

	pPvt->medipixTask();
}

/** This thread controls acquisition, reads image files to get the image data, and
 * does the callbacks to send it to higher layers */
void medipixDetector::medipixTask()
{
	int status = asynSuccess;
	int imageCounter;
	int numImages;
	int multipleFileNextImage = 0; /* This is the next image number, starting at 0 */
	int acquire;
	ADStatus_t acquiring;
	//double startAngle;
	NDArray *pImage;
	double acquireTime, acquirePeriod;
	double readImageFileTimeout;
	int triggerMode;
	epicsTimeStamp startTime;
	const char *functionName = "medipixTask";
	char fullFileName[MAX_FILENAME_LEN];
	//char filePath[MAX_FILENAME_LEN];
	int dims[2];
	int arrayCallbacks;
	//int abortStatus;
	int nread;
	char *bigBuff;

	this->lock();

	// allocate a buffer for reading in images from labview over network
	bigBuff = (char*) calloc(MPX_IMG_FRAME_LEN, 1);

	/* Loop forever */
	while (1)
	{
		/* Is acquisition active? */
		getIntegerParam(ADAcquire, &acquire);
		/* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
		if (!acquire)
		{
			/* Only set the status message if we didn't encounter any errors last time, so we don't overwrite the
			 error message */
			if (!status)
				setStringParam(ADStatusMessage, "Waiting for acquire command");
			callParamCallbacks();
			/* Release the lock while we wait for an event that says acquire has started, then lock again */
			this->unlock();
			asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
					"%s:%s: waiting for acquire to start\n", driverName,
					functionName);
			status = epicsEventWait(this->startEventId);
			this->lock();
			getIntegerParam(ADAcquire, &acquire);
		}

		/* We are acquiring. */
		/* Get the current time */
		epicsTimeGetCurrent(&startTime);

		/* Get the exposure parameters */
		getDoubleParam(ADAcquireTime, &acquireTime);
		getDoubleParam(ADAcquirePeriod, &acquirePeriod);
		getDoubleParam(medipixImageFileTmot, &readImageFileTimeout);

		/* Get the acquisition parameters */
		getIntegerParam(ADTriggerMode, &triggerMode);
		getIntegerParam(ADNumImages, &numImages);

		acquiring = ADStatusAcquire;
		setIntegerParam(ADStatus, acquiring);

		setStringParam(ADStatusMessage, "Starting exposure");
		/* Send the acquire command to Labview and wait for the 15OK response */
		status = mpxCommand(MPXCMD_STARTACQUISITION, Labview_DEFAULT_TIMEOUT);

		/* If the status wasn't asynSuccess or asynTimeout, report the error */
		if (status > 1)
		{
			acquire = 0;
		}
		else
		{
			/* Set status back to asynSuccess as the timeout was expected */
			status = asynSuccess;
			/* Open the shutter */
			setShutter(1);
			/* Set the armed flag */
			setIntegerParam(medipixArmed, 1);
			multipleFileNextImage = 0;
			/* Call the callbacks to update any changes */
			setStringParam(NDFullFileName, fullFileName);
			callParamCallbacks();
		}

/*
 * TODO current implementation is not sending a header
 *
 *
		// Read the Acquisition header
		printf("reading acquisition header from data channel..\n");
		status = mpxRead(this->pasynLabViewData, bigBuff, MPX_ACQ_HDR_LEN, &nread, 5);
		bigBuff[nread] = 0;
		printf("\n\nReceived Acquistion Header:\n%s\n\n", bigBuff);
*/

		while (acquire)
		{
			setStringParam(ADStatusMessage, "Waiting for image response");
			callParamCallbacks();
			/* We release the mutex when waiting because this takes a long time and
			 * we need to allow abort operations to get through */
			this->unlock();

			// Acquire an image from the data channel
			// TODO temp test code - todo use the configurable timeout instead of 10
			// todo replace printfs with asyn logging
			printf("reading image from data channel..\n");
			fromLabviewImgHdr[0] = 0;
			memset(bigBuff, 0, MPX_IMG_FRAME_LEN);

			status = mpxRead(this->pasynLabViewData, bigBuff, MPX_IMG_FRAME_LEN, &nread, 10);
			strncpy(fromLabviewImgHdr, bigBuff, MPX_IMG_HDR_LEN);
			fromLabviewImgHdr[MPX_IMG_HDR_LEN] = 0;

			printf("\n\nReceived image frame of %d bytes\nHeader: %s\n", nread, fromLabviewImgHdr);

			// check for abort event
            if (epicsEventTryWait(this->stopEventId) == epicsEventWaitOK) {
                setStringParam(ADStatusMessage, "Acquisition aborted");
                acquire = 0;
                this->lock();
                continue;
            }

			this->lock();
			/* If there was an error jump to bottom of loop */
			if (status)
			{
				printf("error %d reading image\n",status);
				acquire = 0;
				if (status == asynTimeout)
					setStringParam(ADStatusMessage,
							"Timeout waiting for Labview response");
				else
					setStringParam(ADStatusMessage,
							"Error in Labview response");
				continue;
			}

			getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

			if (arrayCallbacks)
			{
				getIntegerParam(NDArrayCounter, &imageCounter);
				imageCounter++;
				setIntegerParam(NDArrayCounter, imageCounter);
				/* Call the callbacks to update any changes */
				callParamCallbacks();

				/* Get an image buffer from the pool */
				getIntegerParam(ADMaxSizeX, &dims[0]);
				getIntegerParam(ADMaxSizeY, &dims[1]);
				pImage = this->pNDArrayPool->alloc(2, dims, NDInt16, 0, NULL);

				epicsInt16 *pData, *pSrc;
				int i;
				for (	i = 0,
						pData = (epicsInt16 *) pImage->pData,
						pSrc = (epicsInt16 *) (bigBuff + MPX_IMG_HDR_LEN);
						i < dims[0] * dims[1];
						i++, pData++, pSrc++)
				{
					*pData = *pSrc;
				}

				/* Put the frame number and time stamp into the buffer */
				pImage->uniqueId = imageCounter;
				pImage->timeStamp = startTime.secPastEpoch
						+ startTime.nsec / 1.e9;
				pImage->pAttributeList->add("Data Frame Header","Image acquisition header", NDAttrString, fromLabviewImgHdr);

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

			if (numImages == 1)
			{
				acquire = 0;
			}
			else if (numImages > 1)
			{
				multipleFileNextImage++;
				multipleFileNumber++;
				if (multipleFileNextImage == numImages)
					acquire = 0;
			}

		}
		/* We are done acquiring */
		setShutter(0);
		setIntegerParam(ADAcquire, 0);
		setIntegerParam(medipixArmed, 0);

		/* If everything was ok, set the status back to idle */
		if (!status)
			setIntegerParam(ADStatus, ADStatusIdle);
		else
			setIntegerParam(ADStatus, ADStatusError);

		/* Call the callbacks to update any changes */
		callParamCallbacks();
	}
	// release the image buffer
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
	int acquire = 0;
	int statusCode;

	// let the startup script complete before attempting I/O
	epicsThreadSleep(4);

	while (1)
	{
		lock();
		/* Is acquisition active? */
		getIntegerParam(ADAcquire, &acquire);

		result = mpxGet(MPXVAR_DETECTORSTATUS, Labview_DEFAULT_TIMEOUT);
		statusCode = atoi(this->fromLabviewValue);

		// TODO need to wire up status code to somthing useful!
		// TODO review things that need monitoring in this function
		result = mpxGet(MPXVAR_GETSOFTWAREVERSION, Labview_DEFAULT_TIMEOUT);
		statusCode = atoi(this->fromLabviewValue);


		/* Response should contain: 1 = busy, 0 = idle */

		if (result == asynSuccess && this->fromLabviewError == MPX_OK)
		{
			callParamCallbacks();
			unlock();
		}
		else
		{
			setStringParam(ADStatusMessage, "Labview communication error");
			/*Unlock right away and try again next time*/
			unlock();
		}

		/* This thread does not need to run often - wait for 60 seconds and perform status
		 * read again
		 */
		epicsThreadSleep(60);
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

	if (function == ADAcquire)
	{
		getIntegerParam(ADStatus, &adstatus);
		if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError))
		{
			/* Send an event to wake up the medipix task.  */
		    acquisitionMode = AcquireImage;
			epicsEventSignal(this->startEventId);
		}
		if (!value && (adstatus == ADStatusAcquire) && (acquisitionMode == AcquireImage))
		{
			/* This was a command to stop acquisition */
		    mpxCommand(MPXCMD_STOPACQUISITION, Labview_DEFAULT_TIMEOUT);
            epicsEventSignal(this->stopEventId);
            acquisitionMode = NoAcquisition;
		}
	}
	else if (function == medipixStartThresholdScanning)
	{
	    getIntegerParam(ADStatus, &adstatus);
	    if(value && (adstatus == ADStatusIdle || adstatus == ADStatusError))
	    {
	        // start a threshold scan by signalling the status task
            acquisitionMode = ThresholdScan;
            updateThresholdScanParms();
            mpxCommand(MPXCMD_THSTART, Labview_DEFAULT_TIMEOUT);
	    }
	    else if ((adstatus == ADStatusAcquire) && (acquisitionMode == ThresholdScan))
	    {
	        // abort a threshold scan
            mpxCommand(MPXCMD_THSTOP, Labview_DEFAULT_TIMEOUT);
            acquisitionMode = NoAcquisition;
	    }

	}
	else if ((function == ADTriggerMode) || (function == ADNumImages)
			|| (function == ADNumExposures) || (function == medipixGapFill))
	{
		setAcquireParams();
	}
	else if (function == medipixThresholdApply)
	{
		setThreshold();
	}
	else
	{
		// TODO this looks like it does not work to me !! FIRST_medipix_PARAM is the value in medipixDelayTime
		/* If this parameter belongs to a base class call its method */
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

asynStatus medipixDetector::updateThresholdScanParms()
{
    double dval;
    asynStatus status = asynSuccess;
    char valueStr[MPX_MAXLINE];

    status = getDoubleParam(medipixStartThresholdScan, &dval);
    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%f",dval);
        status = mpxSet(MPXVAR_THSTART, valueStr, Labview_DEFAULT_TIMEOUT);
    }

    status = getDoubleParam(medipixStopThresholdScan, &dval);
    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%f", dval);
        status = mpxSet(MPXVAR_THSTOP, valueStr, Labview_DEFAULT_TIMEOUT);
    }

    status = getDoubleParam(medipixStepThresholdScan, &dval);
    if(status == asynSuccess)
    {
        epicsSnprintf(valueStr, MPX_MAXLINE, "%f", dval);
        status = mpxSet(MPXVAR_THSTEP, valueStr, Labview_DEFAULT_TIMEOUT);
    }

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
	double energyLow, energyHigh;
	const char *functionName = "writeFloat64";
	double oldValue;
	char valueStr[MPX_MAXLINE];

	/* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
	 * status at the end, but that's OK */
	getDoubleParam(function, &oldValue);
	status = setDoubleParam(function, value);

	// TODO TODO TODO - where the following are not required - remove their member variables as well
	// as the "function ==" clause below

	/* Changing any of the following parameters requires recomputing the base image */
	if ((function == medipixThreshold0) || (function == medipixThreshold1)|| (function == medipixOperatingEnergy))
	{
		// TODO - we may want to group up other threshold settings like pilatus
		setThreshold();
	}
	else if ((function == ADAcquireTime) || (function == ADAcquirePeriod)
			|| (function == medipixDelayTime))
	{
		setAcquireParams();
	}
	else if ((function == medipixStartThresholdScan) || (function == medipixStopThresholdScan)
	            || (function == medipixStepThresholdScan))
	{
	    updateThresholdScanParms();
	}
	else if ((function == medipixEnergyLow) || (function == medipixEnergyHigh))
	{
		getDoubleParam(medipixEnergyLow, &energyLow);
		getDoubleParam(medipixEnergyHigh, &energyHigh);
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Energy_range %f,%f", energyLow, energyHigh);
		//writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}	else
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

	// Todo this function disabled temporarily
	return asynSuccess;

	/* Set the parameter in the parameter library. */
	status = (asynStatus) setStringParam(function, (char *) value);

	if (function == NDFilePath)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview), "imgpath %s",
				value);
		//writeReadLabview(Labview_DEFAULT_TIMEOUT);
		this->checkPath();
	}
	else if (function == medipixOscillAxis)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Oscillation_axis %s",
				strlen(value) == 0 ? "(nil)" : value);
		//writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else
	{
		/* If this parameter belongs to a base class call its method */
		if (function < FIRST_medipix_PARAM
			) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
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

	/* Create the epicsEvents for signaling to the medipix task when acquisition starts and stops */
	this->startEventId = epicsEventCreate(epicsEventEmpty);
	if (!this->startEventId)
	{
		printf("%s:%s epicsEventCreate failure for start event\n", driverName,
				functionName);
		return;
	}
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId)
    {
        printf("%s:%s epicsEventCreate failure for stop event\n", driverName,
                functionName);
        return;
    }

	/* Allocate the raw buffer we use to read image files.  Only do this once */
	dims[0] = maxSizeX;
	dims[1] = maxSizeY;
	/* Allocate the raw buffer we use for flat fields. */
	this->pFlatField = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);

	/* Connect to Labview */
	status = pasynOctetSyncIO->connect(LabviewCommandPort, 0,
			&this->pasynLabViewCmd, NULL);
	status = pasynOctetSyncIO->connect(LabviewDataPort, 0,
			&this->pasynLabViewData, NULL);

	createParam(medipixDelayTimeString, asynParamFloat64, &medipixDelayTime);
	createParam(medipixThreshold0String, asynParamFloat64, &medipixThreshold0);
	createParam(medipixThreshold1String, asynParamFloat64, &medipixThreshold1);
	createParam(medipixOperatingEnergyString, asynParamFloat64, &medipixOperatingEnergy);
	createParam(medipixThresholdApplyString, asynParamInt32,
			&medipixThresholdApply);
	createParam(medipixThresholdAutoApplyString, asynParamInt32,
			&medipixThresholdAutoApply);
	createParam(medipixArmedString, asynParamInt32, &medipixArmed);
	createParam(medipixImageFileTmotString, asynParamFloat64,
			&medipixImageFileTmot);
	createParam(medipixBadPixelFileString, asynParamOctet,
			&medipixBadPixelFile);
	createParam(medipixNumBadPixelsString, asynParamInt32,
			&medipixNumBadPixels);
	createParam(medipixFlatFieldFileString, asynParamOctet,
			&medipixFlatFieldFile);
	createParam(medipixMinFlatFieldString, asynParamInt32,
			&medipixMinFlatField);
	createParam(medipixFlatFieldValidString, asynParamInt32,
			&medipixFlatFieldValid);
	createParam(medipixGapFillString, asynParamInt32, &medipixGapFill);
	createParam(medipixWavelengthString, asynParamFloat64, &medipixWavelength);
	createParam(medipixEnergyLowString, asynParamFloat64, &medipixEnergyLow);
	createParam(medipixEnergyHighString, asynParamFloat64, &medipixEnergyHigh);
	createParam(medipixDetDistString, asynParamFloat64, &medipixDetDist);
	createParam(medipixDetVOffsetString, asynParamFloat64, &medipixDetVOffset);
	createParam(medipixBeamXString, asynParamFloat64, &medipixBeamX);
	createParam(medipixBeamYString, asynParamFloat64, &medipixBeamY);
	createParam(medipixFluxString, asynParamFloat64, &medipixFlux);
	createParam(medipixFilterTransmString, asynParamFloat64,
			&medipixFilterTransm);
	createParam(medipixStartAngleString, asynParamFloat64, &medipixStartAngle);
	createParam(medipixAngleIncrString, asynParamFloat64, &medipixAngleIncr);
	createParam(medipixDet2thetaString, asynParamFloat64, &medipixDet2theta);
	createParam(medipixPolarizationString, asynParamFloat64,
			&medipixPolarization);
	createParam(medipixAlphaString, asynParamFloat64, &medipixAlpha);
	createParam(medipixKappaString, asynParamFloat64, &medipixKappa);
	createParam(medipixPhiString, asynParamFloat64, &medipixPhi);
	createParam(medipixChiString, asynParamFloat64, &medipixChi);
	createParam(medipixOscillAxisString, asynParamOctet, &medipixOscillAxis);
	createParam(medipixNumOscillString, asynParamInt32, &medipixNumOscill);
	createParam(medipixPixelCutOffString, asynParamInt32, &medipixPixelCutOff);

	createParam(medipixThTemp0String, asynParamFloat64, &medipixThTemp0);
	createParam(medipixThTemp1String, asynParamFloat64, &medipixThTemp1);
	createParam(medipixThTemp2String, asynParamFloat64, &medipixThTemp2);
	createParam(medipixThHumid0String, asynParamFloat64, &medipixThHumid0);
	createParam(medipixThHumid1String, asynParamFloat64, &medipixThHumid1);
	createParam(medipixThHumid2String, asynParamFloat64, &medipixThHumid2);
	createParam(medipixTvxVersionString, asynParamOctet, &medipixTvxVersion);

	createParam(medipixStartThresholdScanString, asynParamFloat64, &medipixStartThresholdScan);
	createParam(medipixStopThresholdScanString, asynParamFloat64, &medipixStopThresholdScan);
	createParam(medipixStepThresholdScanString, asynParamFloat64, &medipixStepThresholdScan);
	createParam(medipixStartThresholdScanningString, asynParamInt32, &medipixStartThresholdScanning);

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

	status |= setIntegerParam(medipixArmed, 0);
	status |= setStringParam(medipixBadPixelFile, "");
	status |= setIntegerParam(medipixNumBadPixels, 0);
	status |= setStringParam(medipixFlatFieldFile, "");
	status |= setIntegerParam(medipixFlatFieldValid, 0);

	setDoubleParam(medipixThTemp0, 0);
	setDoubleParam(medipixThTemp1, 0);
	setDoubleParam(medipixThTemp2, 0);
	setDoubleParam(medipixThHumid0, 0);
	setDoubleParam(medipixThHumid1, 0);
	setDoubleParam(medipixThHumid2, 0);

	if (status)
	{
		printf("%s: unable to set camera parameters\n", functionName);
		return;
	}

	/* TODO do we need any startup coms with labview???

	 // Read the TVX Version at startup.
	 epicsSnprintf(this->toLabview, sizeof(this->toLabview), "version");
	 status=writeReadLabview(1.0);
	 if (!status) {
	 if ((substr = strstr(this->fromLabview, "tvx")) != NULL) {
	 setStringParam(medipixTvxVersion, substr);
	 }
	 }
	 */

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

