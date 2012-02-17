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

#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cbf_ad.h>
#include <tiffio.h>

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
} medipixTriggerMode;

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
#define medipixThresholdString      "THRESHOLD"
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
	int medipixThreshold;
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
	int medipixTvxVersion;
#define LAST_medipix_PARAM medipixTvxVersion

private:
	/* These are the methods that are new to this class */
	void abortAcquisition();
	int stringEndsWith(const char *aString, const char *aSubstring,
			int shouldIgnoreCase);
	asynStatus writeReadLabview(double timeout);
	asynStatus setAcquireParams();
	asynStatus setThreshold();
	void makeMultipleFileFormat(const char *baseFileName);

	/* The labview communication primitives */
	asynStatus mpxGet(char* valueId, double timeout);
	asynStatus mpxSet(char* valueId, char* value, double timeout);
	asynStatus mpxCommand(char* commandId, double timeout);
	asynStatus mpxWrite(double timeout);
	asynStatus mpxReadCmd(double timeout);
	asynStatus mpxWriteRead(double timeout);
	asynStatus mpxRead(asynUser* pasynUser, char* bodyBuf, int bufSize, int* bytesRead, double timeout);

	/* Our data */
	int imagesRemaining;
	epicsEventId startEventId;
	epicsEventId stopEventId;
	NDArray *pFlatField;
	char multipleFileFormat[MAX_FILENAME_LEN];
	int multipleFileNumber;
	asynUser *pasynLabViewCmd;
	asynUser *pasynLabViewData;
	double averageFlatField;

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
	{
		return asynError;
	}

	// Build up command to be sent.
	// length is header + length specifier (10 decimal digits) + "SET" + length of variable name + 3 commas
	buff_len = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + strlen(MPX_CMD)
			+ strlen(commandId) + 3;
	if (buff_len > MPX_MAXLINE)
	{
		return asynError;
	}

	// the message length specifier contains the count of characters including the ',' after itself
	// i.e. total length minus the header length (including 1 comma)
	msg_len = buff_len - MPX_MSG_LEN_DIGITS - strlen(MPX_HEADER) - 1;

	sprintf(toLabview, "%s,%010u,%s,%s", MPX_HEADER, msg_len, MPX_CMD, commandId);

	if ((status = mpxWriteRead(timeout)) != asynSuccess)
	{
		return status;
	}

	// items in the response are comma delimited
	tok = strtok(fromLabviewBody, ",");
	if (tok == NULL || strncmp(MPX_CMD, tok, MPX_MAXLINE))
		return asynError;

	// 2nd item is Command Name which should be echoed back
	tok = strtok(NULL, ",");
	if (tok == NULL || strncmp(tok, commandId, MPX_MAXLINE))
		return asynError;

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

	if ((status = mpxWriteRead(timeout)) != asynSuccess)
	{
		return status;
	}

	// items in the response are comma delimited
	tok = strtok(fromLabviewBody, ",");
	if (tok == NULL || strncmp(MPX_GET, tok, MPX_MAXLINE))
		return asynError;

	// 2nd item is Value Name which should be echoed back
	tok = strtok(NULL, ",");
	if (tok == NULL || strncmp(tok, valueId, MPX_MAXLINE))
		return asynError;

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
 * It aligns to the header
 * MPX,0000000000,
 * Where the numeric portion represents the no. of bytes in body of the
 * frame in decimal (inclusive of the header terminating comma).
 *
 * Reads the body into the passed bodyBuf
 */
asynStatus medipixDetector::mpxRead(asynUser* pasynUser, char* bodyBuf, int bufSize, int* bytesRead, double timeout)
{
	size_t nread = 0;
	asynStatus status = asynSuccess;
	int eomReason;
	const char *functionName = "mpxRead";
	size_t headerSize = strlen(MPX_HEADER) + MPX_MSG_LEN_DIGITS + 2;
	int bodySize;
	int readCount = 0;

	char* tok;
	char header[MPX_MAXLINE];

	// TODO TODO - need to throw away any leading data before "MPX,"
	// in order to realign if we get out of sync with the server

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;

	// first read the header block and message length
	status = pasynOctetSyncIO->read(pasynUser, header, headerSize,
			timeout, &nread, &eomReason);

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
		tok = strtok(header, ",");
		if (tok == NULL || strncmp(MPX_HEADER, tok, MPX_MAXLINE))
			return asynError;

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
asynStatus medipixDetector::mpxReadCmd(double timeout)
{
	int nread = 0;
	asynStatus status = asynSuccess;
	char buff[MPX_MAXLINE];

	// default to this error for any following parsing issues
	fromLabviewError = MPX_ERR_UNEXPECTED;

	status = mpxRead(this->pasynLabViewCmd, buff, MPX_MAXLINE, &nread, timeout);

	// terminate the response for string handling
	buff[nread] = (char) NULL;
	// update the member variables with relevant parts of the response
	strncpy(fromLabview, fromLabviewHeader, MPX_MAXLINE);
	strncpy(fromLabviewBody, buff, MPX_MAXLINE);
	strncat(fromLabview, fromLabviewBody, MPX_MAXLINE);

	// Set output string so it can get back to EPICS
	setStringParam(ADStringFromServer, this->fromLabview);

#ifdef DEBUG
	printf("mpxRead: Full Response: %s\n", fromLabview);
#endif

	fromLabviewError = MPX_OK;
	return status;
}

asynStatus medipixDetector::mpxWriteRead(double timeout)
{
	asynStatus status;

	if ((status = mpxWrite(timeout)) != asynSuccess)
	{
		return status;
	}

	if ((status = mpxReadCmd(timeout)) != asynSuccess)
	{
		return status;
	}

	return asynSuccess;
}

// #######################################################################################
// ##################### END OF Labview communications primitives ########################
// #######################################################################################

void medipixDetector::makeMultipleFileFormat(const char *baseFileName)
{
	/* This function uses the code from Labview */
	char *p, *q;
	int fmt;
	char mfTempFormat[MAX_FILENAME_LEN];
	char mfExtension[10];
	int numImages;

	/* baseFilename has been built by the caller.
	 * Copy to temp */
	strncpy(mfTempFormat, baseFileName, sizeof(mfTempFormat));
	getIntegerParam(ADNumImages, &numImages);
	p = mfTempFormat + strlen(mfTempFormat) - 5; /* look for extension */
	if ((q = strrchr(p, '.')))
	{
		strcpy(mfExtension, q);
		*q = '\0';
	}
	else
	{
		strcpy(mfExtension, ""); /* default is raw image */
	}
	multipleFileNumber = 0; /* start number */
	fmt = 5; /* format length */
	if (!(p = strrchr(mfTempFormat, '/')))
	{
		p = mfTempFormat;
	}
	if ((q = strrchr(p, '_')))
	{
		q++;
		if (isdigit(*q) && isdigit(*(q + 1)) && isdigit(*(q+2))){
		multipleFileNumber=atoi(q);
		fmt=0;
		p=q;
		while(isdigit(*q))
		{
			fmt++;
			q++;
		}
		*p='\0';
		if (((fmt<3) || ((fmt==3) && (numImages>999))) ||
				((fmt==4) && (numImages>9999)))
		{
			fmt=5;
		}
	}
	else if (*q)
	{
		strcat(p, "_"); /* force '_' ending */
	}
}
else
{
	strcat(p, "_"); /* force '_' ending */
}
		/* Build the final format string */
	epicsSnprintf(this->multipleFileFormat, sizeof(this->multipleFileFormat),
			"%s%%.%dd%s", mfTempFormat, fmt, mfExtension);
}

int medipixDetector::stringEndsWith(const char *aString, const char *aSubstring,
		int shouldIgnoreCase)
{
	int i, j;

	i = strlen(aString) - 1;
	j = strlen(aSubstring) - 1;
	while (i >= 0 && j >= 0)
	{
		if (shouldIgnoreCase)
		{
			if (tolower(aString[i]) != tolower(aSubstring[j]))
				return 0;
		}
		else
		{
			if (aString[i] != aSubstring[j])
				return 0;
		}
		i--;
		j--;
	}
	return j < 0;
}

asynStatus medipixDetector::setAcquireParams()
{
	int ival;
	double dval;
	int triggerMode;
	asynStatus status;
	char *substr = NULL;
	int pixelCutOff = 0;

	// TODOTODO - temp disabled
	return (asynSuccess);

	status = getIntegerParam(ADTriggerMode, &triggerMode);
	if (status != asynSuccess)
		triggerMode = TMInternal;

	/* When we change modes download all exposure parameters, since some modes
	 * replace values with new parameters */
	if (triggerMode == TMAlignment)
	{
		setIntegerParam(ADNumImages, 1);
	}
	/* nexpf > 1 is only supported in External Enable mode */
	if (triggerMode != TMExternalEnable)
	{
		setIntegerParam(ADNumExposures, 1);
	}

	status = getIntegerParam(ADNumImages, &ival);
	if ((status != asynSuccess) || (ival < 1))
	{
		ival = 1;
		setIntegerParam(ADNumImages, ival);
	}
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "nimages %d", ival);
	writeReadLabview(Labview_DEFAULT_TIMEOUT);

	status = getIntegerParam(ADNumExposures, &ival);
	if ((status != asynSuccess) || (ival < 1))
	{
		ival = 1;
		setIntegerParam(ADNumExposures, ival);
	}
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "nexpframe %d",
			ival);
	writeReadLabview(Labview_DEFAULT_TIMEOUT);

	status = getDoubleParam(ADAcquireTime, &dval);
	if ((status != asynSuccess) || (dval < 0.))
	{
		dval = 1.;
		setDoubleParam(ADAcquireTime, dval);
	}
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "exptime %f", dval);
	writeReadLabview(Labview_DEFAULT_TIMEOUT);

	status = getDoubleParam(ADAcquirePeriod, &dval);
	if ((status != asynSuccess) || (dval < 0.))
	{
		dval = 2.;
		setDoubleParam(ADAcquirePeriod, dval);
	}
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "expperiod %f",
			dval);
	writeReadLabview(Labview_DEFAULT_TIMEOUT);

	status = getDoubleParam(medipixDelayTime, &dval);
	if ((status != asynSuccess) || (dval < 0.))
	{
		dval = 0.;
		setDoubleParam(medipixDelayTime, dval);
	}
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "delay %f", dval);
	writeReadLabview(Labview_DEFAULT_TIMEOUT);

	status = getIntegerParam(medipixGapFill, &ival);
	if ((status != asynSuccess) || (ival < -2) || (ival > 0))
	{
		ival = -2;
		setIntegerParam(medipixGapFill, ival);
	}
	/* -2 is used to indicate that GapFill is not supported because it is a single element detector */
	if (ival != -2)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview), "gapfill %d",
				ival);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}

	/*Read back the pixel count rate cut off value.*/
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "Tau");
	status = writeReadLabview(5.0);

	/*Response contains the string "cutoff = 1221026 counts"*/
	if (!status)
	{
		if ((substr = strstr(this->fromLabview, "cutoff")) != NULL)
		{
			sscanf(substr, "cutoff = %d counts", &pixelCutOff);
			setIntegerParam(medipixPixelCutOff, pixelCutOff);
		}
	}

	return (asynSuccess);

}

asynStatus medipixDetector::setThreshold()
{
	// TODOTODO - temp disabled
	return (asynSuccess);

	int igain, status;
	double threshold, dgain;
	char *substr = NULL;
	int threshold_readback = 0;

	getDoubleParam(ADGain, &dgain);
	igain = (int) (dgain + 0.5);
	if (igain < 0)
		igain = 0;
	if (igain > 3)
		igain = 3;
	getDoubleParam(medipixThreshold, &threshold);
	epicsSnprintf(this->toLabview, sizeof(this->toLabview),
			"SetThreshold %s %f", gainStrings[igain], threshold * 1000.);
	/* Set the status to waiting so we can be notified when it has finished */
	setIntegerParam(ADStatus, ADStatusWaiting);
	setStringParam(ADStatusMessage, "Setting threshold");
	callParamCallbacks();

	status = writeReadLabview(90.0); /* This command can take 78 seconds on a 6M */
	if (status)
		setIntegerParam(ADStatus, ADStatusError);
	else
		setIntegerParam(ADStatus, ADStatusIdle);
	setIntegerParam(medipixThresholdApply, 0);

	/* Read back the actual setting, in case we are out of bounds.*/
	epicsSnprintf(this->toLabview, sizeof(this->toLabview), "SetThreshold");
	status = writeReadLabview(5.0);

	/* Response should contain "threshold: 9000 eV; vcmp:"*/
	if (!status)
	{
		if ((substr = strstr(this->fromLabview, "threshold: ")) != NULL)
		{
			sscanf(strtok(substr, ";"), "threshold: %d eV",
					&threshold_readback);
			setDoubleParam(medipixThreshold,
					(double) threshold_readback / 1000.0);
		}
	}

	/* The SetThreshold command resets numimages to 1 and gapfill to 0, so re-send current
	 * acquisition parameters */
	setAcquireParams();

	callParamCallbacks();

	return (asynSuccess);
}

asynStatus medipixDetector::writeReadLabview(double timeout)
{
	asynStatus status = asynSuccess;

	// dummy function while refactoring pilatus -> medipix

	return status;
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
	double readImageFileTimeout, timeout;
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
	// bigBuff = (char*) calloc(MPX_IMG_HDR_LEN + MPX_IMAGE_BYTES + 10, 1);
	bigBuff = (char*) calloc(131329, 100);

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

		/* Reset the MX settings start angle */
		/*
		 * TODO - do we need this for medipix?
		 *
		getDoubleParam(medipixStartAngle, &startAngle);
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Start_angle %f", startAngle);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
*/

		/* Create the full filename */
		createFileName(sizeof(fullFileName), fullFileName);

		// TODO - need to set up aquire params here
		/*
		switch (triggerMode)
		{
		case TMInternal:
			epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					"Exposure %s", fullFileName);
			break;
		case TMExternalEnable:
			epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					"ExtEnable %s", fullFileName);
			break;
		case TMExternalTrigger:
			epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					"ExtTrigger %s", fullFileName);
			break;
		case TMMultipleExternalTrigger:
			epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					"ExtMTrigger %s", fullFileName);
			break;
		case TMAlignment:
			getStringParam(NDFilePath, sizeof(filePath), filePath);
			epicsSnprintf(fullFileName, sizeof(fullFileName), "%salignment.tif",
					filePath);
			epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					"Exposure %s", fullFileName);
			break;
		}
		*/

		setStringParam(ADStatusMessage, "Starting exposure");
		/* Send the acquire command to Labview and wait for the 15OK response */
		status = mpxCommand(MPX_STARTACQUISITION, Labview_DEFAULT_TIMEOUT);

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
			/* Create the format string for constructing file names for multi-image collection */
			makeMultipleFileFormat(fullFileName);
			multipleFileNextImage = 0;
			/* Call the callbacks to update any changes */
			setStringParam(NDFullFileName, fullFileName);
			callParamCallbacks();
		}

/*
 * current implementation is not sending a header
 *
 *
		// Read the Acquisition header
		// TODO temp test code
		printf("reading acquisition header from data channel..\n");
		status = mpxRead(this->pasynLabViewData, bigBuff, MPX_ACQ_HDR_LEN, &nread, 5);
		bigBuff[nread] = 0;
		printf("\n\nReceived Acquistion Header:\n%s\n\n", bigBuff);
*/

		while (acquire)
		{
			if (numImages == 1)
			{
				/* For single frame or alignment mode need to wait for 7OK response from Labview
				 * saying acquisition is complete before trying to read file, else we get a
				 * recent but stale file. */
				setStringParam(ADStatusMessage, "Waiting for single image response");
				callParamCallbacks();
				/* We release the mutex when waiting because this takes a long time and
				 * we need to allow abort operations to get through */
				this->unlock();

				// Acquire an image from the data channel
				// TODO temp test code - todo use the configurable timeout instead of 10
				// todo replace printfs with asyn logging
				printf("reading image from data channel..\n");
				// TODO + 10 is because response is somtimes a little longer than expected (often see MPX,0000131329,)
				// TODO Discuss above with detector team
				status = mpxRead(this->pasynLabViewData, bigBuff, MPX_IMG_HDR_LEN + MPX_IMAGE_BYTES + 10, &nread, 10);
				strncpy(fromLabviewImgHdr, bigBuff, MPX_IMG_HDR_LEN);
				fromLabviewImgHdr[MPX_IMG_HDR_LEN] = 0;

				printf("\n\nReceived image frame of %d bytes\nHeader: %s\n", nread, fromLabviewImgHdr);

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
			}
			else
			{
				// TODO GK - how do we handle multiple images?
				// in theory do not need to write these, just treat them the same and let the array
				// callbacks handle them

				/* If this is a multi-file acquisition the file name is built differently */
				epicsSnprintf(fullFileName, sizeof(fullFileName),
						multipleFileFormat, multipleFileNumber);
				setStringParam(NDFullFileName, fullFileName);
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
				if (triggerMode == TMAlignment)
				{
					// TODO GK do we have a equivalent?
					//epicsSnprintf(this->toLabview, sizeof(this->toLabview),
					//		"Exposure %s", fullFileName);
					/* Send the acquire command to Labview and wait for the 15OK response */
					//writeReadLabview(2.0);
				}
				else
				{
					acquire = 0;
				}
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
		/* Wait for the 7OK response from Labview in the case of multiple images */
		if ((numImages > 1) && (status == asynSuccess))
		{
			/* If arrayCallbacks is 0 we will have gone through the above loop without waiting
			 * for each image file to be written.  Thus, we may need to wait a long time for
			 * the 7OK response.
			 * If arrayCallbacks is 1 then the response should arrive fairly soon. */
			if (arrayCallbacks)
				timeout = readImageFileTimeout;
			else
				timeout = numImages * acquireTime + readImageFileTimeout;
			setStringParam(ADStatusMessage, "Waiting for 7OK response");
			callParamCallbacks();
			/* We release the mutex because we may wait a long time and need to allow abort
			 * operations to get through */
			this->unlock();
			// TODO GK readLabview(timeout);
			this->lock();
		}
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

		result = mpxGet(MPX_DETECTORSTATUS, Labview_DEFAULT_TIMEOUT);
		statusCode = atoi(this->fromLabviewValue);

		result = mpxGet(MPX_GETSOFTWAREVERSION, Labview_DEFAULT_TIMEOUT);
		statusCode = atoi(this->fromLabviewValue);
		/* Response should contain: 1 = busy, 0 = idle */

		// NOTE This  will overwrite useful messages set elsewhere
		// so only do update if the error state is clear
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

		/*This thread does not need to run often.*/
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
			epicsEventSignal(this->startEventId);
		}
		if (!value && (adstatus == ADStatusAcquire))
		{
			/* This was a command to stop acquisition */
			// TODO
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
	else if (function == medipixNumOscill)
	{
		// TODO - does medipix have equivalent to this?
		// epicsSnprintf(this->toLabview, sizeof(this->toLabview),
		//		"mxsettings N_oscillations %d", value);
		// writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else
	{
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
	double beamX, beamY;
	int thresholdAutoApply;
	const char *functionName = "writeFloat64";
	double oldValue;

	// Todo this function disabled temporarily
	return asynSuccess;

	/* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
	 * status at the end, but that's OK */
	getDoubleParam(function, &oldValue);
	status = setDoubleParam(function, value);

	/* Changing any of the following parameters requires recomputing the base image */
	if ((function == ADGain) || (function == medipixThreshold))
	{
		getIntegerParam(medipixThresholdAutoApply, &thresholdAutoApply);
		if (thresholdAutoApply)
			setThreshold();
	}
	else if ((function == ADAcquireTime) || (function == ADAcquirePeriod)
			|| (function == medipixDelayTime))
	{
		setAcquireParams();
	}
	else if (function == medipixWavelength)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Wavelength %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if ((function == medipixEnergyLow) || (function == medipixEnergyHigh))
	{
		getDoubleParam(medipixEnergyLow, &energyLow);
		getDoubleParam(medipixEnergyHigh, &energyHigh);
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Energy_range %f,%f", energyLow, energyHigh);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixDetDist)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Detector_distance %f", value / 1000.0);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixDetVOffset)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Detector_Voffset %f", value / 1000.0);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if ((function == medipixBeamX) || (function == medipixBeamY))
	{
		getDoubleParam(medipixBeamX, &beamX);
		getDoubleParam(medipixBeamY, &beamY);
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Beam_xy %f,%f", beamX, beamY);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixFlux)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Flux %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixFilterTransm)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Filter_transmission %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixStartAngle)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Start_angle %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixAngleIncr)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Angle_increment %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixDet2theta)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Detector_2theta %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixPolarization)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Polarization %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixAlpha)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Alpha %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixKappa)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Kappa %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixPhi)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Phi %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
	}
	else if (function == medipixChi)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Chi %f", value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
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

	// Todo this function disabled temporarily
	return asynSuccess;

	/* Set the parameter in the parameter library. */
	status = (asynStatus) setStringParam(function, (char *) value);

	if (function == NDFilePath)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview), "imgpath %s",
				value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
		this->checkPath();
	}
	else if (function == medipixOscillAxis)
	{
		epicsSnprintf(this->toLabview, sizeof(this->toLabview),
				"mxsettings Oscillation_axis %s",
				strlen(value) == 0 ? "(nil)" : value);
		writeReadLabview(Labview_DEFAULT_TIMEOUT);
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
	createParam(medipixThresholdString, asynParamFloat64, &medipixThreshold);
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

