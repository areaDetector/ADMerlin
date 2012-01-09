/* pilatusDetector.cpp
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
 */
 
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

/** Messages to/from camserver */
#define MAX_MESSAGE_SIZE 256 
#define MAX_FILENAME_LEN 256
#define MAX_BAD_PIXELS 100
/** Time to poll when reading from camserver */
#define ASYN_POLL_TIME .01 
#define CAMSERVER_DEFAULT_TIMEOUT 1.0 
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

/** Trigger modes */
typedef enum {
    TMInternal,
    TMExternalEnable,
    TMExternalTrigger,
    TMMultipleExternalTrigger,
    TMAlignment
} PilatusTriggerMode;

/** Bad pixel structure for Pilatus detector */
typedef struct {
    int badIndex;
    int replaceIndex;
} badPixel;


static const char *gainStrings[] = {"lowG", "midG", "highG", "uhighG"};

static const char *driverName = "pilatusDetector";

#define PilatusDelayTimeString      "DELAY_TIME"
#define PilatusThresholdString      "THRESHOLD"
#define PilatusThresholdApplyString "THRESHOLD_APPLY"
#define PilatusThresholdAutoApplyString "THRESHOLD_AUTO_APPLY"
#define PilatusArmedString          "ARMED"
#define PilatusImageFileTmotString  "IMAGE_FILE_TMOT"
#define PilatusBadPixelFileString   "BAD_PIXEL_FILE"
#define PilatusNumBadPixelsString   "NUM_BAD_PIXELS"
#define PilatusFlatFieldFileString  "FLAT_FIELD_FILE"
#define PilatusMinFlatFieldString   "MIN_FLAT_FIELD"
#define PilatusFlatFieldValidString "FLAT_FIELD_VALID"
#define PilatusGapFillString        "GAP_FILL"
#define PilatusWavelengthString     "WAVELENGTH"
#define PilatusEnergyLowString      "ENERGY_LOW"
#define PilatusEnergyHighString     "ENERGY_HIGH"
#define PilatusDetDistString        "DET_DIST"
#define PilatusDetVOffsetString     "DET_VOFFSET"
#define PilatusBeamXString          "BEAM_X"
#define PilatusBeamYString          "BEAM_Y"
#define PilatusFluxString           "FLUX"
#define PilatusFilterTransmString   "FILTER_TRANSM"
#define PilatusStartAngleString     "START_ANGLE"
#define PilatusAngleIncrString      "ANGLE_INCR"
#define PilatusDet2thetaString      "DET_2THETA"
#define PilatusPolarizationString   "POLARIZATION"
#define PilatusAlphaString          "ALPHA"
#define PilatusKappaString          "KAPPA"
#define PilatusPhiString            "PHI"
#define PilatusChiString            "CHI"
#define PilatusOscillAxisString     "OSCILL_AXIS"
#define PilatusNumOscillString      "NUM_OSCILL"
#define PilatusPixelCutOffString    "PIXEL_CUTOFF"
#define PilatusThTemp0String        "TH_TEMP_0"
#define PilatusThTemp1String        "TH_TEMP_1"
#define PilatusThTemp2String        "TH_TEMP_2"
#define PilatusThHumid0String       "TH_HUMID_0"
#define PilatusThHumid1String       "TH_HUMID_1"
#define PilatusThHumid2String       "TH_HUMID_2"
#define PilatusTvxVersionString       "TVXVERSION"

/** Driver for Dectris Pilatus pixel array detectors using their camserver server over TCP/IP socket */
class pilatusDetector : public ADDriver {
public:
    pilatusDetector(const char *portName, const char *camserverPort,
                    int maxSizeX, int maxSizeY,
                    int maxBuffers, size_t maxMemory,
                    int priority, int stackSize);
                 
    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual);
    void report(FILE *fp, int details);
    void pilatusTask(); /* This should be private but is called from C so must be public */
    void pilatusStatus(); /* This should be private but is called from C so must be public */
 
 protected:
    int PilatusDelayTime;
    #define FIRST_PILATUS_PARAM PilatusDelayTime
    int PilatusThreshold;
    int PilatusThresholdApply;
    int PilatusThresholdAutoApply;
    int PilatusArmed;
    int PilatusImageFileTmot;
    int PilatusBadPixelFile;
    int PilatusNumBadPixels;
    int PilatusFlatFieldFile;
    int PilatusMinFlatField;
    int PilatusFlatFieldValid;
    int PilatusGapFill;
    int PilatusWavelength;
    int PilatusEnergyLow;
    int PilatusEnergyHigh;
    int PilatusDetDist;
    int PilatusDetVOffset;
    int PilatusBeamX;
    int PilatusBeamY;
    int PilatusFlux;
    int PilatusFilterTransm;
    int PilatusStartAngle;
    int PilatusAngleIncr;
    int PilatusDet2theta;
    int PilatusPolarization;
    int PilatusAlpha;
    int PilatusKappa;
    int PilatusPhi;
    int PilatusChi;
    int PilatusOscillAxis;
    int PilatusNumOscill;
    int PilatusPixelCutOff;  
    int PilatusThTemp0;
    int PilatusThTemp1;
    int PilatusThTemp2;
    int PilatusThHumid0;
    int PilatusThHumid1;
    int PilatusThHumid2;
    int PilatusTvxVersion;
    #define LAST_PILATUS_PARAM PilatusTvxVersion

 private:                                       
    /* These are the methods that are new to this class */
    void abortAcquisition();
    void makeMultipleFileFormat(const char *baseFileName);
    asynStatus waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    void correctBadPixels(NDArray *pImage);
    int stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase);
    asynStatus readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus readCbf(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage);
    asynStatus writeCamserver(double timeout);
    asynStatus readCamserver(double timeout);
    asynStatus writeReadCamserver(double timeout);
    asynStatus setAcquireParams();
    asynStatus setThreshold();
    void readBadPixelFile(const char *badPixelFile);
    void readFlatFieldFile(const char *flatFieldFile);
   
    /* Our data */
    int imagesRemaining;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    char toCamserver[MAX_MESSAGE_SIZE];
    char fromCamserver[MAX_MESSAGE_SIZE];
    NDArray *pFlatField;
    char multipleFileFormat[MAX_FILENAME_LEN];
    int multipleFileNumber;
    asynUser *pasynUserCamserver;
    badPixel badPixelMap[MAX_BAD_PIXELS];
    double averageFlatField;
};

#define NUM_PILATUS_PARAMS (&LAST_PILATUS_PARAM - &FIRST_PILATUS_PARAM + 1)

void pilatusDetector::readBadPixelFile(const char *badPixelFile)
{
    int i; 
    int xbad, ybad, xgood, ygood;
    int n;
    FILE *file;
    int nx, ny;
    const char *functionName = "readBadPixelFile";
    int numBadPixels=0;

    getIntegerParam(NDArraySizeX, &nx);
    getIntegerParam(NDArraySizeY, &ny);
    setIntegerParam(PilatusNumBadPixels, numBadPixels);
    if (strlen(badPixelFile) == 0) return;
    file = fopen(badPixelFile, "r");
    if (file == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, cannot open file %s\n",
            driverName, functionName, badPixelFile);
        return;
    }
    for (i=0; i<MAX_BAD_PIXELS; i++) {
        n = fscanf(file, " %d,%d %d,%d",
                  &xbad, &ybad, &xgood, &ygood);
        if (n == EOF) break;
        if (n != 4) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, too few items =%d, should be 4\n",
                driverName, functionName, n);
            return;
        }
        this->badPixelMap[i].badIndex = ybad*nx + xbad;
        this->badPixelMap[i].replaceIndex = ygood*ny + xgood;
        numBadPixels++;
    }
    setIntegerParam(PilatusNumBadPixels, numBadPixels);
}


void pilatusDetector::readFlatFieldFile(const char *flatFieldFile)
{
    int i;
    int status;
    int ngood;
    int minFlatField;
    epicsInt32 *pData;
    const char *functionName = "readFlatFieldFile";
    NDArrayInfo arrayInfo;
    
    setIntegerParam(PilatusFlatFieldValid, 0);
    this->pFlatField->getInfo(&arrayInfo);
    getIntegerParam(PilatusMinFlatField, &minFlatField);
    if (strlen(flatFieldFile) == 0) return;
    status = readImageFile(flatFieldFile, NULL, 0., this->pFlatField);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, error reading flat field file %s\n",
            driverName, functionName, flatFieldFile);
        return;
    }
    /* Compute the average counts in the flat field */
    this->averageFlatField = 0.;
    ngood = 0;
    
    for (i=0, pData = (epicsInt32 *)this->pFlatField->pData; 
         i<arrayInfo.nElements; 
         i++, pData++) {
        if (*pData < minFlatField) continue;
        ngood++;
        averageFlatField += *pData;
    }
    averageFlatField = averageFlatField/ngood;
    
    for (i=0, pData = (epicsInt32 *)this->pFlatField->pData; 
         i<arrayInfo.nElements; 
         i++, pData++) {
        if (*pData < minFlatField) *pData = (epicsInt32)averageFlatField;
    }
    /* Call the NDArray callback */
    /* Must release the lock here, or we can get into a deadlock, because we can
     * block on the plugin lock, and the plugin can be calling us */
    this->unlock();
    doCallbacksGenericPointer(this->pFlatField, NDArrayData, 0);
    this->lock();
    setIntegerParam(PilatusFlatFieldValid, 1);
}


void pilatusDetector::makeMultipleFileFormat(const char *baseFileName)
{
    /* This function uses the code from camserver */
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
    if ( (q=strrchr(p, '.')) ) {
        strcpy(mfExtension, q);
        *q = '\0';
    } else {
        strcpy(mfExtension, ""); /* default is raw image */
    }
    multipleFileNumber=0;   /* start number */
    fmt=5;        /* format length */
    if ( !(p=strrchr(mfTempFormat, '/')) ) {
        p=mfTempFormat;
    }
    if ( (q=strrchr(p, '_')) ) {
        q++;
        if (isdigit(*q) && isdigit(*(q+1)) && isdigit(*(q+2))) {
            multipleFileNumber=atoi(q);
            fmt=0;
            p=q;
            while(isdigit(*q)) {
                fmt++;
                q++;
            }
            *p='\0';
            if (((fmt<3)  || ((fmt==3) && (numImages>999))) || 
                ((fmt==4) && (numImages>9999))) { 
                fmt=5;
            }
        } else if (*q) {
            strcat(p, "_"); /* force '_' ending */
        }
    } else {
        strcat(p, "_"); /* force '_' ending */
    }
    /* Build the final format string */
    epicsSnprintf(this->multipleFileFormat, sizeof(this->multipleFileFormat), "%s%%.%dd%s",
                  mfTempFormat, fmt, mfExtension);
}

/** This function waits for the specified file to exist.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus pilatusDetector::waitForFileToExist(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    int fd=-1;
    int fileExists=0;
    struct stat statBuff;
    epicsTimeStamp tStart, tCheck;
    time_t acqStartTime;
    double deltaTime=0.;
    int status=-1;
    const char *functionName = "waitForFileToExist";

    if (pStartTime) epicsTimeToTime_t(&acqStartTime, pStartTime);
    epicsTimeGetCurrent(&tStart);

    while (deltaTime <= timeout) {
	fd = open(fileName, O_RDONLY, 0);
        if ((fd >= 0) && (timeout != 0.)) {
            fileExists = 1;
            /* The file exists.  Make sure it is a new file, not an old one.
             * We don't do this check if timeout==0, which is used for reading flat field files */
            status = fstat(fd, &statBuff);
            if (status){
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s error calling fstat, errno=%d %s\n",
                    driverName, functionName, errno, fileName);
                close(fd);
                return(asynError);
            }
            /* We allow up to 10 second clock skew between time on machine running this IOC
             * and the machine with the file system returning modification time */
            if (difftime(statBuff.st_mtime, acqStartTime) > -10) break;
            close(fd);
            fd = -1;
        }
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        if (status == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }
    if (fd < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s timeout waiting for file to be created %s\n",
            driverName, functionName, fileName);
        if (fileExists) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "  file exists but is more than 10 seconds old, possible clock synchronization problem\n");
            setStringParam(ADStatusMessage, "Image file is more than 10 seconds old");
        } else
            setStringParam(ADStatusMessage, "Timeout waiting for file to be created");
        return(asynError);
    }
    close(fd);
    return(asynSuccess);
}

/** This function replaces bad pixels in the specified image with their replacements
 * according to the bad pixel map.
 */
void pilatusDetector::correctBadPixels(NDArray *pImage)
{
    int i;
    int numBadPixels;

    getIntegerParam(PilatusNumBadPixels, &numBadPixels);
    for (i=0; i<numBadPixels; i++) {
        ((epicsInt32 *)pImage->pData)[this->badPixelMap[i].badIndex] = 
        ((epicsInt32 *)pImage->pData)[this->badPixelMap[i].replaceIndex];
    }    
}

int pilatusDetector::stringEndsWith(const char *aString, const char *aSubstring, int shouldIgnoreCase)
{
    int i, j;

    i = strlen(aString) - 1;
    j = strlen(aSubstring) - 1;
    while (i >= 0 && j >= 0) {
        if (shouldIgnoreCase) {
            if (tolower(aString[i]) != tolower(aSubstring[j])) return 0;
        } else {
            if (aString[i] != aSubstring[j]) return 0;
        }
        i--; j--;
    }
    return j < 0;
}

/** This function reads TIFF or CBF image files.  It is not intended to be general, it
 * is intended to read the TIFF or CBF files that camserver creates.  It checks to make
 * sure that the creation time of the file is after a start time passed to it, to force
 * it to wait for a new file to be created.
 */
asynStatus pilatusDetector::readImageFile(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    const char *functionName = "readImageFile";

    if (stringEndsWith(fileName, ".tif", 1) || stringEndsWith(fileName, ".tiff", 1)) {
        return readTiff(fileName, pStartTime, timeout, pImage);
    } else if (stringEndsWith(fileName, ".cbf", 1)) {
        return readCbf(fileName, pStartTime, timeout, pImage);
    } else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, unsupported image file name extension, expected .tif or .cbf, fileName=%s\n",
            driverName, functionName, fileName);
        setStringParam(ADStatusMessage, "Unsupported file extension, expected .tif or .cbf");
        return(asynError);
    }
}

/** This function reads CBF files using CBFlib.  It is not intended to be general, it is
 * intended to read the CBF files that camserver creates.  It checks to make sure that
 * the creation time of the file is after a start time passed to it, to force it to wait
 * for a new file to be created.
 */
asynStatus pilatusDetector::readCbf(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    int status=-1;
    const char *functionName = "readCbf";
    cbf_handle cbf;
    FILE *file=NULL;
    unsigned int cbfCompression;
    int cbfBinaryId;
    size_t cbfElSize;
    int cbfElSigned;
    int cbfElUnsigned;
    size_t cbfElements;
    int cbfMinElement;
    int cbfMaxElement;
    const char *cbfByteOrder;
    size_t cbfDimFast;
    size_t cbfDimMid;
    size_t cbfDimSlow;
    size_t cbfPadding;
    size_t cbfElementsRead;
    int deleteFlag=0;

    deltaTime = 0.;
    epicsTimeGetCurrent(&tStart);

    status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
    if (status != asynSuccess) {
        return((asynStatus)status);
    }

    cbf_set_warning_messages_enabled(0);
    cbf_set_error_messages_enabled(0);

    while (deltaTime <= timeout) {
        /* At this point we know the file exists, but it may not be completely
         * written yet.  If we get errors then try again. */

        status = cbf_make_handle(&cbf);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, failed to make CBF handle, error code %#x\n",
                driverName, functionName, status);
            return(asynError);
        }

        status = cbf_set_cbf_logfile(cbf, NULL);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, failed to disable CBF logging, error code %#x\n",
                driverName, functionName, status);
            return(asynError);
        }

        file = fopen(fileName, "rb");
        if (file == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, failed to open CBF file \"%s\" for reading: %s\n",
                driverName, functionName, fileName, strerror(errno));
            cbf_free_handle(cbf);
            return(asynError);
        }

        status = cbf_read_widefile(cbf, file, MSG_DIGESTNOW);
        if (status != 0) goto retry;

        status = cbf_find_tag(cbf, "_array_data.data");
        if (status != 0) goto retry;

        /* Do some basic checking that the image size is what we expect */

        status = cbf_get_integerarrayparameters_wdims_fs(cbf, &cbfCompression,
            &cbfBinaryId, &cbfElSize, &cbfElSigned, &cbfElUnsigned,
            &cbfElements, &cbfMinElement, &cbfMaxElement, &cbfByteOrder,
            &cbfDimFast, &cbfDimMid, &cbfDimSlow, &cbfPadding);
        if (status != 0) goto retry;

        if (cbfDimFast != (size_t)pImage->dims[0].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image width incorrect =%zd, should be %d\n",
                driverName, functionName, cbfDimFast, pImage->dims[0].size);
            cbf_free_handle(cbf);
            return(asynError);
        }
        if (cbfDimMid != (size_t)pImage->dims[1].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image height incorrect =%zd, should be %d\n",
                driverName, functionName, cbfDimMid, pImage->dims[1].size);
            cbf_free_handle(cbf);
            return(asynError);
        }

        /* Read the image */

        status = cbf_get_integerarray(cbf, &cbfBinaryId, pImage->pData,
            sizeof(epicsInt32), 1, cbfElements, &cbfElementsRead);
        if (status != 0) goto retry;
        if (cbfElements != cbfElementsRead) goto retry;

        /* Sucesss! */
        status = cbf_free_handle(cbf);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, failed to free CBF handle, error code %#x\n",
                driverName, functionName, status);
            return(asynError);
        }
        break;

        retry:
        status = cbf_free_handle(cbf);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, failed to free CBF handle, error code %#x\n",
                driverName, functionName, status);
            return(asynError);
        }
        /* Sleep, but check for stop event, which can be used to abort a long
         * acquisition */
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        if (status == epicsEventWaitOK) {
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }


    correctBadPixels(pImage);

    /* Delete the file if necessary */
    getIntegerParam(ADDeleteAfterReading, &deleteFlag);
    if (deleteFlag) {
        status = remove(fileName);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, failed to delete CBF file after reading: %s\n",
            driverName, functionName, fileName);
            return(asynError);
        }
    }
    return(asynSuccess);
}

/** This function reads TIFF files using libTiff.  It is not intended to be general,
 * it is intended to read the TIFF files that camserver creates.  It checks to make sure
 * that the creation time of the file is after a start time passed to it, to force it to
 * wait for a new file to be created.
 */
asynStatus pilatusDetector::readTiff(const char *fileName, epicsTimeStamp *pStartTime, double timeout, NDArray *pImage)
{
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    int status=-1;
    const char *functionName = "readTiff";
    int size, totalSize;
    int numStrips, strip;
    char *buffer;
    TIFF *tiff=NULL;
    epicsUInt32 uval;
    int deleteFlag=0;

    deltaTime = 0.;
    epicsTimeGetCurrent(&tStart);

    /* Suppress error messages from the TIFF library */
    TIFFSetErrorHandler(NULL);
    TIFFSetWarningHandler(NULL);

    status = waitForFileToExist(fileName, pStartTime, timeout, pImage);
    if (status != asynSuccess) {
        return((asynStatus)status);
    }

    while (deltaTime <= timeout) {
        /* At this point we know the file exists, but it may not be completely written yet.
         * If we get errors then try again */
        tiff = TIFFOpen(fileName, "rc");
        if (tiff == NULL) {
            status = asynError;
            goto retry;
        }
        
        /* Do some basic checking that the image size is what we expect */
        status = TIFFGetField(tiff, TIFFTAG_IMAGEWIDTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[0].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image width incorrect =%u, should be %d\n",
                driverName, functionName, uval, pImage->dims[0].size);
            goto retry;
        }
        status = TIFFGetField(tiff, TIFFTAG_IMAGELENGTH, &uval);
        if (uval != (epicsUInt32)pImage->dims[1].size) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, image length incorrect =%u, should be %d\n",
                driverName, functionName, uval, pImage->dims[1].size);
            goto retry;
        }
        numStrips= TIFFNumberOfStrips(tiff);
        buffer = (char *)pImage->pData;
        totalSize = 0;
        for (strip=0; (strip < numStrips) && (totalSize < pImage->dataSize); strip++) {
            size = TIFFReadEncodedStrip(tiff, 0, buffer, pImage->dataSize-totalSize);
            if (size == -1) {
                /* There was an error reading the file.  Most commonly this is because the file
                 * was not yet completely written.  Try again. */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s, error reading TIFF file %s\n",
                    driverName, functionName, fileName);
                goto retry;
            }
            buffer += size;
            totalSize += size;
        }
        if (totalSize != pImage->dataSize) {
            status = asynError;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s, file size incorrect =%d, should be %d\n",
                driverName, functionName, totalSize, pImage->dataSize);
            goto retry;
        }
        /* Sucesss! */
        break;
        
        retry:
        if (tiff != NULL) TIFFClose(tiff);
        tiff = NULL;
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        status = epicsEventWaitWithTimeout(this->stopEventId, FILE_READ_DELAY);
        if (status == epicsEventWaitOK) {
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    if (tiff != NULL) TIFFClose(tiff);

    correctBadPixels(pImage);

    /* Delete the file if necessary */
    getIntegerParam(ADDeleteAfterReading, &deleteFlag);
    if (deleteFlag) {
        status = remove(fileName);
        if (status != 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s, failed to delete TIFF file after reading: %s\n",
            driverName, functionName, fileName);
            return(asynError);
        }
    }
    
    return(asynSuccess);
}   

asynStatus pilatusDetector::setAcquireParams()
{
    int ival;
    double dval;
    int triggerMode;
    asynStatus status;
    char *substr = NULL;
    int pixelCutOff = 0;
    
    status = getIntegerParam(ADTriggerMode, &triggerMode);
    if (status != asynSuccess) triggerMode = TMInternal;
    
     /* When we change modes download all exposure parameters, since some modes
     * replace values with new parameters */
    if (triggerMode == TMAlignment) {
        setIntegerParam(ADNumImages, 1);
    }
    /* nexpf > 1 is only supported in External Enable mode */  
    if (triggerMode != TMExternalEnable) {
        setIntegerParam(ADNumExposures, 1);
    }
    
    status = getIntegerParam(ADNumImages, &ival);
    if ((status != asynSuccess) || (ival < 1)) {
        ival = 1;
        setIntegerParam(ADNumImages, ival);
    }
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "nimages %d", ival);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT); 

    status = getIntegerParam(ADNumExposures, &ival);
    if ((status != asynSuccess) || (ival < 1)) {
        ival = 1;
        setIntegerParam(ADNumExposures, ival);
    }
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "nexpframe %d", ival);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT); 

    status = getDoubleParam(ADAcquireTime, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 1.;
        setDoubleParam(ADAcquireTime, dval);
    }
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "exptime %f", dval);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

    status = getDoubleParam(ADAcquirePeriod, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 2.;
        setDoubleParam(ADAcquirePeriod, dval);
    }
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "expperiod %f", dval);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

    status = getDoubleParam(PilatusDelayTime, &dval);
    if ((status != asynSuccess) || (dval < 0.)) {
        dval = 0.;
        setDoubleParam(PilatusDelayTime, dval);
    }
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "delay %f", dval);
    writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

    status = getIntegerParam(PilatusGapFill, &ival);
    if ((status != asynSuccess) || (ival < -2) || (ival > 0)) {
        ival = -2;
        setIntegerParam(PilatusGapFill, ival);
    }
    /* -2 is used to indicate that GapFill is not supported because it is a single element detector */
    if (ival != -2) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "gapfill %d", ival);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT); 
    }

    /*Read back the pixel count rate cut off value.*/
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "Tau");
    status=writeReadCamserver(5.0); 

    /*Response contains the string "cutoff = 1221026 counts"*/
    if (!status) {
      if ((substr = strstr(this->fromCamserver, "cutoff")) != NULL) {
	sscanf(substr, "cutoff = %d counts", &pixelCutOff);
	setIntegerParam(PilatusPixelCutOff, pixelCutOff);
      }
    }
   
    return(asynSuccess);

}

asynStatus pilatusDetector::setThreshold()
{
    int igain, status;
    double threshold, dgain;
    char *substr = NULL;
    int threshold_readback = 0;
    
    getDoubleParam(ADGain, &dgain);
    igain = (int)(dgain + 0.5);
    if (igain < 0) igain = 0;
    if (igain > 3) igain = 3;
    getDoubleParam(PilatusThreshold, &threshold);
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "SetThreshold %s %f", 
                    gainStrings[igain], threshold*1000.);
    /* Set the status to waiting so we can be notified when it has finished */
    setIntegerParam(ADStatus, ADStatusWaiting);
    setStringParam(ADStatusMessage, "Setting threshold");
    callParamCallbacks();
    
    status=writeReadCamserver(90.0);  /* This command can take 78 seconds on a 6M */
    if (status)
        setIntegerParam(ADStatus, ADStatusError);
    else
        setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(PilatusThresholdApply, 0);

    /* Read back the actual setting, in case we are out of bounds.*/
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "SetThreshold");
    status=writeReadCamserver(5.0); 

    /* Response should contain "threshold: 9000 eV; vcmp:"*/
    if (!status) {
      if ((substr = strstr(this->fromCamserver, "threshold: ")) != NULL) {
	sscanf(strtok(substr, ";"), "threshold: %d eV", &threshold_readback);
	setDoubleParam(PilatusThreshold, (double)threshold_readback/1000.0);
      }
    }
    

    /* The SetThreshold command resets numimages to 1 and gapfill to 0, so re-send current
     * acquisition parameters */
    setAcquireParams();

    callParamCallbacks();

    return(asynSuccess);
}

asynStatus pilatusDetector::writeCamserver(double timeout)
{
    size_t nwrite;
    asynStatus status;
    const char *functionName="writeCamserver";

    /* Flush any stale input, since the next operation is likely to be a read */
    status = pasynOctetSyncIO->flush(this->pasynUserCamserver);
    status = pasynOctetSyncIO->write(this->pasynUserCamserver, this->toCamserver,
                                     strlen(this->toCamserver), timeout,
                                     &nwrite);
                                        
    if (status) asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s, status=%d, sent\n%s\n",
                    driverName, functionName, status, this->toCamserver);

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringToServer, this->toCamserver);
    
    return(status);
}


asynStatus pilatusDetector::readCamserver(double timeout)
{
    size_t nread;
    asynStatus status=asynSuccess;
    int eventStatus;
    asynUser *pasynUser = this->pasynUserCamserver;
    int eomReason;
    epicsTimeStamp tStart, tCheck;
    double deltaTime;
    const char *functionName="readCamserver";

    /* We implement the timeout with a loop so that the port does not
     * block during the entire read.  If we don't do this then it is not possible
     * to abort a long exposure */
    deltaTime = 0;
    epicsTimeGetCurrent(&tStart);
    while (deltaTime <= timeout) {
        status = pasynOctetSyncIO->read(pasynUser, this->fromCamserver,
                                        sizeof(this->fromCamserver), ASYN_POLL_TIME,
                                        &nread, &eomReason);
        if (status != asynTimeout) break;
        /* Sleep, but check for stop event, which can be used to abort a long acquisition */
        eventStatus = epicsEventWaitWithTimeout(this->stopEventId, ASYN_POLL_TIME);
        if (eventStatus == epicsEventWaitOK) {
            setStringParam(ADStatusMessage, "Acquisition aborted");
            return(asynError);
        }
        epicsTimeGetCurrent(&tCheck);
        deltaTime = epicsTimeDiffInSeconds(&tCheck, &tStart);
    }

    // If we got asynTimeout, and timeout=0 then this is not an error, it is a poll checking for possible reply and we are done
   if ((status == asynTimeout) && (timeout == 0)) return(asynSuccess);
   if (status != asynSuccess)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s, timeout=%f, status=%d received %d bytes\n%s\n",
                    driverName, functionName, timeout, status, nread, this->fromCamserver);
    else {
        /* Look for the string OK in the response */
        if (!strstr(this->fromCamserver, "OK")) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s unexpected response from camserver, no OK, response=%s\n",
                      driverName, functionName, this->fromCamserver);
            setStringParam(ADStatusMessage, "Error from camserver");
            status = asynError;
        } else
            setStringParam(ADStatusMessage, "Camserver returned OK");
    }

    /* Set output string so it can get back to EPICS */
    setStringParam(ADStringFromServer, this->fromCamserver);

    return(status);
}

asynStatus pilatusDetector::writeReadCamserver(double timeout)
{
    asynStatus status;
    
    status = writeCamserver(timeout);
    if (status) return status;
    status = readCamserver(timeout);
    return status;
}

static void pilatusTaskC(void *drvPvt)
{
    pilatusDetector *pPvt = (pilatusDetector *)drvPvt;
    
    pPvt->pilatusTask();
}

/** This thread controls acquisition, reads image files to get the image data, and
  * does the callbacks to send it to higher layers */
void pilatusDetector::pilatusTask()
{
    int status = asynSuccess;
    int imageCounter;
    int numImages;
    int multipleFileNextImage=0;  /* This is the next image number, starting at 0 */
    int acquire;
    ADStatus_t acquiring;
    double startAngle;
    NDArray *pImage;
    double acquireTime, acquirePeriod;
    double readImageFileTimeout, timeout;
    int triggerMode;
    epicsTimeStamp startTime;
    const char *functionName = "pilatusTask";
    char fullFileName[MAX_FILENAME_LEN];
    char filePath[MAX_FILENAME_LEN];
    char statusMessage[MAX_MESSAGE_SIZE];
    int dims[2];
    int arrayCallbacks;
    int flatFieldValid;
    int abortStatus;

    this->lock();

    /* Loop forever */
    while (1) {
	/* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);
        /* If we are not acquiring then wait for a semaphore that is given when acquisition is started */
        if (!acquire) {
            /* Only set the status message if we didn't encounter any errors last time, so we don't overwrite the 
             error message */
            if (!status)
            setStringParam(ADStatusMessage, "Waiting for acquire command");
            callParamCallbacks();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            this->unlock();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                "%s:%s: waiting for acquire to start\n", driverName, functionName);
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
        getDoubleParam(PilatusImageFileTmot, &readImageFileTimeout);
        
        /* Get the acquisition parameters */
        getIntegerParam(ADTriggerMode, &triggerMode);
        getIntegerParam(ADNumImages, &numImages);
        
        acquiring = ADStatusAcquire;
        setIntegerParam(ADStatus, acquiring);

        /* Reset the MX settings start angle */
        getDoubleParam(PilatusStartAngle, &startAngle);
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Start_angle %f", startAngle);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);

        /* Create the full filename */
        createFileName(sizeof(fullFileName), fullFileName);
        
        switch (triggerMode) {
            case TMInternal:
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                    "Exposure %s", fullFileName);
                break;
            case TMExternalEnable:
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                    "ExtEnable %s", fullFileName);
                break;
            case TMExternalTrigger:
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                    "ExtTrigger %s", fullFileName);
                break;
            case TMMultipleExternalTrigger:
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                    "ExtMTrigger %s", fullFileName);
                break;
            case TMAlignment:
                getStringParam(NDFilePath, sizeof(filePath), filePath);
                epicsSnprintf(fullFileName, sizeof(fullFileName), "%salignment.tif", 
                              filePath);
                epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                    "Exposure %s", fullFileName);
                break;
        }
        setStringParam(ADStatusMessage, "Starting exposure");
        /* Send the acquire command to camserver and wait for the 15OK response */
        writeReadCamserver(2.0);
	
	/* Do another read in case there is an ERR string at the end of the input buffer */
	status=readCamserver(0.0);
		
	/* If the status wasn't asynSuccess or asynTimeout, report the error */
	if (status>1) {
	    acquire = 0;
	}
	else {
	    /* Set status back to asynSuccess as the timeout was expected */
	    status = asynSuccess;
            /* Open the shutter */
            setShutter(1);
            /* Set the armed flag */
            setIntegerParam(PilatusArmed, 1);
            /* Create the format string for constructing file names for multi-image collection */
            makeMultipleFileFormat(fullFileName);
            multipleFileNextImage = 0;
            /* Call the callbacks to update any changes */
            setStringParam(NDFullFileName, fullFileName);
            callParamCallbacks();
	}

        /* If the status wasn't asynSuccess or asynTimeout, report the error */
        if (status>1) {
            acquire = 0;
        }
        else {
            /* Set status back to asynSuccess as the timeout was expected */
            status = asynSuccess;
            /* Open the shutter */
            setShutter(1);
            /* Set the armed flag */
            setIntegerParam(PilatusArmed, 1);
            /* Create the format string for constructing file names for multi-image collection */
            makeMultipleFileFormat(fullFileName);
            multipleFileNextImage = 0;
            /* Call the callbacks to update any changes */
            setStringParam(NDFullFileName, fullFileName);
            callParamCallbacks();
        }

        while (acquire) {
            if (numImages == 1) {
                /* For single frame or alignment mode need to wait for 7OK response from camserver
                 * saying acquisition is complete before trying to read file, else we get a
                 * recent but stale file. */
                setStringParam(ADStatusMessage, "Waiting for 7OK response");
                callParamCallbacks();
                /* We release the mutex when waiting for 7OK because this takes a long time and
                 * we need to allow abort operations to get through */
                this->unlock();
                status = readCamserver(acquireTime + readImageFileTimeout);
                this->lock();
                /* If there was an error jump to bottom of loop */
                if (status) {
                    acquire = 0;
                    if(status==asynTimeout)
                      setStringParam(ADStatusMessage, "Timeout waiting for camserver response");
                    continue;
                }
             } else {
                /* If this is a multi-file acquisition the file name is built differently */
                epicsSnprintf(fullFileName, sizeof(fullFileName), multipleFileFormat, 
                              multipleFileNumber);
                setStringParam(NDFullFileName, fullFileName);
            }
            getIntegerParam(NDArrayCallbacks, &arrayCallbacks);

	    if (arrayCallbacks) {
                getIntegerParam(NDArrayCounter, &imageCounter);
                imageCounter++;
                setIntegerParam(NDArrayCounter, imageCounter);
                /* Call the callbacks to update any changes */
                callParamCallbacks();

                /* Get an image buffer from the pool */
                getIntegerParam(ADMaxSizeX, &dims[0]);
                getIntegerParam(ADMaxSizeY, &dims[1]);
                pImage = this->pNDArrayPool->alloc(2, dims, NDInt32, 0, NULL);
                epicsSnprintf(statusMessage, sizeof(statusMessage), "Reading image file %s", fullFileName);
                setStringParam(ADStatusMessage, statusMessage);
                callParamCallbacks();
                /* We release the mutex when calling readImageFile, because this takes a long time and
                 * we need to allow abort operations to get through */
                this->unlock();
                status = readImageFile(fullFileName, &startTime, acquireTime + readImageFileTimeout, pImage); 
                this->lock();
                /* If there was an error jump to bottom of loop */
                if (status) {
                    acquire = 0;
                    pImage->release();
                    continue;
                }

                getIntegerParam(PilatusFlatFieldValid, &flatFieldValid);
                if (flatFieldValid) {
                    epicsInt32 *pData, *pFlat, i;
                    for (i=0, pData = (epicsInt32 *)pImage->pData, pFlat = (epicsInt32 *)this->pFlatField->pData;
                         i<dims[0]*dims[1]; 
                         i++, pData++, pFlat++) {
                        *pData = (epicsInt32)((this->averageFlatField * *pData) / *pFlat);
                    }
                } 
                /* Put the frame number and time stamp into the buffer */
                pImage->uniqueId = imageCounter;
                pImage->timeStamp = startTime.secPastEpoch + startTime.nsec / 1.e9;

                /* Get any attributes that have been defined for this driver */        
                this->getAttributes(pImage->pAttributeList);
                
                /* Call the NDArray callback */
                /* Must release the lock here, or we can get into a deadlock, because we can
                 * block on the plugin lock, and the plugin can be calling us */
                this->unlock();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
                     "%s:%s: calling NDArray callback\n", driverName, functionName);
                doCallbacksGenericPointer(pImage, NDArrayData, 0);
                this->lock();
                /* Free the image buffer */
                pImage->release();
            }
            if (numImages == 1) {
                if (triggerMode == TMAlignment) {
                   epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), 
                        "Exposure %s", fullFileName);
                    /* Send the acquire command to camserver and wait for the 15OK response */
                    writeReadCamserver(2.0);
                } else {
                    acquire = 0;
                }
            } else if (numImages > 1) {
                multipleFileNextImage++;
                multipleFileNumber++;
                if (multipleFileNextImage == numImages) acquire = 0;
            }
            
        }
        /* We are done acquiring */
        /* Wait for the 7OK response from camserver in the case of multiple images */
        if ((numImages > 1) && (status == asynSuccess)) {
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
            readCamserver(timeout);
            this->lock();
        }
        setShutter(0);
        setIntegerParam(ADAcquire, 0);
        setIntegerParam(PilatusArmed, 0);

        /* If everything was ok, set the status back to idle */
        if (!status) 
            setIntegerParam(ADStatus, ADStatusIdle);
        else
            setIntegerParam(ADStatus, ADStatusError);
    
        /* Call the callbacks to update any changes */
        callParamCallbacks();
    }
}

static void pilatusStatusC(void *drvPvt)
{
    pilatusDetector *pPvt = (pilatusDetector *)drvPvt;
    
    pPvt->pilatusStatus();
}

/** This thread periodically read the detector status (temperature, humidity, etc.)
    It does not run if we are acquiring data, to avoid polling camserver when taking data.*/
void pilatusDetector::pilatusStatus()
{
  int status = asynSuccess;
  int acquire = 0;
  float temp = 0.0;
  float humid = 0.0;
  char *substr = NULL;

  while(1) {
    lock();
    /* Is acquisition active? */
    getIntegerParam(ADAcquire, &acquire);
    if (!acquire) {
      /*Keep lock and read temp and humidity.*/
      epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "Th");
      status=writeReadCamserver(1.0); 
      
      /* Response should contain: 
	 Channel 0: Temperature = 31.4C, Rel. Humidity = 22.1%;\n
	 Channel 1: Temperature = 25.8C, Rel. Humidity = 33.5%;\n
	 Channel 2: Temperature = 28.6C, Rel. Humidity = 2.0%
	 However, not every detector has all 3 channels.*/
      if (!status) {
	if ((substr = strstr(this->fromCamserver, "Channel 0: ")) != NULL) {
	  sscanf(strtok(substr, "%"), "Channel 0: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
	  setDoubleParam(PilatusThTemp0, temp);
	  setDoubleParam(PilatusThHumid0, humid);
	  setDoubleParam(ADTemperature, temp);
	}
	if ((substr = strstr(this->fromCamserver, "Channel 1: ")) != NULL) {
	  sscanf(strtok(substr, "%"), "Channel 1: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
	  setDoubleParam(PilatusThTemp1, temp);
	  setDoubleParam(PilatusThHumid1, humid);
	}
	if ((substr = strstr(this->fromCamserver, "Channel 2: ")) != NULL) {
	  sscanf(strtok(substr, "%"), "Channel 2: Temperature = %fC, Rel. Humidity = %f", &temp, &humid);
	  setDoubleParam(PilatusThTemp2, temp);
	  setDoubleParam(PilatusThHumid2, humid);
	}
      }
      
      callParamCallbacks();
      unlock();
    } else {
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
asynStatus pilatusDetector::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int adstatus;
    asynStatus status = asynSuccess;
    const char *functionName = "writeInt32";

    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        getIntegerParam(ADStatus, &adstatus);
        if (value && (adstatus == ADStatusIdle || adstatus == ADStatusError)) {
            /* Send an event to wake up the Pilatus task.  */
            epicsEventSignal(this->startEventId);
        } 
        if (!value && (adstatus == ADStatusAcquire)) {
            /* This was a command to stop acquisition */
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "Stop");
            writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
            epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "K");
            writeCamserver(CAMSERVER_DEFAULT_TIMEOUT);
            epicsEventSignal(this->stopEventId);
        }
    } else if ((function == ADTriggerMode) ||
               (function == ADNumImages) ||
               (function == ADNumExposures) ||
               (function == PilatusGapFill)) {
        setAcquireParams();
    } else if (function == PilatusThresholdApply) {
        setThreshold();
    } else if (function == PilatusNumOscill) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings N_oscillations %d", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else { 
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PILATUS_PARAM) status = ADDriver::writeInt32(pasynUser, value);
    }
            
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    
    if (status) 
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s: error, status=%d function=%d, value=%d\n", 
              driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%d\n", 
              driverName, functionName, function, value);
    return status;
}


/** Called when asyn clients call pasynFloat64->write().
  * This function performs actions for some parameters, including ADAcquireTime, ADGain, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus pilatusDetector::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    double energyLow, energyHigh;
    double beamX, beamY;
    int thresholdAutoApply;
    const char *functionName = "writeFloat64";
    double oldValue;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    getDoubleParam(function, &oldValue);
    status = setDoubleParam(function, value);
    

    /* Changing any of the following parameters requires recomputing the base image */
    if ((function == ADGain) ||
        (function == PilatusThreshold)) {
        getIntegerParam(PilatusThresholdAutoApply, &thresholdAutoApply);
        if (thresholdAutoApply) 
            setThreshold();
    } else if ((function == ADAcquireTime) ||
               (function == ADAcquirePeriod) ||
               (function == PilatusDelayTime)) {
        setAcquireParams();
    } else if (function == PilatusWavelength) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Wavelength %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if ((function == PilatusEnergyLow) ||
               (function == PilatusEnergyHigh)) {
        getDoubleParam(PilatusEnergyLow, &energyLow);
        getDoubleParam(PilatusEnergyHigh, &energyHigh);
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Energy_range %f,%f", energyLow, energyHigh);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusDetDist) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Detector_distance %f", value / 1000.0);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusDetVOffset) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Detector_Voffset %f", value / 1000.0);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if ((function == PilatusBeamX) ||
               (function == PilatusBeamY)) {
        getDoubleParam(PilatusBeamX, &beamX);
        getDoubleParam(PilatusBeamY, &beamY);
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Beam_xy %f,%f", beamX, beamY);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusFlux) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Flux %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusFilterTransm) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Filter_transmission %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusStartAngle) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Start_angle %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusAngleIncr) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Angle_increment %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusDet2theta) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Detector_2theta %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusPolarization) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Polarization %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusAlpha) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Alpha %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusKappa) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Kappa %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusPhi) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Phi %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else if (function == PilatusChi) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Chi %f", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PILATUS_PARAM) status = ADDriver::writeFloat64(pasynUser, value);
    }

    if (status) {
        /* Something went wrong so we set the old value back */
        setDoubleParam(function, oldValue);
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
              "%s:%s error, status=%d function=%d, value=%f\n", 
              driverName, functionName, status, function, value);
    }
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%f\n", 
              driverName, functionName, function, value);
    
    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();
    return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including PilatusBadPixelFile, ADFilePath, etc.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus pilatusDetector::writeOctet(asynUser *pasynUser, const char *value, 
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";

    /* Set the parameter in the parameter library. */
    status = (asynStatus)setStringParam(function, (char *)value);

    if (function == PilatusBadPixelFile) {
        this->readBadPixelFile(value);
    } else if (function == PilatusFlatFieldFile) {
        this->readFlatFieldFile(value);
    } else if (function == NDFilePath) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "imgpath %s", value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
        this->checkPath();
    } else if (function == PilatusOscillAxis) {
        epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "mxsettings Oscillation_axis %s",
            strlen(value) == 0 ? "(nil)" : value);
        writeReadCamserver(CAMSERVER_DEFAULT_TIMEOUT);
    } else {
        /* If this parameter belongs to a base class call its method */
        if (function < FIRST_PILATUS_PARAM) status = ADDriver::writeOctet(pasynUser, value, nChars, nActual);
    }
    
     /* Do callbacks so higher layers see any changes */
    status = (asynStatus)callParamCallbacks();

    if (status) 
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize, 
                  "%s:%s: status=%d, function=%d, value=%s", 
                  driverName, functionName, status, function, value);
    else        
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: function=%d, value=%s\n", 
              driverName, functionName, function, value);
    *nActual = nChars;
    return status;
}



/** Report status of the driver.
  * Prints details about the driver if details>0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void pilatusDetector::report(FILE *fp, int details)
{

    fprintf(fp, "Pilatus detector %s\n", this->portName);
    if (details > 0) {
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

extern "C" int pilatusDetectorConfig(const char *portName, const char *camserverPort, 
                                    int maxSizeX, int maxSizeY,
                                    int maxBuffers, size_t maxMemory,
                                    int priority, int stackSize)
{
    new pilatusDetector(portName, camserverPort, maxSizeX, maxSizeY, maxBuffers, maxMemory,
                        priority, stackSize);
    return(asynSuccess);
}

/** Constructor for Pilatus driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver, and ADDriver.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] camserverPort The name of the asyn port previously created with drvAsynIPPortConfigure to
  *            communicate with camserver.
  * \param[in] maxSizeX The size of the Pilatus detector in the X direction.
  * \param[in] maxSizeY The size of the Pilatus detector in the Y direction.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is 
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in] priority The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in] stackSize The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
pilatusDetector::pilatusDetector(const char *portName, const char *camserverPort,
                                int maxSizeX, int maxSizeY,
                                int maxBuffers, size_t maxMemory,
                                int priority, int stackSize)

    : ADDriver(portName, 1, NUM_PILATUS_PARAMS, maxBuffers, maxMemory,
               0, 0,             /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
      imagesRemaining(0)

{
    int status = asynSuccess;
    const char *functionName = "pilatusDetector";
    int dims[2];
    char *substr = NULL;

    /* Create the epicsEvents for signaling to the pilatus task when acquisition starts and stops */
    this->startEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startEventId) {
        printf("%s:%s epicsEventCreate failure for start event\n", 
            driverName, functionName);
        return;
    }
    this->stopEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopEventId) {
        printf("%s:%s epicsEventCreate failure for stop event\n", 
            driverName, functionName);
        return;
    }
    
    /* Allocate the raw buffer we use to read image files.  Only do this once */
    dims[0] = maxSizeX;
    dims[1] = maxSizeY;
    /* Allocate the raw buffer we use for flat fields. */
    this->pFlatField = this->pNDArrayPool->alloc(2, dims, NDUInt32, 0, NULL);
    
    /* Connect to camserver */
    status = pasynOctetSyncIO->connect(camserverPort, 0, &this->pasynUserCamserver, NULL);

    createParam(PilatusDelayTimeString,      asynParamFloat64, &PilatusDelayTime);
    createParam(PilatusThresholdString,      asynParamFloat64, &PilatusThreshold);
    createParam(PilatusThresholdApplyString, asynParamInt32,   &PilatusThresholdApply);
    createParam(PilatusThresholdAutoApplyString, asynParamInt32,   &PilatusThresholdAutoApply);
    createParam(PilatusArmedString,          asynParamInt32,   &PilatusArmed);
    createParam(PilatusImageFileTmotString,  asynParamFloat64, &PilatusImageFileTmot);
    createParam(PilatusBadPixelFileString,   asynParamOctet,   &PilatusBadPixelFile);
    createParam(PilatusNumBadPixelsString,   asynParamInt32,   &PilatusNumBadPixels);
    createParam(PilatusFlatFieldFileString,  asynParamOctet,   &PilatusFlatFieldFile);
    createParam(PilatusMinFlatFieldString,   asynParamInt32,   &PilatusMinFlatField);
    createParam(PilatusFlatFieldValidString, asynParamInt32,   &PilatusFlatFieldValid);
    createParam(PilatusGapFillString,        asynParamInt32,   &PilatusGapFill);
    createParam(PilatusWavelengthString,     asynParamFloat64, &PilatusWavelength);
    createParam(PilatusEnergyLowString,      asynParamFloat64, &PilatusEnergyLow);
    createParam(PilatusEnergyHighString,     asynParamFloat64, &PilatusEnergyHigh);
    createParam(PilatusDetDistString,        asynParamFloat64, &PilatusDetDist);
    createParam(PilatusDetVOffsetString,     asynParamFloat64, &PilatusDetVOffset);
    createParam(PilatusBeamXString,          asynParamFloat64, &PilatusBeamX);
    createParam(PilatusBeamYString,          asynParamFloat64, &PilatusBeamY);
    createParam(PilatusFluxString,           asynParamFloat64, &PilatusFlux);
    createParam(PilatusFilterTransmString,   asynParamFloat64, &PilatusFilterTransm);
    createParam(PilatusStartAngleString,     asynParamFloat64, &PilatusStartAngle);
    createParam(PilatusAngleIncrString,      asynParamFloat64, &PilatusAngleIncr);
    createParam(PilatusDet2thetaString,      asynParamFloat64, &PilatusDet2theta);
    createParam(PilatusPolarizationString,   asynParamFloat64, &PilatusPolarization);
    createParam(PilatusAlphaString,          asynParamFloat64, &PilatusAlpha);
    createParam(PilatusKappaString,          asynParamFloat64, &PilatusKappa);
    createParam(PilatusPhiString,            asynParamFloat64, &PilatusPhi);
    createParam(PilatusChiString,            asynParamFloat64, &PilatusChi);
    createParam(PilatusOscillAxisString,     asynParamOctet,   &PilatusOscillAxis);
    createParam(PilatusNumOscillString,      asynParamInt32,   &PilatusNumOscill);
    createParam(PilatusPixelCutOffString,    asynParamInt32,   &PilatusPixelCutOff);
    createParam(PilatusThTemp0String,        asynParamFloat64, &PilatusThTemp0);
    createParam(PilatusThTemp1String,        asynParamFloat64, &PilatusThTemp1);
    createParam(PilatusThTemp2String,        asynParamFloat64, &PilatusThTemp2);
    createParam(PilatusThHumid0String,       asynParamFloat64, &PilatusThHumid0);
    createParam(PilatusThHumid1String,       asynParamFloat64, &PilatusThHumid1);
    createParam(PilatusThHumid2String,       asynParamFloat64, &PilatusThHumid2);
    createParam(PilatusTvxVersionString,     asynParamOctet,   &PilatusTvxVersion);

    /* Set some default values for parameters */
    status =  setStringParam (ADManufacturer, "Dectris");
    status |= setStringParam (ADModel, "Pilatus");
    status |= setIntegerParam(ADMaxSizeX, maxSizeX);
    status |= setIntegerParam(ADMaxSizeY, maxSizeY);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeX, maxSizeX);
    status |= setIntegerParam(ADSizeY, maxSizeY);
    status |= setIntegerParam(NDArraySizeX, maxSizeX);
    status |= setIntegerParam(NDArraySizeY, maxSizeY);
    status |= setIntegerParam(NDArraySize, 0);
    status |= setIntegerParam(NDDataType,  NDUInt32);
    status |= setIntegerParam(ADImageMode, ADImageContinuous);
    status |= setIntegerParam(ADTriggerMode, TMInternal);

    status |= setIntegerParam(PilatusArmed, 0);
    status |= setStringParam (PilatusBadPixelFile, "");
    status |= setIntegerParam(PilatusNumBadPixels, 0);
    status |= setStringParam (PilatusFlatFieldFile, "");
    status |= setIntegerParam(PilatusFlatFieldValid, 0);

    setDoubleParam(PilatusThTemp0, 0);
    setDoubleParam(PilatusThTemp1, 0);
    setDoubleParam(PilatusThTemp2, 0);
    setDoubleParam(PilatusThHumid0, 0);
    setDoubleParam(PilatusThHumid1, 0);
    setDoubleParam(PilatusThHumid2, 0);

    if (status) {
        printf("%s: unable to set camera parameters\n", functionName);
        return;
    }

    /*Read the TVX Version at startup.*/
    epicsSnprintf(this->toCamserver, sizeof(this->toCamserver), "version");
    status=writeReadCamserver(1.0);
    if (!status) {
      if ((substr = strstr(this->fromCamserver, "tvx")) != NULL) {
	setStringParam(PilatusTvxVersion, substr);
      }
    }
    
    /* Create the thread that updates the images */
    status = (epicsThreadCreate("PilatusDetTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)pilatusTaskC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for image task\n", 
            driverName, functionName);
        return;
    }

    /* Create the thread that monitors detector status (temperature, humidity, etc). */
    status = (epicsThreadCreate("PilatusStatusTask",
                                epicsThreadPriorityMedium,
                                epicsThreadGetStackSize(epicsThreadStackMedium),
                                (EPICSTHREADFUNC)pilatusStatusC,
                                this) == NULL);
    if (status) {
        printf("%s:%s epicsThreadCreate failure for status task\n", 
            driverName, functionName);
        return;
    }


}

/* Code for iocsh registration */
static const iocshArg pilatusDetectorConfigArg0 = {"Port name", iocshArgString};
static const iocshArg pilatusDetectorConfigArg1 = {"camserver port name", iocshArgString};
static const iocshArg pilatusDetectorConfigArg2 = {"maxSizeX", iocshArgInt};
static const iocshArg pilatusDetectorConfigArg3 = {"maxSizeY", iocshArgInt};
static const iocshArg pilatusDetectorConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg pilatusDetectorConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg pilatusDetectorConfigArg6 = {"priority", iocshArgInt};
static const iocshArg pilatusDetectorConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const pilatusDetectorConfigArgs[] =  {&pilatusDetectorConfigArg0,
                                                              &pilatusDetectorConfigArg1,
                                                              &pilatusDetectorConfigArg2,
                                                              &pilatusDetectorConfigArg3,
                                                              &pilatusDetectorConfigArg4,
                                                              &pilatusDetectorConfigArg5,
                                                              &pilatusDetectorConfigArg6,
                                                              &pilatusDetectorConfigArg7};
static const iocshFuncDef configPilatusDetector = {"pilatusDetectorConfig", 8, pilatusDetectorConfigArgs};
static void configPilatusDetectorCallFunc(const iocshArgBuf *args)
{
    pilatusDetectorConfig(args[0].sval, args[1].sval, args[2].ival,  args[3].ival,  
                          args[4].ival, args[5].ival, args[6].ival,  args[7].ival);
}


static void pilatusDetectorRegister(void)
{

    iocshRegister(&configPilatusDetector, configPilatusDetectorCallFunc);
}

extern "C" {
epicsExportRegistrar(pilatusDetectorRegister);
}

