/*
 * medipixDetector.h
 *
 *  Created on: 16 Oct 2013
 *      Author: hgv27681
 */

#ifndef MEDIPIXDETECTOR_H_
#define MEDIPIXDETECTOR_H_

/** Messages to/from Labview command channel */
#define MAX_MESSAGE_SIZE 256
#define MAX_FILENAME_LEN 256
#define MAX_BAD_PIXELS 100
/** Time to poll when reading from Labview */
#define ASYN_POLL_TIME .01
#define Labview_DEFAULT_TIMEOUT 2.0
/** Time between checking to see if image file is complete */
#define FILE_READ_DELAY .01

#define DIMS 2

/** Detector Types */
typedef enum
{
    Merlin, MedipixXBPM, UomXBPM
} medipixDetectorType;

/** Trigger modes */
typedef enum
{
    TMInternal,
    TMExternalEnable,
    TMExternalTriggerHigh,
    TMExternalTriggerLow,
    TMExternalTriggerRising,
    TMSoftwareTrigger
} medipixTriggerMode;/** Trigger modes */

/** Medipix Individual Trigger types */

#define TMTrigInternal  "0"
#define TMTrigRising    "1"
#define TMTrigFalling   "2"
#define TMTrigSoftware  "3"

/** ASYN PARAMETER NAMES **/

#define medipixDelayTimeString              "DELAY_TIME"
#define medipixThreshold0String             "THRESHOLD0"
#define medipixThreshold1String             "THRESHOLD1"
#define medipixOperatingEnergyString        "OPERATINGENERGY"

#define medipixThresholdApplyString         "THRESHOLD_APPLY"
#define medipixThresholdAutoApplyString     "THRESHOLD_AUTO_APPLY"
#define medipixArmedString                  "ARMED"

#define medipixmedpixThresholdScanString    "THRESHOLDSCAN"
#define medipixStartThresholdScanString     "THRESHOLDSTART"
#define medipixStopThresholdScanString      "THRESHOLDSTOP"
#define medipixStepThresholdScanString      "THRESHOLDSTEP"
#define medipixStartThresholdScanningString "STARTTHRESHOLDSCANNING"
#define medipixCounterDepthString           "COUNTERDEPTH"
#define medipixResetString                  "RESET"
#define medipixSoftwareTriggerString        "SOFTWARETRIGGER"
#define medipixEnableCounter1String         "ENABLECOUNTER1"
#define medipixContinuousRWString           "CONTINUOUSRW"
#define medipixProfileControlString         "PROFILECONTROL"
#define medipixProfileXString               "PROFILE_AVERAGE_X"
#define medipixProfileYString               "PROFILE_AVERAGE_Y"

class mpxConnection;

/** Driver for Dectris medipix pixel array detectors using their Labview server over TCP/IP socket */
class medipixDetector: public ADDriver
{
public:
    medipixDetector(const char *portName, const char *LabviewCmdPort,
            const char *LabviewDataPort, int maxSizeX, int maxSizeY,
            int detectorType, int maxBuffers, size_t maxMemory, int priority,
            int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value,
            size_t nChars, size_t *nActual);
    void report(FILE *fp, int details);
    void medipixTask(); /* This should be private but is called from C so must be public */
    void medipixStatus(); /* This should be private but is called from C so must be public */

    void fromLabViewStr(const char *str);
    void toLabViewStr(const char *str);

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
    int medipixSoftwareTrigger;
    int medipixReset;
    int medipixEnableCounter1;
    int medipixContinuousRW;
    int medipixProfileControl;
    int medipixProfileX;
    int medipixProfileY;

#define LAST_medipix_PARAM medipixProfileY

private:
    /* These are the methods that are new to this class */
    void abortAcquisition();
    asynStatus setModeCommands(int function);
    asynStatus setAcquireParams();
    asynStatus getThreshold();
    asynStatus updateThresholdScanParms();
    asynStatus setROI();

    NDArray* copyProfileToNDArray32(size_t *dims, char *buffer,
            int profileMask);
    NDArray* copyToNDArray16(size_t *dims, char *buffer);
    NDArray* copyToNDArray32(size_t *dims, char *buffer);
    inline void endian_swap(unsigned short& x);
    inline void endian_swap(unsigned int& x);
    inline void endian_swap(uint64_t& x);
    unsigned int maxSize[2];

    /* Our data */
    int imagesRemaining;
    NDArray *pFlatField;
    int multipleFileNumber;
    asynUser *pasynLabViewCmd;
    asynUser *pasynLabViewData;
    double averageFlatField;

    int *profileX;
    int *profileY;

    bool startingUp;  // used to avoid very chatty initialisation

    char LabviewCommandPortName[20];
    char LabviewDataPortName[20];

    medipixDetectorType detType;

    mpxConnection *cmdConnection;
    mpxConnection *dataConnection;
};

#define NUM_medipix_PARAMS (&LAST_medipix_PARAM - &FIRST_medipix_PARAM + 1)

static const char *driverName = "medipixDetector";

#endif /* MEDIPIXDETECTOR_H_ */
