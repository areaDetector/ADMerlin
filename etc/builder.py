#Minimal IOC Builder definition file for medipix area detector

from iocbuilder import Device, AutoSubstitution, SetSimulation
from iocbuilder.modules.asyn import Asyn, AsynIP
from iocbuilder.modules.areaDetector import AreaDetector, _NDFile, _ADBase
from iocbuilder.arginfo import *

class _medipix(AutoSubstitution):
    TemplateFile = "medipix.template"
    SubstitutionOverwrites = [_NDFile]

class medipix(_ADBase):
    """Creates a medipix areaDetector driver"""
    _SpecificTemplate = _medipix
    def __init__(self, LABVIEW_CMD = "localhost:14000", LABVIEW_DATA = "localhost:14001", XSIZE = 256, YSIZE = 256,
            DETECTOR_TYPE = 0, TRACING = 0x1, BUFFERS = 50, MEMORY = -1, **args):
                
        # Make an asyn IP ports to talk to lab view
        self.LABVIEW_CMD_PORT = args["PORT"] + "cmd"
        self.LABVIEW_DATA_PORT = args["PORT"] + "data"
        self.cmd = AsynIP(LABVIEW_CMD, name = self.LABVIEW_CMD_PORT)
        self.data = AsynIP(LABVIEW_DATA, name = self.LABVIEW_DATA_PORT)
        
        # Init the superclass
        self.__super.__init__(XSIZE=XSIZE, YSIZE=YSIZE, **args) 
        # Init the file writing class
        self.file = _NDFile(**filter_dict(args, _NDFile.ArgInfo.Names()))
        # Store the args
        self.__dict__.update(self.file.args)
#        self.__dict__.update(locals())
        self.LABVIEW_CMD = LABVIEW_CMD
        self.LABVIEW_DATA = LABVIEW_DATA
        self.XSIZE = XSIZE
        self.YSIZE = YSIZE
        self.BUFFERS = BUFFERS
        self.MEMORY = MEMORY
        self.DETECTOR_TYPE = DETECTOR_TYPE
        self.TRACING = TRACING

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _NDFile.ArgInfo + \
            _SpecificTemplate.ArgInfo.filtered(without = ["LABVIEW_PORT"]) + \
            makeArgInfo(__init__,
        LABVIEW_CMD = Simple('port for medipix Labview command channel'),
        LABVIEW_DATA = Simple('port for medipix Labview data channel'),
        XSIZE = Simple('Maximum X dimension of the image', int),
        YSIZE = Simple('Maximum Y dimension of the image', int),
        DETECTOR_TYPE = Enum ('Detector Type', ("Merlin", "Medipix_XBPM", "UoM_BPM", "QuadMerlin")),
        TRACING = Simple('Tracing flag - use 0x301 for verbose and 0x101 for less verbose'),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))

    # Device attributes
#    LibFileList = ['medipixDetector', 'cbfad']
    LibFileList = ['medipixDetector']
    DbdFileList = ['medipix']

    def Initialise(self):
        print '# medipixDetectorConfig(portName, commandPort, dataPort, maxSizeX, ' \
            'maxSizeY, ,detectorType maxBuffers, maxMemory)' \
            'detectorType: 0 = Merlin, 1 = Medipix_XBPM, 2 = UoM_BPM, 3 = Quad Merlin'
        print 'medipixDetectorConfig("%(PORT)s", "%(LABVIEW_CMD_PORT)s", "%(LABVIEW_DATA_PORT)s", ' \
            '%(XSIZE)d, %(YSIZE)d, %(DETECTOR_TYPE)d, %(BUFFERS)d, %(MEMORY)d)' % self.__dict__
        
        print '# driver specific TRACING '
        print '#  0x101 = less verbose, 0x301 = most verbose, 0x01 = errors only'
        print 'asynSetTraceMask %s 0 %s'%(self.PORT, self.TRACING)

#def medipix_sim(**kwargs):
#    return simDetector(2500, 2000, **kwargs)

# SetSimulation(medipix, medipix_sim)
