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
    def __init__(self, LABVIEW = "localhost:41234", XSIZE = 1024, YSIZE = 768,
            BUFFERS = 50, MEMORY = -1, **args):
        # Make an asyn IP port to talk to medipixdtb on
        self.LABVIEW_PORT = args["PORT"] + "ip"
        self.ip = AsynIP(LABVIEW, name = self.LABVIEW_PORT,
            input_eos = "\030", output_eos = "\n")
        # Init the superclass
        self.__super.__init__(**args)
        # Init the file writing class
        self.file = _NDFile(**filter_dict(args, _NDFile.ArgInfo.Names()))
        # Store the args
        self.__dict__.update(self.file.args)
#        self.__dict__.update(locals())
        self.LABVIEW = LABVIEW
        self.XSIZE = XSIZE
        self.YSIZE = YSIZE
        self.BUFFERS = BUFFERS
        self.MEMORY = MEMORY

    # __init__ arguments
    ArgInfo = _ADBase.ArgInfo + _NDFile.ArgInfo + \
            _SpecificTemplate.ArgInfo.filtered(without = ["LABVIEW_PORT"]) + \
            makeArgInfo(__init__,
        LABVIEW = Simple('Machine:port that medipix LABVIEW is running on'),
        XSIZE = Simple('Maximum X dimension of the image', int),
        YSIZE = Simple('Maximum Y dimension of the image', int),
        BUFFERS = Simple('Maximum number of NDArray buffers to be created for '
            'plugin callbacks', int),
        MEMORY = Simple('Max memory to allocate, should be maxw*maxh*nbuffer '
            'for driver and all attached plugins', int))

    # Device attributes
#    LibFileList = ['medipixDetector', 'cbfad']
    LibFileList = ['medipixDetector']
    DbdFileList = ['medipix']

    def Initialise(self):
        print '# medipixDetectorConfig(portName, serverPort, maxSizeX, ' \
            'maxSizeY, maxBuffers, maxMemory)'
        print 'medipixDetectorConfig("%(PORT)s", "%(LABVIEW_PORT)s", ' \
            '%(XSIZE)d, %(YSIZE)d, %(BUFFERS)d, %(MEMORY)d)' % self.__dict__

#def medipix_sim(**kwargs):
#    return simDetector(2500, 2000, **kwargs)

# SetSimulation(medipix, medipix_sim)
