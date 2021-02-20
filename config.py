
import time
from datetime import datetime as dt
import inspect
import os
import logging
from enum import Enum
from collections import deque
from typing import List

from marvinglobal import skeletonCommandMethods
from marvinglobal import cartCommandMethods

processName = "navManager"
marvinShares = None   # shared data

obstacleDistances = None        # for each cm slice in front of the robot the distance to an obstacle

processSimulated = []
skeletonCommandMethods = skeletonCommandMethods.SkeletonCommandMethods()
cartCommandMethods = cartCommandMethods.CartCommandMethods()

waitForArucoEyeCam:bool = False
waitForArucoCartCam:bool = False

lookForMarkers = []


# use threadProcessImages to look for markers or add depth information to map
flagProcessCartcamImage = False
flagProcessEyecamImage = False
flagProcessHeadcamImage = False
flagAddObstaclesToMap = False

# temporary values for addPartialMap as it runs in separate thread
depthCamDistances = None
depthCamDegrees = 0
depthCamX = None
depthCamY = None

floorPlan = None
floorPlanFat = None
obstacleMap = None
obstacleMapFat = None

fullScanPlan = None
fullScanPlanFat = None

mapSettings = {
    'showCart':         True,
    'showScanLocation': False,
    'showTarget':       False,
    'showMarkers':      False,
    'showMovePath':     False }

scanLocations = []
markerList = []

# shared queues
taskRequestQueue = None
taskRespondQueue = None

robotMovesQueue = deque(maxlen=100)

#servoCurrent = {'assigned', 'moving', 'detached', 'position', 'degrees', servoName}
servoCurrent = {}

DOCKING_MARKER_ID = 10
DOCKING_DETAIL_ID = 11


# list of possible tasks
tasks = ["restartServers",
         "createFloorPlan",
         "fullScanAtPosition",
         "rover",
         "checkForPerson",
         "flagTakeCartcamImage",
         "takeEyecamImage",
         "takeHeadcamImage",
         "findObstacles",
         "dock",
         "moveCart",
         "stop",
         "restPosition",
         "cartMovePose",
         "queryCartInfo",
         "requestD415Depth",
         "requestHeadOrientation",
         "exit"]


taskStack = []
task = None
prevTask = None

head = {}
oHead =  None


# base folder for room information
PATH_ROOM_DATA = "D:/Projekte/InMoov/navManager/ROOMS"
room = 'unknown'
fullScanDone = False


# positions where a 360 view has been made and recorded
_dockingMarkerPosition = None    # position and degrees that showed the docking marker

_arucoMarkers = []
# a list of markers of type cMarker

EYE_CAM_HORIZONTAL_ANGLE = 60
EYE_CAM_COLS = 640
EYE_X_CORR = -5     # Offset Auge zum Bild-Zentrum (negativ=Auge hat rechtsdrall)

CART_CAM_HORIZONTAL_ANGLE = 60
CART_CAM_COLS = 640

# Variables for scanning environment
_cartcamImgId = 0
_cartcamImgReceived = False
_depthImgId = 0
_depthImgReceived = False
_eyecamImgId = 0
_eyecamImgReceived = False

batteryStatus = None

mapCenterColor = (255,255,255)
markerColor = (255,255,0)   # yellow
targetColor = (0,255,0)     # green
scanLocationColor = (128,255,128)   # light green

# tensorflow nets for people locator, age and gender classification
netsLoaded = False

class Direction(Enum):
    STOP = 0
    FORWARD = 1
    FOR_DIAG_RIGHT = 2
    FOR_DIAG_LEFT = 3
    LEFT = 4
    RIGHT = 5
    BACKWARD = 6
    BACK_DIAG_RIGHT = 7
    BACK_DIAG_LEFT = 8
    ROTATE_LEFT = 9
    ROTATE_RIGHT = 10

class cArm:     #leftArm = {'omoplate': 0, 'shoulder': 0, 'rotate': 0, 'bicep': 0}
    def __init__(self):
        self.omoplate = 0
        self.shoulder = 0
        self.rotate = 0
        self.bicep = 0

oLeftArm = cArm()
oRightArm = cArm()


class cCart:
    def __init__(self):
        self.cartX = 0
        self.cartY = 0
        self.cartYaw = 0
        self.xCorr = 0
        self.yCorr = 0
        self.platformImuYawCorrection = 0
        self.moving = False
        self.rotating = False
        self.blocked = False
        self.headYaw = 0
        self.headPitch = 0
        self.docked = False
        self.mapColor = (128,255,128)     # light green
        self.updateTime = time.time()

    def setCartX(self, x):
        self.cartX = int(x)

    def setCartY(self, y):
        self.cartY = int(y)

    def setCartYaw(self, degrees):
        self.cartYaw = int(degrees)

    def getCartX(self):
        return int(round(self.cartX + self.xCorr))

    def getCartY(self):
        return int(round(self.cartY + self.yCorr))

    def getCartYaw(self):
        return int(round(self.cartYaw + self.platformImuYawCorrection))

oCart = cCart()     # only 1 cart object


class cHead:
    isHeadImuCalibrated = False     # needs cart and servo connection
    isHeadImuCalibrationInitialized = False     # needs cart and servo connection
    headImuYawCorrection = 0
    headYaw = 0
    headRoll = 0
    headPitch = 0

    def startHeadImuCalibration(self):
        self.isHeadImuCalibrationInitialized = True

    def applyHeadImuYawCalibration(self, offset):
        self.headImuYawCorrection = -offset
        self.isHeadImuCalibrated = True
        self.isHeadImuCalibrationInitialized = False

    def setHeadOrientation(self, values):
        thisYaw = (values[0] + self.headImuYawCorrection + oCart.getCartYaw()) % 360
        if thisYaw < 180:
            self.headYaw = thisYaw
        else:
            self.headYaw = thisYaw - 360
        self.headRoll = values[1]
        self.headPitch = values[2]

    def getHeadYaw(self): return self.headYaw
    def getHeadRoll(self): return self.headRoll
    def getHeadPitch(self): return self.headPitch

oHead = cHead()     # only 1 instance


class cTarget:
    _x : int = 0
    _y : int = 0
    mapColor = (0,255,255)     # yellow

    def setX(self, x):
        self._x = int(round(x))

    def setY(self, y):
        self._y = int(round(y))

    def getX(self):
        return self._x

    def getY(self):
        return self._y


oTarget = cTarget()     # only 1 target object


# a sequence of cart moves that can be interrupted and continued
oMoveSteps = None

from dataslots import with_slots
from dataclasses import dataclass

@with_slots     # prevents adding dynamic attributes to class
@dataclass
class cMarker:
    markerId: int = None
    cameraType: str = None
    cartX: int = None
    cartY: int = None
    cartDegrees: int = None
    camDegrees: int = None
    atAngleFromCart: int = None
    distanceCamToMarker: int = None
    markerX: int = None
    markerY: int = None
    markerDegrees: int = None

    def props(self):
        pr = {}
        for name in dir(self):
            value = getattr(self, name)
            if not name.startswith('__') and not inspect.ismethod(value):
                pr[name] = value
        return pr


class CartError(Exception):
    """
      usage:
    try: 
       raise(CartError, "rotation"
    except CartError as error:
        print('cart exception raised: ', error.value
    """
 
    # Constructor or Initializer
    def __init__(self, value):
        self.value = value
 
    # __str__ is to print() the value
    def __str__(self):
        return(repr(self.value))


class ArucoError(Exception):
    """
    usage:
    try:
       raise(ArucoError, <markerId>)
   except ArucoError as error:
        print('aruco exception raised: ', error.value
    """
 
    # Constructor or Initializer
    def __init__(self, value):
        self.value = value
 
    # __str__ is to print() the value
    def __str__(self):
        return(repr(self.value))


class objectview(object):
    """
    allows attibs of a dict to be accessed with dot notation
    e.g.
    mydict={'a':1,'b':2}
    oMydict = objectview(mydict)
    then instead of mydict['a'] we can write mydict.a
    """
    def __init__(self, d):
        self.__dict__ = d



def initLogging():
    # Logging, renaming old logs for reviewing ...
    baseName = "log/navManager"
    oldName = f"{baseName}9.log"
    if os.path.isfile(oldName):
        os.remove(oldName)
    for i in reversed(range(9)):
        oldName = f"{baseName}{i}.log"
        newName = f"{baseName}{i+1}.log"
        if os.path.isfile(oldName):
            os.rename(oldName, newName)
    oldName = f"{baseName}.log"
    newName = f"{baseName}0.log"
    if os.path.isfile(oldName):
        try:
            os.rename(oldName, newName)
        except Exception as e:
            log(f"can not rename {oldName} to {newName}")

    logging.basicConfig(
        filename="log/navManager.log",
        level=logging.INFO,
        format='%(asctime)s - %(message)s',
        filemode="w")


def log(msg, publish=True):
    msg = f"navManager - " + msg
    if publish:
        print(f"{str(dt.now())[11:22]} {msg}")
    logging.info(msg)


def othersLog(msg):
    print(f"{str(dt.now())[11:22]} {msg}")
    logging.info(msg)

