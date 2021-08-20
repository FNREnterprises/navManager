
import time
import os
from datetime import datetime as dt
import inspect

import logging
import numpy as np
from enum import Enum
from collections import deque

from marvinglobal import marvinglobal as mg
from marvinglobal import environmentClasses
from marvinglobal import cartClasses
from marvinglobal import skeletonCommandMethods
from marvinglobal import cartCommandMethods

processName = 'navManager'
marvinShares = None   # shared data


# base folder for room information
PATH_ROOM_DATA = f"{mg.PERSISTED_DATA_FOLDER}/{mg.ROOM_FOLDER}"

# locally modifiable shared objects
roomDataLocal = environmentClasses.RoomData()
scanLocationListLocal = environmentClasses.ScanLocationList()
markerListLocal = environmentClasses.MarkerList()


skeletonCommandMethods = skeletonCommandMethods.SkeletonCommandMethods()
cartCommandMethods = cartCommandMethods.CartCommandMethods()

camRequest = {mg.CamTypes.EYE_CAM: None,
              mg.CamTypes.CART_CAM: None,
              mg.CamTypes.HEAD_CAM: None}

imageId = 0
fullScanDone = False

def updateSharedScanLocationList():
    msg = {'msgType': mg.SharedDataItems.ENVIRONMENT_SCAN_LOCATION, 'sender':processName,
           'info': scanLocationListLocal}
    updateSharedDict(msg)

def updateSharedDict(msg):
    #log(f"updateSharedDict, {msg=}")
    if not marvinShares.updateSharedData(msg):
        log(f"connection with shared data lost, going down") # connection to marvinData lost, try to reconnect
        os._exit(1)


#cams = {}       # dict of dict of cam properties received from cartControl

#eyecamImage = None
#cartcamImage = None
#depthcamImage = None
#headcamImage = None

# use threadProcessImages to look for markers or add depth information to map
#flagProcessCartcamImage = False
#flagProcessEyecamImage = False
#flagProcessHeadcamImage = False
#flagProcessDepthcamImage = False

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
         "getDepthImage",
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

leftArm = {'omoplate': 0, 'shoulder': 0, 'rotate': 0, 'bicep': 0}
oLeftArm = None

rightArm = {'omoplate': 0, 'shoulder': 0, 'rotate': 0, 'bicep': 0}
oRightArm = None

#head = {}
head =  None



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
_allowCartRotation = True
_cartcamImgId = 0
_cartcamImgReceived = False
_depthImgId = 0
_depthImgReceived = False
_eyecamImgId = 0
_eyecamImgReceived = False

batteryStatus = None

mapCenterColor = (255,255,255)
#markerColor = (255,255,0)   # yellow
targetColor = (0,255,0)     # green
##scanLocationColor = (128,255,128)   # light green

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


class Cart:
    cartX = 0
    cartY = 0
    cartYaw = 0
    xCorr = 0
    yCorr = 0
    platformImuYawCorrection = 0
    moving = False
    rotating = False
    blocked = False
    docked = False
    mapColor = (128,255,128)     # light green
    updateTime = time.time()

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

cart = Cart()     # only 1 cart object


class Head:
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
        thisYaw = (values[0] + self.headImuYawCorrection + cart.getCartYaw()) % 360
        if thisYaw < 180:
            self.headYaw = thisYaw
        else:
            self.headYaw = thisYaw - 360
        self.headRoll = values[1]
        self.headPitch = values[2]

    def getHeadYaw(self): return self.headYaw
    def getHeadRoll(self): return self.headRoll
    def getHeadPitch(self): return self.headPitch

head = Head()     # only 1 instance


class Target:
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


target = Target()     # only 1 target object


# a sequence of cart moves that can be interrupted and continued
moveSteps = None

"""
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
    markerYaw: int = None

    def props(self):
        pr = {}
        for name in dir(self):
            value = getattr(self, name)
            if not name.startswith('__') and not inspect.ismethod(value):
                pr[name] = value
        return pr
"""

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


# TODO convert to class
def createObjectViews():
    global oLeftArm, oRightArm
    oLeftArm = objectview(leftArm)
    oRightArm = objectview(rightArm)


def log(msg, publish=True):
    msg = f"navManager - " + msg
    if publish:
        print(f"{str(dt.now())[11:23]} {msg}")
    logging.info(msg)


def othersLog(msg):
    print(f"{str(dt.now())[11:23]} {msg}")
    logging.info(msg)




def setDockingMarkerPosition(location, degrees):
    
    global _dockingMarkerPosition

    _dockingMarkerPosition = [location, degrees]


def getDockingMarkerPosition():
    return _dockingMarkerPosition



def distDegToScanPos(index):
    """
    from the current cart position calc distance and degree to any other scan position
    """
    dx = scanLocationListLocal[index][0] - cart.getCartX()
    dy = scanLocationListLocal[index][1] - cart.getCartY()
    directionDegrees = np.degrees(np.arctan2(dx, dy))
    distance = np.hypot(dx, dy)
    return (directionDegrees, distance)

'''
def nextCartcamImgId():

    global _cartcamImgId

    _cartcamImgId += 1
    return _cartcamImgId

def nextEyecamImgId():

    global _eyecamImgId

    _eyecamImgId += 1
    return _eyecamImgId

def nextDepthImgId():

    global _depthImgId

    _depthImgId += 1
    return _depthImgId
'''

def allowCartRotation(newStatus):

    global _allowCartRotation

    if newStatus:       # allow rotation only when cartcam and depth image available
        if _cartcamImgReceived and _depthImgReceived:
            _allowCartRotation = True
    else:
        _allowCartRotation = False


def signedAngleDifference(start, end):
    """
    calculate angle difference in range -180 .. 180 between start and end degrees in range 0 .. 360
    """
    diff = end - start
    d = abs(diff) % 360
    value = 360 - d if d > 180 else d
    sign = 1 if (0 <= diff <= 180) or (-180 >= diff >= -360) else -1
    return int(round(sign * value))


def getCartLocation() -> mg.Location:
    return marvinShares.cartDict.get(mg.SharedDataItems.CART_LOCATION)

def getRoomData() -> environmentClasses.RoomData:
    return marvinShares.environmentDict.get(mg.SharedDataItems.ENVIRONMENT_ROOM)

def getScanLocationList() -> environmentClasses.ScanLocationList:
    return marvinShares.environmentDict.get(mg.SharedDataItems.ENVIRONMENT_SCAN_LOCATION_LIST)

def getMarkerList() -> environmentClasses.MarkerList:
    return marvinShares.environmentDict.get(mg.SharedDataItems.ENVIRONMENT_MARKER_LIST)
