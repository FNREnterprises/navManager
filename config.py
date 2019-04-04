
import time
from datetime import datetime as dt

import logging
import numpy as np
from enum import Enum
from collections import deque


# involved computers
marvin = "192.168.0.17"
pcjm = "192.168.0.14"

# rpc
MY_IP = pcjm
MY_RPC_PORT = 20010

# NOTE master for ip and ports is the taskOrchestrator
# rpc connection with task orchestrator
taskOrchestrator = None


# navMap
addPartialMap = False
addPartialMapDone = True

# use navMap thread to take a cart pic
takeCartcamImage = False
takeCartcamImageDone = False
takeCartcamImageSuccessful = False

# use navMap thread to take a depth pic
takeDepthImage = False
takeDepthImageDone = False
takeDepthImageSuccessful = False


floorPlan = None
floorPlanFat = None
obstacleMap = None
obstacleMapFat = None

fullScanPlan = None
fullScanPlanFat = None

scanLocations = []
markerList = []

robotMovesQueue = deque(maxlen=100)

#servoCurrent = {'assigned', 'moving', 'detached', 'position', 'degrees', servoName}
servoCurrent = {}


DOCKING_MARKER_ID = 10
DOCKING_DETAIL_ID = 11
KINECT_X_RANGE = 60        # degrees of view with Kinect 1, we need 6 cart rotations for full circle



# list of possible tasks
tasks = ["restartServers",
         "createFloorPlan",
         "fullScanAtPosition",
         "rover",
         "checkForPerson",
         "takeCartcamImage",
         "getDepthImage",
         "findMarker",
         "approachTarget",
         "dock",
         "moveCart",
         "stop",
         "restPosition",
         "cartMovePose",
         "queryCartInfo",
         "activateKinectPower",
         "exit"]


taskStack = []
task = None

target = {'x': 0, 'y': 0, 'orientation': 0, 'show': False}
oTarget = None

leftArm = {'omoplate': 0, 'shoulder': 0, 'rotate': 0, 'bicep': 0}
oLeftArm = None

rightArm = {'omoplate': 0, 'shoulder': 0, 'rotate': 0, 'bicep': 0}
oRightArm = None

head = {}
oHead =  None


# base folder for room information
PATH_ROOM_DATA = "D:/Projekte/InMoov/navManager/ROOMS"
room = 'unknown'
fullScanDone = False


# positions where a 360 view has been made and recorded
_dockingMarkerPosition = None    # position and orientation that showed the docking marker

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

cartColor = (255,255,0)     # cyan
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


class cCart:
    _x = 0
    _y = 0
    _o = 0
    xCorr = 0
    yCorr = 0
    oCorr = 0
    moving = False
    rotating = False
    blocked = False
    docked = False
    updateTime = time.time()

    def setX(self, x):
        self._x = int(x)

    def getX(self):
        return round(self._x + self.xCorr)

    def setY(self, y):
        self._y = int(y)

    def getY(self):
        return round(self._y + self.yCorr)

    def setYaw(self, o):
        self._o = int(o)

    def getYaw(self):
        return round(self._o + self.oCorr)

oCart = cCart()

class cMarker:
    def __init__(self):
        markerId = None
        cameraType = None
        camX = None
        camY = None
        camYaw = None
        angleInImage = None
        distanceCamToMarker = None
        markerX = None
        markerY = None
        markerYaw = None


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


class cServer:
    def __init__(self):
        simulated = False
        startupTime = None
        startRequested = None
        ip =  None
        port = None
        conn =  None
        lifeSignalRequest = None
        lifeSignalReceived = None
        connectionState = 'unknown'
        basicDataReceived = False

# Kinect must be started first, otherwise driver complains
servers = {'kinect': cServer(),
           'aruco':  cServer(),
           'servoControl': cServer(),
           'cartControl': cServer()
           }

def defineServers():

    server = 'kinect'
    servers[server].simulated = False
    servers[server].startupTime = 10
    servers[server].startRequested = None
    servers[server].ip =  marvin
    servers[server].port = 20003
    servers[server].conn =  None
    servers[server].lifeSignalRequest = time.time()
    servers[server].lifeSignalReceived = time.time()+1
    servers[server].connectionState = 'unknown'

    server = 'aruco'
    servers[server].simulated = False
    servers[server].startupTime = 10
    servers[server].startRequested = None
    servers[server].ip =  marvin
    servers[server].port = 20002
    servers[server].conn =  None
    servers[server].lifeSignalRequest = time.time()
    servers[server].lifeSignalReceived = time.time()+1
    servers[server].connectionState = 'unknown'

    server = 'servoControl'
    servers[server].simulated = False
    servers[server].startupTime = 10
    servers[server].startRequested = None
    servers[server].ip =  marvin
    servers[server].port = 20004
    servers[server].conn =  None
    servers[server].lifeSignalRequest = time.time()
    servers[server].lifeSignalReceived = time.time()+1
    servers[server].connectionState = 'unknown'
    servers[server].basicDataReceived = False

    server = 'cartControl'
    servers[server].simulated = False
    servers[server].startupTime = 10
    servers[server].startRequested = None
    servers[server].ip =  marvin
    servers[server].port = 20001
    servers[server].conn =  None
    servers[server].lifeSignalRequest = time.time()
    servers[server].lifeSignalReceived = time.time()+1
    servers[server].connectionState = 'unknown'
    servers[server].basicDataReceived = False

def createObjectViews():
    global oTarget, oLeftArm, oRightArm, oHead
    oTarget = objectview(target)
    oLeftArm = objectview(leftArm)
    oRightArm = objectview(rightArm)
    oHead = objectview(head)

def log(msg, publish=True):
    msg = f"navManager - " + msg
    if publish:
        print(f"{str(dt.now())[11:22]} {msg}")
    logging.info(msg)


def othersLog(msg):
    print(f"{str(dt.now())[11:22]} {msg}")
    logging.info(msg)




def setDockingMarkerPosition(location, orientation):
    
    global _dockingMarkerPosition

    _dockingMarkerPosition = [location, orientation]


def getDockingMarkerPosition():
    return _dockingMarkerPosition



def distDegToScanPos(index):
    '''
    from the current cart position calc distance and degree to any other scan position
    '''
    dx = scanLocations[index][0] - oCart.getX()
    dy = scanLocations[index][1] - oCart.getY()
    directionDegrees = np.degrees(np.arctan2(dx, dy))
    distance = np.hypot(dx, dy)
    return (directionDegrees, distance)


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


def allowCartRotation(newStatus):

    global _allowCartRotation

    if newStatus:       # allow rotation only when cartcam and depth image available
        if _cartcamImgReceived and _depthImgReceived:
            _allowCartRotation = True
    else:
        _allowCartRotation = False


#def setCartDocked(newStatus):

#    global _cartDocked

#    _cartDocked = newStatus


#def isCartDocked():
#    return _cartDocked




