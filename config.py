
import time
from datetime import datetime as dt

import logging
import numpy as np
import cv2
from collections import deque

import navMap

# involved computers
marvin = "192.168.0.17"
pcjm = "192.168.0.14"

# rpc
MY_IP = pcjm
MY_PORT = 20010

# NOTE master for ip and ports is the taskOrchestrator
# rpc connection with task orchestrator
taskOrchestrator = None

# Kinect must be started first, otherwise driver complains
navManagerServers = { 'kinect':       {'simulated': False, 'startupTime': 15, 'startRequested': None, 'ip': marvin, 'port': 20003, 'conn': None,
                                       'lifeSignalRequest': time.time(), 'lifeSignalReceived': time.time()+1},
                      'aruco':        {'simulated': False, 'startupTime': 15, 'startRequested': None, 'ip': marvin, 'port': 20002, 'conn': None,
                                       'lifeSignalRequest': time.time(), 'lifeSignalReceived': time.time()+1},
                      'servoControl': {'simulated': False, 'startupTime': 25, 'startRequested': None, 'ip': marvin, 'port': 20004, 'conn': None,
                                       'lifeSignalRequest': time.time(), 'lifeSignalReceived': time.time()+1},
                      'cartControl':  {'simulated': False, 'startupTime': 20, 'startRequested': None, 'ip': marvin, 'port': 20001, 'conn': None,
                                       'lifeSignalRequest': time.time(), 'lifeSignalReceived': time.time()+1}
                      }


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

scanLocations = []
markerInfo = []

robotMovesQueue = deque(maxlen=100)

#servoCurrent = {'assigned', 'moving', 'detached', 'position', 'degrees', servoName}
servoCurrent = {}


DOCKING_MARKER_ID = 10
DOCKING_DETAIL_ID = 11
KINECT_X_RANGE = 60        # degrees of view with Kinect 1, we need 6 cart rotations for full circle
DISTANCE_CART_CENTER_CAM = 330 
MARKER_XOFFSET_CORRECTION = -15     # distance docking cave - docking detail marker might not be equal to distance docking fingers - cartcam
DOCKING_START_DISTANCE_FROM_MARKER = 600

# list of possible tasks
tasks = ["restartServers",
         "createFloorPlan",
         "fullScanAtPosition",
         "rover",
         "showEyecamImage",
         "showCartcamImage",
         "getDepthImage",
         "findMarker",
         "approachTarget",
         "dock",
         "stop",
         "moveForward50",
         "moveBackward50",
         "moveHead",
         "setPin41",
         "clearPin41",
         "exit"]


taskStack = []
task = None

#cartLocation = Point2D(0, 0)                #
#cartOrientation = 0

cartInfo = {'x': 0, 'y': 0, 'orientation': 0, 'moving:': False, 'rotating': False, 'blocked': False, 'docked': False, 'updateTime': time.time()}
oCart = None

# navManager local cart position correction, gets set by fullScanAtPosition and reset after sending it to the cart
cartPositionCorrection = {'x': 0, 'y': 0, 'o': 0}

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
fullScanResult = np.zeros((navMap.MAP_WIDTH, navMap.MAP_HEIGHT), dtype=np.uint8)

# positions where a 360 view has been made and recorded
_dockingMarkerPosition = None    # position and orientation that showed the docking marker

_arucoMarkers = []
emptyMarkerInfo = {'markerId': 0, 'distance' : 0, 'markerAngleInImage': 0, 'markerYawDegrees': 0}
#markerInfoRec = Record.create_type('markerInfo', 'id', 'distance', 'yawToMarker', 'yawToCartTarget', 'distToCartTarget')
#markerInfo = markerInfoRec(None,0,0,0,0)


EYE_X_CORR = -5     # Offset Auge zum Bild-Zentrum (negativ=Auge hat rechtsdrall)


# Variables for scanning environment
_allowCartRotation = True
_cartcamImgId = 0
_cartcamImgReceived = False
_depthImgId = 0
_depthImgReceived = False
_eyecamImgId = 0
_eyecamImgReceived = False

batteryStatus = None

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
    '''
    usage:
    try: 
       raise(ArucoError, <markerId>)
   except ArucoError as error:
        print('aruco exception raised: ', error.value
    '''
 
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


def createObjectViews():
    global oCart, oTarget
    oCart = objectview(cartInfo)
    oTarget = objectview(target)
    oLeftArm = objectview(leftArm)
    oRightArm = objectview(rightArm)
    oHead = objectview(head)

def log(msg, publish=True):
    msg = f"navManager - " + msg
    if publish:
        print(f"{dt.now()} {msg}")
    logging.info(msg)


def othersLog(msg):
    print(f"{dt.now()} {msg}")
    logging.info(msg)



def setTask(newTask):

    global taskStack, task

    # we can have a stack of started tasks and can return to the last requested task
    if newTask == "pop":

        #log(f"taskStack before pop: {taskStack}")

        # if we have no stacked tasks set task to notask
        if not taskStack:
            newTask = "notask"
        else:
            taskStack.pop()
            newTask = taskStack[-1]
            log(f"newTask: {newTask}, taskStack after pop: {taskStack}")
            return

    # if we have run into a problem or successfully finished all open tasks we have notask
    if newTask == "notask":
        taskStack = []
    else:
        taskStack.append(newTask)

    task = newTask

    if task != "notask":
        print()
        log(f"new task requested: {newTask}")

    if len(taskStack) > 0:
        log(f"taskStack: {taskStack}")


def addArrow(img, mapX, mapY, orientation, length, color=(128,128,128)):

    arrowXCorr = int(length * np.cos(np.radians(90 + orientation)))
    arrowYCorr = int(length * np.sin(np.radians(90 + orientation)))
    cv2.circle(img, (mapX,mapY), 3, color, -1)
    cv2.arrowedLine(img, (mapX, mapY),
                        (mapX + arrowXCorr, mapY - arrowYCorr), color, 1, tipLength=0.3)  # mark cart orientation
    return img


def setDockingMarkerPosition(location, orientation):
    
    global _dockingMarkerPosition

    _dockingMarkerPosition = [location, orientation]


def getDockingMarkerPosition():
    return _dockingMarkerPosition



def distDegToScanPos(index):
    '''
    from the current cart position calc distance and degree to any other scan position
    '''
    dx = scanLocations[index][0] - oCart.x
    dy = scanLocations[index][1] - oCart.y
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


def clearMarkerList():
    
    global _arucoMarkers

    _arucoMarkers = []


def addArucoMarkers(markerList):

    global _arucoMarkers

    for m in markerList:
        for indexN, n in enumerate(_arucoMarkers):
            if m["markerId"] == n["markerId"]:
                del _arucoMarkers[indexN]

        _arucoMarkers.append(m)


def getArucoMarkerInfo(markerId):
    for m in _arucoMarkers:
        if m["markerId"] == markerId:
            return m

    return {}



