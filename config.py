
import time
import os
import sys
import logging
import rpyc
import xmlrpc.client

from pyrecord import Record
import numpy as np

import simplejson as json
import rpcReceive
import rpcSend

Point2D = Record.create_type("Point2D","x","y")


DOCKING_MARKER_ID = 10
DOCKING_DETAIL_ID = 11
KINECT_X_RANGE = 60        # degrees of view with Kinect 1, we need 6 cart rotations for full circle
DISTANCE_CART_CENTER_CAM = 330 
MARKER_XOFFSET_CORRECTION = -15     # distance docking cave - docking detail marker might not be equal to distance docking fingers - cartcam
DOCKING_START_DISTANCE_FROM_MARKER = 600

# list of possible tasks
tasks = ["restartTasks",
         "createFloorPlan",
         "showEyecamImage",
         "showCartcamImage",
         "showDepthImage",
         "findMarker",
         "approachTarget",
         "rover",
         "dock",
         "stop"]

#_task = "findMarker"
_taskStack = []

cartLocation = Point2D(0, 0)                #
cartOrientation = 0

cartMoving = False
cartRotating = False
_cartBlocked = False
_cartDocked = False

_targetLocation = Point2D(0,0)
_targetOrientation = 0

# base folder for room information
PATH_ROOM_DATA = "D:/Projekte/InMoov/navManager/ROOMS"
_room = 'unknown'
_fullScanDone = False

# positions where a 360 view has been made and recorded
_scanLocations = []
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

SERVER_WATCH_INTERVAL = 5

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


class CartControlError(Exception):
    """
      usage:
    try:
       raise(CartControlError, "move"
    except CartControlError as error:
        print('cart exception raised: ', error.value
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


def log(msg):
    msg = "navManager - " + msg
    print(msg)
    logging.info(msg)


def othersLog(msg):
    print(msg)
    logging.info(msg)


def taskSimulated(server):
    return rpcSend.navManagerServers[server]['simulated']

  
def setDockingMarkerPosition(location, orientation):
    
    global _dockingMarkerPosition

    _dockingMarkerPosition = [location, orientation]


def getDockingMarkerPosition():
    return _dockingMarkerPosition


def getRoomScanCompleted():
    return True


def distDegToScanPos(index):
    '''
    from the current cart position calc distance and degree to any other scan position
    '''
    dx = _scanLocations[index][0] - cartLocation.x
    dy = _scanLocations[index][1] - cartLocation.y
    directionDegrees = np.degrees(np.arctan2(dx, dy))
    distance = np.hypot(dx, dy)
    return (directionDegrees, distance)


def setCartOrientation(newOrientation):

    global cartOrientation

    cartOrientation = newOrientation


def getCartOrientation():
    if not rpcSend.navManagerServers['cartControl']['simulated']:
        rpcSend.navManagerServers['cartControl']['conn'].root.exposed_requestCartOrientation()       # query cart first as we experienced offsets between cart and navManager
    return cartOrientation


def setCartLocation(posX, posY):

    global cartLocation

    cartLocation.x = posX
    cartLocation.y = posY


def getCartLocation():
    return cartLocation.x, cartLocation.y


def setCartInfo(newCartInfo):

    global cartOrientation, cartLocation, cartMoving, cartRotating

    cartOrientation, cartLocation.x, cartLocation.y, cartMoving, cartRotating = newCartInfo


def isCartRotating():
    '''
    get current status from cart and update navManager value
    '''
    global cartRotating

    cartRotating = rpcSend.navManagerServers['cartControl']['conn'].root.exposed_isCartRotating()
    return cartRotating


def isCartMoving():

    global cartMoving

    cartMoving = rpcSend.navManagerServers['cartControl']['conn'].root.exposed_isCartMoving()
    return cartMoving


def setTargetOrientation(degree):

    global _targetOrientation

    _targetOrientation = degree


def getTargetOrientation():
    return _targetOrientation


def getRemainingRotation():
    a = _targetOrientation - cartOrientation
    a = (a + 180) % 360 - 180
    log(f".getRemainingRotation: targetOrientation {_targetOrientation:.0f}, cartOrientation: {cartOrientation:.0f}, diff: {a:.0f}")
    return a


def getRemainingDistance(distance):
    remaining = np.hypot(_targetLocation.x - cartLocation.x, _targetLocation.y - cartLocation.y)
    #log(f"requested distance {distance}, remaining distance: {remaining}")
    return remaining




def setTask(newTask):

    global _taskStack, _task

    # we can have a stack of started tasks and can return to the last requested task    
    if newTask == "pop":

        # if we have no stacked tasks set task to notask
        if not _taskStack:
            newTask = "notask"
        else:
            # try to continue previous task
            newTask = _taskStack.pop()
            log(f"task popped back to {newTask}")
            return

    # if we have run into a problem or successfully finished all open tasks we have notask
    if newTask == "notask":
        _taskStack = []
    else:
        _taskStack.append(newTask)

    _task = newTask
    log(f"new task requested: {newTask}")


def getTask():
    return _task


def setCartBlocked(yesNo):

    global _cartBlocked

    _cartBlocked = yesNo


def getCartBlocked():
    return _cartBlocked


def getTargetLocation():
    return _targetLocation


def evalTargetPos(distance, mapDegree):

    global _targetLocation
    trigDegree = (mapDegree + 90) % 360
    dx = distance * np.cos(np.radians(trigDegree))
    dy = distance * np.sin(np.radians(trigDegree))
    _targetLocation.x = cartLocation.x + dx
    _targetLocation.y = cartLocation.y - dy
    return _targetLocation


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


def setCartcamImgReceived(trueFalse):

    global _cartcamImgReceived

    _cartcamImgReceived = trueFalse


def getCartcamImgReceived():
    return _cartcamImgReceived


def allowCartRotation(newStatus):

    global _allowCartRotation

    if newStatus:       # allow rotation only when cartcam and depth image available
        if _cartcamImgReceived and _depthImgReceived:
            _allowCartRotation = True
    else:
        _allowCartRotation = False


def setCartDocked(newStatus):

    global _cartDocked

    _cartDocked = newStatus


def isCartDocked():
    return _cartDocked


def saveMapInfo():
    # Saving the objects:
    mapInfo = { 'room':_room, 'fullScanDone':_fullScanDone }
    filename = f"{PATH_ROOM_DATA}/mapInfo.json"
    with open(filename, "w") as write_file:
        json.dump(mapInfo, write_file)


def loadMapInfo():
    '''
    the cartControl task keeps track of position and location. this is necessary because the cart
    can also be moved by directly using the cartControl interface without connection to the navManager
    '''
    global _room, _fullScanDone

    # Getting back the map data:
    filename = f"{PATH_ROOM_DATA}/mapInfo.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            mapInfo = json.load(read_file)

        _room = mapInfo['room']
        _fullScanDone = mapInfo['fullScanDone']

    else:
        _room = 'unknown'
        _fullScanDone = False
        saveMapInfo()

    if not rpcSend.navManagerServers['cartControl']['simulated']:
        cartsCartOrientation, cartsPosX, cartsPosY, cartMoving, cartRotating = (0, 0, 0, False, False)
        try:
            cartsCartOrientation, cartsPosX, cartsPosY, cartMoving, cartRotating = rpcSend.navManagerServers['cartControl']['conn'].root.exposed_getCartInfo()
        except Exception as e:
            log(f"could not get cartInfo from 'cartControl', {e}")

        setCartLocation(cartsPosX, cartsPosY)
        setCartOrientation(cartsCartOrientation)
        log(f"cartInfo: cartsCartOrientation: {cartsCartOrientation}, cartsPosX: {cartsPosX}, cartsPosY: {cartsPosY}, cartMoving: {cartMoving}, cartRotating: {cartRotating}")


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


def setFullScanDone(newStatus):

    global _fullScanDone

    _fullScanDone = newStatus

    # persist fullScanDone information
    saveMapInfo()

