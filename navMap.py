import time
import os, shutil

import numpy as np
import json
import rpyc
import cv2
from pyrecord import Record

import config
import rpcSend
import robotControl
import marker

Point2D = Record.create_type("Point2D","x","y")

rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True


MM_PER_MAP_PIXEL = 20       # each pixel in the map represents a 20x20 mm square
MAP_HEIGHT = 1000
MAP_WIDTH = 1000

floorPlan = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)
floorPlanFat = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)

_obstacles = None
_obstacleMapOrientation = 0
_obstacleMapLocation = (0,0)
_triggerMapUpdate = False
_mapUpdateDone = True

_scanLocations = []

mapCenterX = int(floorPlan.shape[0]/2)
mapCenterY = int(floorPlan.shape[1]/2)

mapCenter = Point2D(mapCenterX, mapCenterY)

CART_RADIUS_MM = 340        # cart representation as circle in the map, footprint of robot 
cartRadiusPix = int(CART_RADIUS_MM / MM_PER_MAP_PIXEL)



def clearFloorPlan():

    global floorPlan, floorPlanFat

    floorPlan = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)
    floorPlanFat = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)


def loadFloorPlan(room):
    '''
    try to load the floor plan of the last used room
    '''
    global floorPlan, floorPlanFat

    floorPlan = cv2.imread(f"{config.PATH_ROOM_DATA}/{room}/floorPlan/floorPlan.jpg", 0)
    if floorPlan is not None:
        floorPlanFat = cv2.imread(f"{config.PATH_ROOM_DATA}/{room}/floorPlan/floorPlanFat.jpg", 0)

    cv2.imshow("floorPlanFat", floorPlanFat)
    cv2.waitKey(500)
    cv2.destroyAllWindows()

    if floorPlan is None or floorPlanFat is None:
        floorPlan = None
        floorPlanFat = None
        return False

    return True


def clearCartLocationImages():

    global floorPlan, floorPlanFat

    floorPlan = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)
    floorPlanFat = np.zeros((MAP_HEIGHT,MAP_WIDTH), dtype=np.uint8)


def loadScanLocations():

    global _scanLocations

    filename = f"{config.PATH_ROOM_DATA}/{config._room}/scanLocations.json"
    if os._exists(filename):
        with open(filename, "r") as read_file:
            _scanLocations = json.load(read_file)
    else:
        _scanLocations = []


def getCartLocationMap():
    cartX, cartY = _obstacleMapLocation
    return int((cartX / MM_PER_MAP_PIXEL) + MAP_WIDTH/2), int((-cartY / MM_PER_MAP_PIXEL) + MAP_HEIGHT/2)


def addScanLocation():
    '''
    add the scan position to the list of visited positions (map pixel pos)
    for Y-Pos 0 is bottom
    '''
    global _scanLocations

    cartX, cartY = getCartLocationMap()
    _scanLocations.append((cartX, cartY))
    
    # persist list of scan positions
    filename = f"{config.PATH_ROOM_DATA}/{config._room}/scanLocations.json"
    with open(filename, "w") as write_file:
        json.dump(_scanLocations, write_file)



def setMapUpdateTrigger(new):

    global _triggerMapUpdate

    _triggerMapUpdate = new


def getMapUpdateTrigger():
    return _triggerMapUpdate


def setMapUpdateDone(new):

    global _mapUpdateDone

    _mapUpdateDone = new
    if new:
        config.log(f"mapUpdateDone {new}")


def isMapUpdateDone():
    return _mapUpdateDone



def getDepth():
    if not rpcSend.navManagerServers['kinect']['simulated']:
        return rpcSend.getDepth(config.getCartOrientation())
    return 0


def addCartcamImage(room, orientation):
    try:
        imgCart = rpcSend.getCartcamImage()
    except Exception as e:
        config.log(f"error getting cartcam image: {e}")
        return False

    if imgCart is not None:
        imgPath = f"{config.PATH_ROOM_DATA}/{room}/cartcamImages/{orientation}.jpg"
        cv2.imwrite(imgPath, imgCart)
        return True


def addObstacleMap(depthImage):

    global _obstacles, _obstacleMapOrientation

    # wait for possibly still running last map update
    while not isMapUpdateDone():
        time.sleep(0.1)

    # trigger floor plan update (separate thread)
    _obstacles = depthImage
    _obstacleMapOrientation = config.getCartOrientation()
    _obstacleMapLocation = config.getCartLocation()
    setMapUpdateTrigger(True)
    setMapUpdateDone(False)


def saveMapImage(img, type):
    mapPartFilename = f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/{type}{config.getCartOrientation():03.0f}.jpg"
    cv2.imwrite(mapPartFilename, img)


def translateImage(img, translateX, translateY):
    M = np.float32([[1,0,translateX],[0,1,translateY]])
    return cv2.warpAffine(img, M, img.shape[:2])


def rotateImage(img, rotation):
    cols, rows = img.shape[:2]
    M = cv2.getRotationMatrix2D((cols/2,rows/2), rotation, 1)
    return cv2.warpAffine(img, M, (cols,rows))
    

def addPartialMap():
    '''
    this function is called by the update floor plan thread
    it uses the array of distances for each column (640) as created by the kinect task
    '''
    config.log(f"addPartialMap, obstacleMapOrientation {_obstacleMapOrientation}, obstacleMapLocation: {_obstacleMapLocation}")

    # store the distance array for verification, distances are from right to left!
    filename = f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/planPart_{_obstacleMapOrientation}.json"
    if os._exists(filename):
        with open(filename, 'w') as jsonFile:
            json.dump(np.nan_to_num(_obstacles).tolist(), jsonFile, indent=4)
    else:
        config.log(f"addPartialMap did not find the file {filename}")
        return

    # create a 2d image from column and obstacle-distancies in the column. the distance represents the closest obstacle of that column
    # cam position is middle bottom
    MAX_SCALED_DIST = 400       # distance in 2 cm steps, max 8 m
    numColumns = 640
    depth = MAX_SCALED_DIST     # corresponds to 8 meters distance
    floorPlanWidth, floorPlanHeight = floorPlan.shape[:2]

    # add the distance information in an image of size floorPlan
    obstacleMap = np.zeros_like(floorPlan)      # numpy image is rows, cols
    obstacleMapFat = np.zeros_like(floorPlan)

    # draw obstacles as seen from cam at middle bottom of image (camPoint)
    baseRow = int(obstacleMap.shape[1]/2)        # place distance 0 at center of partial floor plan
    baseCol = int(obstacleMap.shape[0]/2)
    scaledObstacleDistancies = _obstacles/20
    for x, y in enumerate(scaledObstacleDistancies):   # here y is distance in mm
        if not np.isnan(y) and y < MAX_SCALED_DIST:

            # calculate col from col angle and distance
            colAngle = (x * config.KINECT_X_RANGE / numColumns) - (config.KINECT_X_RANGE / 2)
            col = y * np.tan(np.radians(colAngle))

            cv2.circle(obstacleMap, (baseCol - int(col), baseRow - int(y)), 1, 255, -1)                  #opencv point is (x, y)!!!!

            # the fat version of the obstacle map takes the size of the robot into account in order to keep the
            # center of the robot away from the obstacle line
            cv2.circle(obstacleMapFat, (baseCol - int(col), baseRow - int(y)), cartRadiusPix, 255, -1)

    # add a base line for verification only
    xMax = int(MAX_SCALED_DIST * np.tan(np.radians(config.KINECT_X_RANGE / 2)))
    cv2.line(obstacleMap, (baseCol-xMax,baseRow), (baseCol+xMax,baseRow), (255), 1)

    # only for visual control of obstacles
    saveMapImage(obstacleMap, "bw_")
    saveMapImage(obstacleMapFat, "bwFat_")

    # calc image rotation from cartOrientation
    # NOTE: graphic programs use 90 degrees (up) as 0 and positive values for clockwise rotation!
    # while cart orientation 0 is to the right and positive is counterclock
    imgRotation = -_obstacleMapOrientation
    #imgRotation = -45

    # now rotate the partial map around the center (camPoint)
    partToAdd = rotateImage(obstacleMap, imgRotation)
    partToAddFat = rotateImage(obstacleMapFat, imgRotation)

    # and translate it to the carts position
    partToAdd = translateImage(partToAdd, _obstacleMapLocation[0], _obstacleMapLocation[1])

    # add arrow to mark cart orientation
    cartXMap = int(_obstacleMapLocation[0] + (floorPlanWidth/2))
    cartYMap = int(_obstacleMapLocation[1] + (floorPlanHeight/2))
    arrowLength = 10
    arrowXCorr = int(arrowLength * np.cos(np.radians(-imgRotation+90)))
    arrowYCorr = int(arrowLength * np.sin(np.radians(-imgRotation+90)))
    cv2.arrowedLine(floorPlan, (cartXMap+arrowXCorr, cartYMap+arrowYCorr), (cartXMap-arrowXCorr,cartYMap-arrowYCorr), (200), 1, tipLength=0.3 )     # mark cart location

    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/planPart_{_obstacleMapOrientation}.jpg", partToAdd)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/planPartFat_{_obstacleMapOrientation}.jpg", partToAddFat)

    # add the partial part to the floor plan
    cv2.addWeighted(floorPlan, 1, partToAdd, 1, 0.0, floorPlan)
    cv2.addWeighted(floorPlanFat, 1, partToAddFat, 1, 0.0, floorPlanFat)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/floorPlan.jpg", floorPlan)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config._room}/floorPlan/floorPlanFat.jpg", floorPlanFat)
    '''
    cv2.imshow("obstacleMap", obstacleMap)
    cv2.imshow("partToAdd", partToAdd)
    cv2.imshow("floorPlan", floorPlan)
    cv2.waitKey(500)
    cv2.destroyAllWindows()
    '''

###########################################################
# runs as separate thread
###########################################################
def updateFloorPlan():

    while True:

        if getMapUpdateTrigger():

            setMapUpdateTrigger(False)
            addPartialMap()
            setMapUpdateDone(True)

        time.sleep(1)

    
def evalMaxProbeDistance(x, y):
    '''
    based on the current cart position on the map eval the max distance
    to the map border
    '''
    n1 = np.linalg.norm(np.array([x, y]))
    n2 = np.linalg.norm(np.array([MAP_HEIGHT - x, y]))
    n3 = np.linalg.norm(np.array([x, MAP_WIDTH - y]))
    n4 = np.linalg.norm(np.array([MAP_HEIGHT - x, MAP_WIDTH - y]))

    return int(max(np.array([n1,n2,n3,n4])))

    
def evalNewPos(x, y, deg, dist):
    xNew = int(x + dist * np.cos(np.radians(deg)))
    yNew = int(y - dist * np.sin(np.radians(deg)))
    return (xNew, yNew)


def findNewScanLocation():
    '''
    based on the map, the cart size and already visited places try to find another
    position to scan for markers

    Be careful with units! 
    we have map pixels and mm. from the map we get pixel values, for all distancies
    used for cart movements we use however millimeters
    '''

    #########################################################
    showEval = False
    ##########################################################

    config.log("start findNewScanLocation")
    SCAN_DEGREE_STEPS = 5
    numSections = int(360 / SCAN_DEGREE_STEPS)
    obstacleDistancies = np.zeros(numSections)
    checkPosMapX = np.zeros(numSections, dtype=np.uint16)
    checkPosMapY = np.zeros(numSections, dtype=np.uint16)
    
    '''
    beside the real obstacles we also want to avoid going forth and back between 2 scan positions.
    Block distance/degree positions already scanned and create a virtual obstacle at the recorded scan positions
    '''
    for i in range(len(config._scanLocations) - 1):

        cv2.circle(config.floorPlanFat, config._scanLocations[i], cartRadiusPix, 255, -1)
        #degree, distance = navGlobal.distDegToScanPos(i)
        #index = int(round(degree / SCAN_DEGREE_STEPS))
        #obstacleDistancies[index] *= 0.5    # half the distance for this virtual obstacle
    
    '''
    Based on the current cart position define the max distance to the map corners.
    This limits the range we have to look for obstacles
    '''
    cartMapX, cartMapY = getCartLocationMap()
    maxProbeDistance = evalMaxProbeDistance(cartMapX, cartMapY)
    
    # in maxProbeDistance range check for obstacle-free cart position
    for mapDistance in range(2*cartRadiusPix, maxProbeDistance, cartRadiusPix):

        # for a set of directions check for obstacles
        for loop2 in range(0, len(obstacleDistancies)):

            # check for direction already blocked
            if obstacleDistancies[loop2] > 0:
                continue

            # each entry in obstacleDistancies is bound to a direction
            # modify direction to get 0 as straight up
            direction = ((loop2 * SCAN_DEGREE_STEPS) + 90) % 360

            # position of obstacle detection with direction and distance off the current location
            checkPosMapX[loop2], checkPosMapY[loop2] = evalNewPos(cartMapX, cartMapY, direction, mapDistance)

            #print(f"direction: {direction}, checkPosMap: {checkPosMapX,checkPosMapY}")

            spotlightSize = int(1.5 * cartRadiusPix)
            
            mask = cv2.circle(np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8), (checkPosMapX[loop2], checkPosMapY[loop2]), spotlightSize, 255, -1)
            # cv2.imshow("mask", mask)

            # for visualization only (ring)
            if showEval:
                mask1 = cv2.circle(np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8), (checkPosMapX[loop2],checkPosMapY[loop2]), spotlightSize, 255)
                merged = floorPlanFat + mask1
                #print(f"loop2: {loop2}, checkPosMap: {checkPosMapX[loop2], checkPosMapY[loop2]}")

            obst = cv2.bitwise_and(floorPlanFat, mask) 

            if showEval:
                #print(f"pixels in mask {np.sum(obst)}")
                cv2.imshow("plan and mask area", merged)
                cv2.waitKey(10)
                cv2.destroyAllWindows()
                
            # if the bitwise and of map and mask > 50 (ignore single bits) set the distance as obstacleDistance
            if np.sum(obst) > 50:
                obstacleDistancies[loop2] = mapDistance * MM_PER_MAP_PIXEL
                #navGlobal.log(f"obstacle at degree: {loop2 * SCAN_DEGREE_STEPS}, mapDistance: {mapDistance * 20}, obst: {np.sum(obst)}")


    # find the longest free move, index of farthest obstacle distance
    idxCandidate = np.argmax(obstacleDistancies)

    mapX, mapY = getCartLocationMap()

    if showEval:
        print(f"idxCandidate: {idxCandidate}, targetPos = {checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]}")
        target = floorPlanFat
        cv2.circle(target, (checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]), 10, 255)
        cv2.line(target, (mapX, mapY), (checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]), 255, 2)

        arrowLength = 10
        arrowXCorr = int(arrowLength * np.cos(np.radians(90 + config.getCartOrientation())))
        arrowYCorr = int(arrowLength * np.sin(np.radians(90 + config.getCartOrientation())))
        cv2.arrowedLine(target, (mapX + arrowXCorr, mapY + arrowYCorr),
                        (mapX - arrowXCorr, mapY - arrowYCorr), 200, 1, tipLength=0.3)  # mark cart orientation

        cv2.imshow("target", target)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # map 0 is to the top, compensate for cart orientation
    targetInMapDegree = idxCandidate * SCAN_DEGREE_STEPS
    #requiredCartRotation = 360 - config.getCartOrientation() - targetInMapDegree

    config.log(f"idxCandidate: {idxCandidate}, rotate: {targetInMapDegree}, distance: {obstacleDistancies[idxCandidate]}")
    #cv2.waitKey()

    # check for minimal free room        
    if obstacleDistancies[idxCandidate] < 1000:
        # give up
        config.log("findNewScanLocation, no candidate position found")
        return (0,0)
    else:
        # go partially towards that direction
        
        # try to move close to the border to avoid blocking future positions
        distanceMm = obstacleDistancies[idxCandidate] - 500

        cartX, cartY = config.getCartLocation()
        config.log(f"new scan location found, degree: {int(targetInMapDegree)}, distance: {int(distanceMm)}, cartX: {cartX}, cartY: {cartY}")

        # return the candidate angle and distance (absolute map value)
        return (targetInMapDegree, distanceMm)


def fullScanAtPosition(lookForMarkers=[]):
    '''
    will start at current cart position and orientation
    scans with different head rotation angles images for markers
    rotates with cart around center of cart and 
    - takes a kinect depth image for each cart orientation
    - takes a cart cam image to check for docking station marker
    for each cart orientation
    - takes a inmoov eye cam picture and adds it to the room folder
    - checks for markers and adds them to the marker list
    after full cart rotation try to find another cart position for completing the floor plan (findNewScanLocation)
    if none found consider room as mapped
    '''

    addScanLocation()     # lets us remember we have been here

    rpcSend.queryCartInfo()

    # until we have a map update function start with a clear floor plan
    clearCartLocationImages()

    # move InMoov eye cam into capturing pose
    marker.setupEyeCam()

    # eval number of necessary rotations to get a full circle
    numPlannedCartRotationSteps = int(360 / config.KINECT_X_RANGE)    # assume KINECT_X_RANGE adds up to 360

    # for all orientations of the cart
    while numPlannedCartRotationSteps > 0:

        # request a kinect and a cartcam picture
        config.log(f"take kinect and cartcam image, orientation: {config.getCartOrientation()}")
        if not rpcSend.navManagerServers['aruco']['simulated']:
            if not addCartcamImage(config._room, config.getCartOrientation()):
                return False

        if not rpcSend.navManagerServers['kinect']['simulated']:
            depthImage = rpcSend.getDepth(config.getCartOrientation())

            depthFolder = f"{config.PATH_ROOM_DATA}/{config._room}/depthImages"
            cv2.imwrite(f"{depthFolder}/{config.getCartOrientation():03}.jpg", depthImage)

            # use the depth map (w=640, h=480) to create a top view of obstacles
            addObstacleMap(depthImage)

        if marker.scanWithHead(startDegrees= -config.KINECT_X_RANGE / 2,
                            endDegrees=config.KINECT_X_RANGE / 2,
                            steps=5):

            # rotate cart
            numPlannedCartRotationSteps -= 1
            if numPlannedCartRotationSteps > 0:
                robotControl.rotateCartRelative(config.KINECT_X_RANGE, 200)
        else:
            return False    # problems with scanWithHead
    # look out straight for next moves
    robotControl.servoMoveToPosition('head.eyeY', 90)
    robotControl.servoMoveToPosition('head.rothead', 90)

    # silence neck and rothead (head may move down though)
    robotControl.servoSetAutoDetach('head.neck', 1000)
    robotControl.servoSetAutoDetach('head.rothead', 1000)
    return True


def createImageFolders():
    """
    copy existing room folder to folder ..._depricated
    create new empty room folders
    :return:
    """
    roomFolder = f"{config.PATH_ROOM_DATA}/{config._room}"
    if os.path.exists(roomFolder):
        if os.path.exists(roomFolder+"_depricated"):
            try:
                shutil.rmtree(roomFolder+"_depricated")
            except Exception as e:
                config.log(f"could not remove roomFolder: {roomFolder}, error: {str(e)}")
                raise SystemExit(0)

        shutil.copytree(roomFolder, roomFolder+"_depricated")

    if os.path.exists(roomFolder):
        try:
            shutil.rmtree(roomFolder)
        except Exception as e:
            config.log(f"could not remove roomFolder: {roomFolder}, error: {str(e)}")
            raise SystemExit(0)

    floorPlanFolder = f"{roomFolder}/floorPlan"
    os.makedirs(floorPlanFolder)

    wallImagesFolder = f"{roomFolder}/wallImages"
    os.makedirs(wallImagesFolder)

    cartcamImagesFolder = f"{roomFolder}/cartcamImages"
    os.makedirs(cartcamImagesFolder)

    depthImagesFolder = f"{roomFolder}/depthImages"
    os.makedirs(depthImagesFolder)


def createFloorPlan():

    createImageFolders()

    try:
        if fullScanAtPosition():
            config.setFullScanDone(True)
            return True
        else:
            return False

    except config.CartError as e:
        config.log(f"cart rotation/move unsuccessful {e}")
        return False


def rover():
    try:

        absDegree, distance = findNewScanLocation()
        if distance > 0:
            moveSuccess = robotControl.moveCart(absDegree, distance, 200)
            if moveSuccess:
                config.setTask("pop")
            else:
                config.log(f"move to new scan location failed")
                config.setTask("notask")
        else:
            config.log("no other scan location found")
            config.setTask("notask")

    except config.CartError as e:
        config.log(f"rover - cart command unsuccessful {e}")
        config.setTask("notask")
    except config.CartControlError as e:
        config.log(f"rover - cart control failure {e}")
        config.setTask("notask")
    except Exception as e:
        config.log(f"rover, unexpected exception with moveCart {e}")
        config.setTask("notask")

    return
