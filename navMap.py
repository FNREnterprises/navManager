
import time
import os, shutil
import imutils

import numpy as np
import json
import _pickle as pickle
import cv2
#from pyrecord import Record
from skimage.morphology import skeletonize

import inmoovGlobal
import config
import rpcSend
import robotHandling
import marker
import guiUpdate
import cartHandling
import navManager
import threadProcessImages


MM_PER_MAP_PIXEL = 20       # each pixel in the map represents a 20x20 mm square
MAP_HEIGHT = 1000
MAP_WIDTH = 1000
CART_RADIUS_MM = 340        # cart representation as circle in the map, footprint of robot
cartRadiusPix = int(CART_RADIUS_MM / MM_PER_MAP_PIXEL)

CART_WIDTH_MAP = round(440 / MM_PER_MAP_PIXEL)
CART_LENGTH_MAP = round(600 / MM_PER_MAP_PIXEL)

SCAN_LOCATION_MARKER_SHADE = 150




def clearFloorPlan():

    config.floorPlan = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.uint8)
    config.floorPlanFat = np.zeros_like(config.floorPlan)

    config.room = "unknown"
    config.fullScanDone = False
    saveMapInfo()

    config.scanLocations = []
    saveScanLocations()

    config.markerList = []
    saveMarkerList()


def loadFloorPlan(room):
    """
    try to load the floor plan of the last used room
    """
    filePath = f"{config.PATH_ROOM_DATA}/{room}/floorPlan/floorPlan.jpg"
    if os.path.isfile(filePath):
        config.floorPlan = cv2.imread(filePath, cv2.IMREAD_GRAYSCALE)
        if config.floorPlan is not None:
            config.floorPlanFat = cv2.imread(f"{config.PATH_ROOM_DATA}/{room}/floorPlan/floorPlanFat.jpg", cv2.IMREAD_GRAYSCALE)

        #cv2.imshow("floorPlanFat", config.floorPlanFat)
        #cv2.waitKey(500)
        #cv2.destroyAllWindows()

    if config.floorPlan is None or config.floorPlanFat is None:
        config.floorPlan = None
        config.floorPlanFat = None
        return False

    return True


def buildImageName(x, y, degrees=None, pitch=None):
    """
    use rounded location and degrees values for imageName
    :return: <rounded x>_<rounded y>_<rounded degrees>
    """
    nX = f"{round(x / 100) * 100:+05d}"
    nY = f"{round(y / 100) * 100:+05d}"
    ndegrees = f"{round(degrees / 5) * 5:+04d}" if degrees is not None else ""
    nPitch = f"{pitch:+04d}" if pitch is not None else ""
    return f"{nX}{nY}{ndegrees}{nPitch}"


def loadScanLocations():

    filename = f"{config.PATH_ROOM_DATA}/{config.room}/scanLocations.json"
    if os.path.isfile(filename):
        with open(filename, "r") as read_file:
            config.scanLocations = json.load(read_file)
    else:
        config.scanLocations = []


def saveScanLocations():

    filename = f"{config.PATH_ROOM_DATA}/{config.room}/scanLocations.json"
    with open(filename, "w") as write_file:
        json.dump(config.scanLocations, write_file, indent=2)


def saveMarkerList():

    """filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerList.pickle"
    with open(filename, "wb") as write_file:
        for m in config.markerList:
            pickle.dump(m, write_file)
    """
    filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerList.json"
    with open(filename, "w") as write_file:
        markers = []
        for m in config.markerList:
            markers.append(m.props())
        json.dump(markers, write_file, indent=2)



def loadMarkerList():

    """filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerList.pickle"
    if os.path.exists(filename):
        removeFile = False
        with open(filename, "rb") as read_file:
            while True:     # each marker gets loaded separately
                try:
                    thisMarker = pickle.load(read_file)
                    config.markerList.append(thisMarker)
                    config.log(f"marker: {thisMarker}")
                except EOFError:
                    break
                except Exception as e:
                    removeFile = True
                    config.log(f"error loading markers, remove file")
                    break
        if removeFile:
            os.remove(filename)
    """
    filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerList.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            markers = json.load(read_file)

        for m in markers:
            oMarker = config.cMarker()
            oMarker.markerId = m['markerId']
            oMarker.cameraType = m['cameraType']
            oMarker.cartX = m['cartX']
            oMarker.cartY = m['cartY']
            oMarker.cartDegrees = m['cartDegrees']     # eyecam: head degrees, cartcam
            oMarker.camDegrees = m['camDegrees']
            oMarker.atAngleFromCart = m['atAngleFromCart']
            oMarker.distanceCamToMarker = m['distanceCamToMarker']
            oMarker.markerX = m['markerX']
            oMarker.markerY = m['markerY']
            oMarker.markerDegrees = m['markerDegrees']

            config.markerList.append(oMarker)
            config.log(f"marker: {m}")


def evalMapLocation(cartX, cartY):
    return int((cartX / MM_PER_MAP_PIXEL) + MAP_WIDTH/2), int(MAP_HEIGHT/2 - (cartY / MM_PER_MAP_PIXEL))


def addScanLocation():
    """
    add the rounded cart position as scan location
    """
    locX = round(config.oCart.getCartX() / 100) * 100
    locY = round(config.oCart.getCartY() / 100) * 100
    loc = (locX, locY)

    if not loc in config.scanLocations:
        config.scanLocations.append(loc)
        config.log(f"new scan location added: {loc}")

    saveScanLocations()


def takeDepthcamImage(show=False):

    config.depthcamImage = rpcSend.getImage(inmoovGlobal.HEAD_DEPTH)
    if config.depthcamImage is None:
        config.log(f"WARNING: could not acquire depthcam image")
        return False

    if config.depthcamImage is not None:
        config.flagProcessDepthcamImage = True      # signal for threadProcessImages

        if show:
            cv2.imshow("depthcam", config.depthcamImage)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    return True


def takeHeadcamImage(show=False):

    config.headcamImage = rpcSend.getImage(inmoovGlobal.HEAD_RGB)
    if config.headcamImage is None:
        config.log(f"WARNING: could not acquire headcam image")
        return False

    if config.headcamImage is not None:
        config.flagProcessHeadcamImage = True

        if show:
            cv2.imshow("headcam", config.headcamImage)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    return True


def takeEyecamImage(show=False):

    config.eyecamImage = rpcSend.getImage(inmoovGlobal.EYE_CAM)
    if config.eyecamImage is None:
        config.log(f"WARNING: could not acquire eyecam image")
        return False

    if config.eyecamImage is not None:
        config.flagProcessEyecamImage = True

        if show:
            cv2.line(config.cartcamImage, (320,0),(320,479),255,2)
            cv2.imshow("cartCam", config.cartcamImage)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    return True


def takeCartcamImage(show=False):

    config.cartcamImage = rpcSend.getImage(inmoovGlobal.CART_CAM)
    if config.cartcamImage is None:
        config.log(f"WARNING: could not acquire cartcam image")
        return False

    if config.cartcamImage is not None:
        config.flagProcessCartcamImage = True

        if show:
            cv2.line(config.cartcamImage, (320,0),(320,479),255,2)
            cv2.imshow("cartCam", config.cartcamImage)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    return True


def addCenter(img):
    # draw hair cross at 0,0
    hairCrossColor = (0,255,255)
    cv2.line(img, (495,500), (505, 500), hairCrossColor , 1)
    cv2.line(img, (500, 495), (500, 505), hairCrossColor, 1)



def addTarget(img):
    # show target if requested
    targetMapX, targetMapY = evalMapLocation(config.oTarget.getCartX(), config.oTarget.getCartY())
    cv2.circle(img,(targetMapX,targetMapY), 4, config.targetColor, -1)


def addMarker(img, markerX, markerY, markerDegrees):
    mapX, mapY = evalMapLocation(markerX, markerY)
    #config.log(f"markerMapX: {mapX}, markerMapY: {mapY}, markerDegrees:{markerDegrees}")
    addArrow(img, mapX, mapY, markerDegrees+90, 600/20, config.markerColor)      # cart center 600 mm in front of marker
    baseLength = 10
    baseXCorr = int(baseLength * np.cos(np.radians(markerDegrees)))
    baseYCorr = int(baseLength * np.sin(np.radians(markerDegrees)))
    cv2.line(img,(mapX - baseXCorr, mapY + baseYCorr), (mapX + baseXCorr, mapY - baseYCorr), config.markerColor, 3)
    cv2.line(img,(mapX - baseXCorr, mapY + baseYCorr), (mapX + baseXCorr, mapY - baseYCorr), (0,0,0), 1)


def addPathToTarget(img):
    cartMapX, cartMapY = evalMapLocation(config.oCart.getCartX(), config.oCart.getCartY())
    targetMapX, targetMapY = evalMapLocation(config.oTarget.getCartX(), config.oTarget.getCartY())
    cv2.line(img,(cartMapX, cartMapY), (targetMapX, targetMapY), (0,0,255), 4)


def addArrow(img, mapX, mapY, degrees, length, color=(128,128,128)):

    arrowXCorr = int(length * np.cos(np.radians(degrees)))
    arrowYCorr = int(length * np.sin(np.radians(degrees)))
    #cv2.circle(img, (mapX,mapY), 3, color, -1)
    cv2.arrowedLine(img, (mapX, mapY),
                    (mapX + arrowXCorr, mapY - arrowYCorr), color, 3, tipLength=0.3)  # mark cart degrees
    cv2.arrowedLine(img, (mapX, mapY),
                    (mapX + arrowXCorr, mapY - arrowYCorr), (0,0,0), 1, tipLength=0.3)  # black inside
    return img


def addCart(img, cartX, cartY, cartDegrees, cartColor):
    """
    img is the map (1000*1000 pix)
    can draw a cart on the map
    :param img:
    :return:
    """
    mapX, mapY = evalMapLocation(cartX, cartY)

    # overlay cart at position and degrees
    cartImg = np.zeros((CART_WIDTH_MAP+2, CART_LENGTH_MAP+2, 3), dtype = np.uint8)
    cv2.rectangle(cartImg, (0,0),(CART_LENGTH_MAP, CART_WIDTH_MAP), cartColor, 1)
    h,w = cartImg.shape[:2]

    # add arrow
    addArrow(cartImg, round(w-14), round(h/2), 0, 10, cartColor)

    # make sure image size adjusts to rotated cart size
    #rotated = imutils.rotate_bound(cartImg, -config.oCart.getCartYaw())
    rotated = imutils.rotate_bound(cartImg, -cartDegrees)
    rotH,rotW = rotated.shape[:2]

    # add cart to map
    x1 = mapX - round(rotW/2)
    x2 = x1 + rotW
    y1 = mapY - round(rotH/2)
    y2 = y1 + rotH
    img[y1:y2,x1:x2,:] = rotated

    #config.log(f"cartMapX: {mapX}, cartMapY: {mapY}, cartDegrees:{cartDegrees}")

    cv2.circle(img, (mapX,mapY), 5, cartColor)
    #cv2.imshow("cart", img)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()


def loadMapInfo():
    """
    the cartControl task keeps track of position and location. this is necessary because the cart
    can also be moved by directly using the cartControl interface without connection to the navManager
    """

    # Getting back the map data:
    filename = f"{config.PATH_ROOM_DATA}/mapInfo.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            mapInfo = json.load(read_file)

        config.room = mapInfo['room']
        config.fullScanDone = mapInfo['fullScanDone']

    else:
        config.room = 'unknown'
        config.fullScanDone = False
        saveMapInfo()

    rpcSend.queryCartInfo()


def saveMapInfo():
    # Saving the objects:
    mapInfo = { 'room': config.room, 'fullScanDone': config.fullScanDone }
    filename = f"{config.PATH_ROOM_DATA}/mapInfo.json"
    with open(filename, "w") as write_file:
        json.dump(mapInfo, write_file, indent=2)



def rotateImage(img, rotation):
    cols, rows = img.shape[:2]
    M = cv2.getRotationMatrix2D((cols/2,rows/2), rotation, 1)
    return cv2.warpAffine(img, M, (cols,rows))


def cropped(imgA, imgB):
    """
    The cropped function returns the relevant part of the 2 images based on white pixels
    In addition it leaves a small black border to allow for small shifting without losing pixels
    :param imgA:
    :param imgB:
    :return:
    """
    mask = imgA > 0
    coords = np.argwhere(mask)  # Coordinates of non-black pixels.
    iAy0, iAx0 = coords.min(axis=0)             # Bounding box of non-black pixels.
    iAy1, iAx1 = coords.max(axis=0) + 1   # slices are exclusive at the top

    mask = imgB > 0
    coords = np.argwhere(mask)  # Coordinates of non-black pixels.
    iBy0, iBx0 = coords.min(axis=0)             # Bounding box of non-black pixels.
    iBy1, iBx1 = coords.max(axis=0) + 1   # slices are exclusive at the top

    y0 = min(iAy0, iBy0) - 10
    x0 = min(iAx0, iBx0) - 10
    y1 = max(iAy1, iBy1) + 10
    x1 = max(iAx1, iBx1) + 10

    return imgA[y0:y1, x0:x1], imgB[y0:y1, x0:x1], y0, x0    # Get a pointer to the bounding box within colImg


def evalMaxProbeDistance(x, y):
    """
    based on the current cart position on the map eval the max distance
    to the map border
    """
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
    """
    based on the map, the cart size and already visited places try to find another
    position to scan for markers

    Be careful with units!
    we have map pixels and mm. from the map we get pixel values, for all Distances
    used for cart movements we use however millimeters
    """

    #########################################################
    showEval = True
    ##########################################################
    wait = 0
    config.log(f"start findNewScanLocation")
    SCAN_DEGREE_STEPS = 5
    numSections = int(360 / SCAN_DEGREE_STEPS)
    obstacleSpot = np.zeros(numSections)
    candidateSpot = np.zeros(numSections)
    checkPosMapX = np.zeros(numSections, dtype=np.uint16)
    checkPosMapY = np.zeros(numSections, dtype=np.uint16)

    floorPlanFatCopy = np.copy(config.floorPlanFat)
    '''
    beside the real obstacles we also want to avoid going forth and back between 2 scan positions.
    Block distance/degree positions already scanned and create a virtual obstacle at the recorded scan positions
    Do not include current position in blocking
    '''
    #cv2.imshow("floorPlanFatCopy", floorPlanFatCopy)

    for i in range(len(config.scanLocations) - 1):
        mapX = int(config.scanLocations[i][0] / MM_PER_MAP_PIXEL) + 500
        mapY = 500 - int(config.scanLocations[i][1] / MM_PER_MAP_PIXEL)
        cv2.circle(floorPlanFatCopy, (mapX, mapY), cartRadiusPix*2, SCAN_LOCATION_MARKER_SHADE, -1)

    '''
    Based on the current cart position define the max distance to the map corners.
    This limits the range we have to look for obstacles
    '''
    mapX, mapY = evalMapLocation(config.oCart.getCartX(), config.oCart.getCartY())
    maxProbeDistance = evalMaxProbeDistance(mapX, mapY)
    
    # in maxProbeDistance range check for obstacle-free cart position
    for mapDistance in range(2*cartRadiusPix, maxProbeDistance, cartRadiusPix):

        # for a set of directions check for obstacles
        for loop2 in range(0, len(obstacleSpot)):

            # check for direction already blocked
            if obstacleSpot[loop2] > 0:
                continue

            # each entry in obstacleDistances is bound to a direction
            # modify direction to get 0 as straight up
            direction = (loop2 * SCAN_DEGREE_STEPS) % 360

            # position of obstacle detection with direction and distance off the current location
            checkPosMapX[loop2], checkPosMapY[loop2] = evalNewPos(mapX, mapY, direction, mapDistance)

            #print(f"direction: {direction}, checkPosMap: {checkPosMapX,checkPosMapY}")

            spotlightSize = int(1.5 * cartRadiusPix)
            
            mask = cv2.circle(np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8),
                              (checkPosMapX[loop2], checkPosMapY[loop2]),
                              spotlightSize, 255, -1)
            # cv2.imshow("mask", mask)

            # for visualization only (ring)
            if showEval:
                mask1 = cv2.circle(np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8), (checkPosMapX[loop2],checkPosMapY[loop2]), spotlightSize, 255)
                merged = floorPlanFatCopy + mask1
                #print(f"loop2: {loop2}, checkPosMap: {checkPosMapX[loop2], checkPosMapY[loop2]}")

            obst = cv2.bitwise_and(floorPlanFatCopy, mask)

            if showEval:
                #print(f"pixels in mask {np.sum(obst)}")
                #cv2.imshow("obst", obst)
                cv2.imshow("find location", merged)
                cv2.waitKey(wait)
                wait=1
                #cv2.destroyAllWindows()

            # if the bitwise and of map and mask > 50 (ignore single bits) set the distance as obstacleDistance
            # this will only find locations on a direct path
            boolArray = np.asarray(obst)
            whitePixInSpot = (boolArray > SCAN_LOCATION_MARKER_SHADE).sum()
            if whitePixInSpot > 50:
                obstacleSpot[loop2] = mapDistance * MM_PER_MAP_PIXEL    # this blocks further checks in that direction
            else:
                # check for spot area in previous scan location
                grayPixInSpot = (boolArray == SCAN_LOCATION_MARKER_SHADE).sum()
                if grayPixInSpot < 50:
                    # add to candidate if area is not visited yet
                    candidateSpot[loop2] = mapDistance * MM_PER_MAP_PIXEL

            #navGlobal.log(f"obstacle at degree: {loop2 * SCAN_DEGREE_STEPS}, mapDistance: {mapDistance * 20}, obst: {np.sum(obst)}")


    # find the longest free move, index of farthest candidate spot
    idxCandidate = int(np.argmax(candidateSpot))

    mapX, mapY = evalMapLocation(config.oCart.getCartX(), config.oCart.getCartY())

    if showEval:
        #config.log(f"idxCandidate: {idxCandidate}, targetPosMap = {checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]}")
        target = np.copy(config.floorPlanFat)
        cv2.circle(target, (checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]), 10, 255)
        cv2.line(target, (mapX, mapY), (checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]), 255, 2)

        locationId = f"{checkPosMapX[idxCandidate]:.0f}_{checkPosMapY[idxCandidate]:.0f}"
        cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/newScanLoc{locationId}.jpg", target)
        cv2.imshow("find location", target)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # idxCandidate is the number of lookup steps to find the farthest corner
    # lookout starts with cart degrees
    targetInMapDegree = idxCandidate * SCAN_DEGREE_STEPS

    config.log(f"idxCandidate: {idxCandidate}, targetPosMap = {checkPosMapX[idxCandidate], checkPosMapY[idxCandidate]}, degrees: {targetInMapDegree}, distance: {obstacleSpot[idxCandidate]}")
    #cv2.waitKey()

    # check for minimal distance for gaining new information
    if obstacleSpot[idxCandidate] < 1500:
        # give up
        config.log(f"findNewScanLocation, no candidate position found, min. distance = 1500")
        return (0,0)
    else:
        # go partially towards that direction
        
        # try to move close to the border to avoid blocking future positions
        distanceMm = obstacleSpot[idxCandidate] - 500

        #cartX, cartY = config.getCartLocation()
        #config.log(f"new scan location found, degree: {int(targetInMapDegree)}, distance: {int(distanceMm)}, cartX: {cartX}, cartY: {cartY}")
        config.log(f"new scan location found, degree: {int(targetInMapDegree)}, "
                   f"distance: {int(distanceMm)}, "
                   f"cartX: {config.oCart.getCartX()}, cartY: {config.oCart.getCartY()}")

        # return the candidate angle and distance (absolute map value)
        return (targetInMapDegree, distanceMm)


def rebuildMap():

    clearFloorPlan()

    directory = "D:/Projekte/InMoov/navManager/ROOMS/unknown/floorPlan"
    if os.path.isfile(f"{directory}/floorPlan.jpg"):
        os.remove(f"{directory}/floorPlan.jpg")
    if os.path.isfile(f"{directory}/floorPlanFat.jpg"):
        os.remove(directory + "/floorPlanFat.jpg")

    for file in os.listdir(directory):

        if file.endswith(".json"):
            with open(f"{directory}/{file}", "r") as read_file:
                obstacleList = json.load(read_file)
            depthCamDistances = np.array(obstacleList)
            depthCamDegrees = int(file.split()[1].split(".")[0])
            config.log(f"load partialMap {len(depthCamDistances)} {depthCamDegrees}")
            config.flagAddPartialMap = True

            timeout = time.time() + 5
            while not config.flagAddPartialMapDone and time.time() < timeout:
                time.sleep(0.5)
            if time.time() > timeout:
                config.log(f"timeout addPartialMapDone")
                navManager.setTask("notask")
        else:
            continue



def adjustCartLocation(showBest=True):
    """
    if this is not the first full scan verify the carts position
    When doing a fullScanAtPosition we might end up with a floor plan that is slightly misaligned in relation
    to the existing floor plan
    It looks like the IMU is rather stable and rotating the scan plan for alignment does not help much
    However caused by the carts wheel slippage and the only roughly estimated side moves the cart position
    might be off after a number of moves
    Try to find the best shift (x,y) of the scan plan to match the floor plan
    """

    config.log(f"try to align new full scan floor plan with existing plan")

    # use gray image
    if len(config.floorPlanFat.shape) == 3:
        im1Gray = cv2.cvtColor(config.floorPlan, cv2.COLOR_BGR2GRAY)
    else:
        im1Gray = config.floorPlanFat
    if len(config.fullScanPlanFat.shape) == 3:
        im2Gray = cv2.cvtColor(config.fullScanPlanFat, cv2.COLOR_BGR2GRAY)
    else:
        im2Gray = config.fullScanPlanFat

    # limit comparison to white area with a black border
    imA, imB, dy, dx = cropped(im1Gray, im2Gray)

    x, y = 0,0

    im12 = cv2.add(imA, imB)

    im12Sum = np.sum(im12)      # reference sum to be optimized by rotation and shift of imgB
    #print(im12Sum)
    best = [im12Sum, x, y]

    # move new scan plan pixelwise around to find best match with floor plan
    # its currently brute force and does not try to shorten the loops
    for x in range(-15,15):
        imBx = imutils.translate(imB,x,0)

        for y in range(-15,15):
            imBxy = imutils.translate(imBx,0,y)
            imABxy = cv2.add(imA,imBxy)
            imABxySum = np.sum(imABxy)

            if imABxySum < best[0]:
                best = [imABxySum, x, y]
                imBest = imABxy.copy()


    # skeletonize the plan and fatten with cart size again
    # TODO find a way to get rid ot the strings attached to the contour
    skeleton = skeletonize(imBest > 100).astype(np.uint8)
    skeleton = cv2.bitwise_and(imBest, imBest, mask=skeleton)

    # replace area in floor plan with imBest
    h,w = imBest.shape
    im1Gray[dy:dy+h,dx:dx+w] = skeleton
    config.floorPlan = im1Gray

    kernel7 = np.ones((7,7),np.uint8)
    skeletonFat = cv2.dilate(skeleton, kernel7, iterations=5)
    im1Gray[dy:dy+h,dx:dx+w] = skeletonFat
    config.floorPlanFat = im1Gray

    # persist the floorPlan
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlan.jpg", config.floorPlan)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanFat.jpg", config.floorPlanFat)

    # move fullScanFat by the same offset pixels
    #imFat = imutils.translate(config.fullScanPlanFat, x, y)
    #imFat = cv2.add(config.floorPlanFat, imFat)
    ##cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanFat.jpg", imFat)

    # use the new map offset to correct the cart location
    xCorr = best[1] * MM_PER_MAP_PIXEL
    yCorr = best[2] * MM_PER_MAP_PIXEL

    config.log(f"update cart position based on full scan, xCorr:{xCorr}, yCorr: {yCorr}")
    rpcSend.adjustCartPosition(xCorr, yCorr, 0)

    if showBest:
        """
        wName = f"best, {best[1]}, {best[2]}"
        cv2.imshow(wName, imBest)
        cv2.moveWindow(wName, 100,500)

        wName = f"skeleton"
        cv2.imshow(wName, config.floorPlan)
        cv2.moveWindow(wName, 500,100)
        """
        wName = f"dilated"
        cv2.imshow(wName, config.floorPlanFat)
        cv2.moveWindow(wName, 500,500)

        cv2.waitKey(1000)
        cv2.destroyAllWindows()



def fullScanAtPosition(lookForMarkers=None):
    """
    will start at current cart position and degrees
    takes with different head rotation/neck angles rgb images and scans them for markers
    with head rotation angle 0 takes a depth image and creates the obstacle line
    after a full head scan rotates the cart (around center of cart)

    all eyecam pictures are added to the room folder

    identified markers are added to the marker list

    after full cart rotation try to find another cart position for completing the floor plan (findNewScanLocation)
    if none found consider room as mapped
    """

    config.log(f"in fullScanAtPosition")

    if lookForMarkers is None:      # python issue, a mutable default param lookForMarkers=[] raises an error
        lookForMarkers = []

    rpcSend.queryCartInfo()
    startAngle = config.oCart.getCartYaw()

    # move InMoov eye cam into capturing pose
    marker.setupEyeCam()

    # eval number of necessary cart rotations to get a full circle
    # use the fovH of the HEAD_DEPTH cam
    headFovH = config.cams[inmoovGlobal.HEAD_DEPTH]['fovH']
    numPlannedCartRotationSteps = int(360 / headFovH)
    cartRange = int(360 / (numPlannedCartRotationSteps + 1))

    # start with an empty scan plan
    config.fullScanPlan = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8)
    config.fullScanPlanFat = np.zeros((MAP_WIDTH, MAP_HEIGHT), dtype=np.uint8)

    # for all orientations of the cart
    eyeFovH = config.cams[inmoovGlobal.EYE_CAM]['fovH']
    numPlannedHeadRotationSteps = int(cartRange / eyeFovH)
    headRange = int(cartRange / (numPlannedHeadRotationSteps + 1))

    while numPlannedCartRotationSteps > 0:

        # request a cartcam picture, done in navMap thread
        config.log(f"take cartcam image, degrees: {config.oCart.getCartYaw()}")
        takeCartcamImage()

        # take several Eyecam images and one depth image with this cart orientation
        if marker.scanWithHead(startDegrees= -cartRange/2 + headRange/2,
                            endDegrees=cartRange/2 - headRange/2,
                            steps=numPlannedHeadRotationSteps+1):

            # check for image processing done
            timeout = time.time() + 5
            while config.flagProcessCartcamImage and time.time() < timeout:
                time.sleep(0.1)
            if time.time() > timeout:
                config.log(f"navMap, timeout processing CartcamImage, stopping scan")
                return False

            while not config.flagProcessDepthcamImage and time.time() < timeout:
                time.sleep(0.1)
            if time.time() > timeout:
                config.log(f"navMap, timeout processing DepthcamImage, stopping scan")
                return False

            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

            # rotate cart
            numPlannedCartRotationSteps -= 1
            depthXRange = config.cams[inmoovGlobal.HEAD_DEPTH]['fovH']
            relAngle = 360 - (numPlannedCartRotationSteps * depthXRange)
            nextDegrees = (relAngle + startAngle) % 360
            config.log(f"start angle: {startAngle}, rotation steps: {numPlannedCartRotationSteps}, next degrees: {nextDegrees}")

            if numPlannedCartRotationSteps > 0:
                try:
                    #config.log(f"rotation disabled for test")
                    if cartHandling.createMoveSequence(nextDegrees, 0, 0):
                        cartHandling.moveCart()
                except Exception as e:
                    config.log(f"failure in cart rotation to {nextDegrees} degrees, {e}")
                    return False

        else:
            config.log(f"scan with head failure")
            return False    # problems with scanWithHead

    if not config.fullScanDone:
        # save first floor plan
        cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlan.jpg", config.floorPlan)
        cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanFat.jpg", config.floorPlanFat)

    config.fullScanDone = True

    # TODO ask for room name
    saveMapInfo()

    imageName = buildImageName(depthCamX, depthCamY)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/fullScanPlan_{imageName}.jpg", config.fullScanPlan)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/fullScanPlanFat_{imageName}.jpg", config.fullScanPlanFat)

    # for additional full scans verify the alignment with the floor plan and adjust cart location
    if len(config.scanLocations) > 1:
        adjustCartLocation()

    addScanLocation()     # let us remember we have been here, use corrected position


    # look out straight for next moves
    robotHandling.servoMoveToPosition('head.eyeY', 90)
    robotHandling.servoMoveToPosition('head.rothead', 90)

    # silence neck and rothead (head may move down though)
    robotHandling.servoSetAutoDetach('head.neck', 300)
    robotHandling.servoSetAutoDetach('head.rothead', 300)
    return True


def createImageFolders():
    """
    copy existing room folder to folder ..._depricated
    create new empty room folders
    :return:
    """
    config.log(f"move current room to _depricated folder")
    roomFolder = f"{config.PATH_ROOM_DATA}/{config.room}"
    if os.path.exists(roomFolder):
        if os.path.exists(roomFolder+"_depricated"):
            config.log(f"remove _depricated folder")
            try:
                shutil.rmtree(roomFolder+"_depricated", ignore_errors=True)
            except Exception as e:
                config.log(f"could not remove roomFolder: {roomFolder}_depricated, error: {str(e)}")
                return False

        config.log(f"renamed current room folder to _depricated folder")
        time.sleep(1)
        try:
            os.rename(roomFolder, roomFolder+"_depricated")
        except Exception as e:
            config.log(f"can not rename roomFolder: {roomFolder} to {roomFolder}_depricated, error: {str(e)}")

    if os.path.exists(roomFolder):
        config.log(f"room folder still here, try to remove it")
        try:
            #os.system(f"rm -fr {roomFolder}")
            shutil.rmtree(roomFolder, ignore_errors=True)
            time.sleep(1)
        except Exception as e:
            config.log(f"could not remove roomFolder: {roomFolder}, error: {str(e)}")
            return False

    config.log(f"create empty folders")
    floorPlanFolder = f"{roomFolder}/floorPlan"
    if not os.path.exists(floorPlanFolder):
        os.makedirs(floorPlanFolder)

    floorPlanPartsFolder = f"{roomFolder}/floorPlan/floorPlanParts"
    if not os.path.exists(floorPlanPartsFolder):
        os.makedirs(floorPlanPartsFolder)

    wallImagesFolder = f"{roomFolder}/wallImages"
    if not os.path.exists(wallImagesFolder):
        os.makedirs(wallImagesFolder)

    cartcamImagesFolder = f"{roomFolder}/cartcamImages"
    if not os.path.exists(cartcamImagesFolder):
        os.makedirs(cartcamImagesFolder)

    config.log(f"delete scan and marker locations and move history")
    scanLocationFile = f"{roomFolder}/scanLocations.json"
    if os.path.exists(scanLocationFile):
        os.remove(scanLocationFile)
    markerLocationFile = f"{roomFolder}/markerLocations.json"
    if os.path.exists(markerLocationFile):
        os.remove(markerLocationFile)
    moveHistoryFile = f"{roomFolder}/moveHistory.json"
    if os.path.exists(moveHistoryFile):
        os.remove(moveHistoryFile)

    config.log(f"file work for new room done")

    return True


def createFloorPlan():

    config.log(f"clear folders for new floor plan")
    if not createImageFolders():
        config.log(f"could not remove room folder, open in exlorer/qdir?")
        return False

    # until we have a map update function start with a clear floor plan
    clearFloorPlan()

    # set location in cartControl to 0
    if not config.servers['cartControl'].simulated:
        config.servers['cartControl'].conn.root.setCartLocation(0, 0)

    return True


def rover():

    config.log(f"in rover")
    try:

        absDegree, distance = findNewScanLocation()
        if distance > 0:

            if cartHandling.createMoveSequence(absDegree, distance, 200):
                if cartHandling.moveCart():
                    navManager.setTask("pop")
                    return
            else:
                config.log(f"move to new scan location failed")
        else:
            config.log(f"no other scan location found")

    except config.CartError as e:
        config.log(f"rover - cart command unsuccessful {e}")
    #except Exception as e:
    #    config.log(f"rover, unexpected exception in rover: {e}")

    navManager.setTask("notask")
    return



