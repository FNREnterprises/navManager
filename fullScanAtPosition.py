
import time
import numpy as np

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonClasses
#from marvinglobal import cartClasses
#from marvinglobal import skeletonCommandMethods

import config
import imageHandler
import skeletonHandling
import cartHandling
import navMap

# for full scan at position
# list of neck positions and camera to use for each head rotation position
# list in order of neg to pos neckAngles
neckAngles = [{"neckAngle": mg.pitchLowerDegrees, "camType": mg.CamTypes.EYE_CAM},
              {"neckAngle": mg.pitchWallWatchDegrees, "camType": mg.CamTypes.HEAD_CAM},
              {"neckAngle": mg.pitchUpperDegrees, "camType": mg.CamTypes.EYE_CAM},
              ]


def setupEyeCamViewDirection():
    """
    make sure neck stays at position and set eyes direction straight out
    """
    config.log(f"set neck autoDetach to 15 sec")
    request = {'msgType': 'setAutoDetach', 'sender': config.processName,
               'servoName': 'head.neck', 'duration': 15000}
    config.marvinShares.skeletonRequestQueue.put(request)

    # set eye direction straight out
    skeletonHandling.requestServoDegrees('head.eyeX', 0, duration=100, wait=True)
    skeletonHandling.requestServoDegrees('head.eyeY', 0, duration=100, wait=True)


def scanWithHead(startDegrees, endDegrees, steps, lookForMarkers=None) -> bool:

    sequenceNumber = 0
    if lookForMarkers is None:      # avoid mutable default value lookForMarkers=[]
        lookForMarkers = []

    # create a list of head rotations for the requested scan
    rotheadDegreesList = list(np.linspace(startDegrees, endDegrees, steps))
    config.log(f"{rotheadDegreesList=}")

    # create a list of neck positions for each head yaw
    neckDegreesList = list(neckAngles)
    config.log(f"{neckDegreesList=}")

    #---------------------------------------------------------------------------------------------
    # before starting to loop over all rothead and neck positions find shortest way to start with
    #---------------------------------------------------------------------------------------------
    servoCurrentDict = config.marvinShares.servoDict.get(mg.SharedDataItems.SERVO_CURRENT)
    rotheadCurrent:skeletonClasses.ServoCurrent = servoCurrentDict.get('head.rothead')

    if rotheadCurrent.currentDegrees > 0:
        rotheadDegreesList.reverse()

    neckCurrent:skeletonClasses.ServoCurrent = servoCurrentDict.get('head.neck')
    if neckCurrent.currentDegrees > -5:
        neckDegreesList.reverse()

    # ---------------------------------------------------------------------------------------------

    while len(rotheadDegreesList) > 0:

        requestedHeadYaw = int(rotheadDegreesList.pop())
        if not skeletonHandling.requestServoDegrees('head.rothead', requestedHeadYaw, duration=300, wait=True):
            config.log(f"stop scan, head.rothead not positioned")
            return False

        cartLocation:mg.Location = config.marvinShares.cartDict.get(mg.SharedDataItems.CART_LOCATION)
        camDegrees = requestedHeadYaw
        cartDegrees = cartLocation.yaw
        absDegreeImage = (cartDegrees + camDegrees) % 360

        # with same rothead position use a upper and lower neck position for marker scan
        for neckPos in neckDegreesList:

            # take depth image only in rothead 0 position
            if requestedHeadYaw != 0 and neckPos['camType'] == mg.CamTypes.HEAD_CAM:
                continue

            if not skeletonHandling.requestServoDegrees('head.neck', neckPos['neckAngle'], duration=300, wait=True):
                config.log(f"stop scan, could not position head.neck in time")
                return False

            if neckPos['camType'] == mg.CamTypes.EYE_CAM:

                imageName = navMap.buildImageName(cartLocation.x, cartLocation.y, absDegreeImage, neckPos['neckAngle'])
                environment = config.getRoomData()
                filename = f"{config.PATH_ROOM_DATA}/{environment.roomName}/eyecamImages/{imageName}.jpg"

                config.log(f"request TAKE_IMAGE with EYE_CAM, rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}, imageId: {config.imageId}")

                # try to take image and check for any aruco markers
                request = {'msgType': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                           'camType': mg.CamTypes.EYE_CAM, 'imgPath': filename, 'show': False, 'findMarkers':[],
                           'checkBlurThreshold': True, 'maxWaitMs': 1000}

                # create image request object and send request to imageProcessing
                config.camRequest[request['camType']] = imageHandler.ImageRequest(request, timeout=7)

                # wait for result from imageProcessing (includes detection of aruco markers)
                config.camRequest[request['camType']].waitForResult()
                if not config.camRequest[request['camType']].isImageAvailable():
                    config.log(f"stop scan, eyeCamRequest failed, stop scan")
                    return False

                # check for scan for specific marker[s]
                if len(lookForMarkers) > 0:
                    # stop scan if we have found one of the requested markers
                    markerList = config.getMarkerList()
                    for marker in markerList:
                        if marker in lookForMarkers:
                            config.log(f"requested marker found, markerId: {marker}")
                            break

                    config.log(f"requested marker[s] not found yet")


            if neckPos['camType'] == mg.CamTypes.HEAD_CAM:
                config.log(f"request TAKE_IMAGE with HEAD_CAM, rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}, imageId: {config.imageId}")
                imageName = f"x{cartLocation.x}_y{cartLocation.y}_yaw{cartLocation.yaw}"
                environment = config.getRoomData()
                filename = f"{config.PATH_ROOM_DATA}/{environment.roomName}/headcamImages/{imageName}.jpg"

                request = {'msgType': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                           'camType': mg.CamTypes.HEAD_CAM, 'show': False, 'checkBlurThreshold': False, 'maxWaitMs': 5000, 'imgPath': filename}

                config.camRequest[request['camType']] = imageHandler.ImageRequest(request, timeout=8)
                #    config.log(f"TODO: do something with the depth image")

                config.camRequest[request['camType']].waitForResult()
                if not config.camRequest[request['camType']].isImageAvailable():
                    config.log(f"stop scan, headCamRequest failed, stop scan")
                    return False

        # for current rothead position all neck positions done
        neckDegreesList.reverse()       # continue with same neck position as in last rothead position

    config.log(f"scan with head done")
    return True


def fullScanAtPosition(lookForMarkers=None):
    """
    will start at current cart position and cart direction
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

    startAngleCart = config.getCartLocation().yaw

    # move InMoov eye cam into capturing pose
    setupEyeCamViewDirection()

    # eval number of necessary cart rotations to get a full circle
    # use the fovH of the HEAD cam
    headFovH = 60   #HEAD_CAM.fovH
    numPlannedCartRotationSteps = int(360 / headFovH)
    cartRange = int(360 / (numPlannedCartRotationSteps + 1))

    # start with an empty scan plan
    config.fullScanPlan = np.zeros((navMap.MAP_WIDTH, navMap.MAP_HEIGHT), dtype=np.uint8)
    config.fullScanPlanFat = np.zeros_like(config.fullScanPlan)

    # for all orientations of the cart
    eyeFovH = 12 #EYE_CAM.fovH
    numPlannedHeadRotationSteps = int(cartRange / eyeFovH)
    headRange = int(cartRange / (numPlannedHeadRotationSteps + 1))

    while numPlannedCartRotationSteps > 0:

        # request a cartcam picture from imageProcessing
        config.log(f"request TAKE_IMAGE with CART_CAM, imageId: {config.imageId}")
        cartLocation: mg.Location = config.marvinShares.cartDict.get(mg.SharedDataItems.CART_LOCATION)
        imageName = f"x{cartLocation.x}_y{cartLocation.y}_yaw{cartLocation.yaw}"
        environment = config.getRoomData()
        filename = f"{config.PATH_ROOM_DATA}/{environment.roomName}/cartcamImages/{imageName}.jpg"

        request = {'msgType': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                   'camType': mg.CamTypes.CART_CAM, 'checkBlurThreshold': True, 'maxWaitMs': 3000, 'imgPath': filename}
        config.camRequest[request['camType']] = imageHandler.ImageRequest(request, timeout=8)

        # take several Eyecam images and one depth image with this cart orientation
        if scanWithHead(startDegrees= -cartRange/2 + headRange/2,
                            endDegrees=cartRange/2 - headRange/2,
                            steps=numPlannedHeadRotationSteps+1):

            # for each rotation step we take a cart image
            # check for success in take cart image
            config.camRequest[mg.CamTypes.CART_CAM].waitForResult()
            if not config.camRequest[mg.CamTypes.CART_CAM].isImageAvailable():
                config.log(f"cartCamRequest failed, stop scan")
                return False


            #guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

            # rotate cart by horizontal observation range of head cam
            numPlannedCartRotationSteps -= 1
            relAngle = 360 - (numPlannedCartRotationSteps * headFovH)
            nextDegrees = (relAngle + startAngleCart) % 360
            config.log(f"start angle: {startAngleCart}, rotation steps: {numPlannedCartRotationSteps}, next degrees: {nextDegrees}")

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
        cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.environment}/floorPlan/floorPlan.jpg", config.floorPlan)
        cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.environment}/floorPlan/floorPlanFat.jpg", config.floorPlanFat)

    config.fullScanDone = True

    # TODO ask for room name
    saveMapInfo()

    imageName = buildImageName(depthCamX, depthCamY)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.environment}/floorPlan/floorPlanParts/fullScanPlan_{imageName}.jpg", config.fullScanPlan)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.environment}/floorPlan/floorPlanParts/fullScanPlanFat_{imageName}.jpg", config.fullScanPlanFat)

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
