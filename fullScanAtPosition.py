
import time
import numpy as np

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonClasses
from marvinglobal import cartClasses
from marvinglobal import skeletonCommandMethods

import config
import marker
import navMap


neckAngles = [{"neckAngle": mg.pitchWallWatchDegrees, "cam": mg.CamTypes.HEAD_CAM},
              {"neckAngle": 0, "cam": mg.CamTypes.EYE_CAM},
              {"neckAngle": -20, "cam": mg.CamTypes.EYE_CAM}]


def setupEyeCamViewDirection(self):
    """
    move head into position for taking wall pictures, make sure neck stays at position
    """
    config.log(f"set neck autoDetach to 5 sec")
    request = {'cmd': 'setAutoDetach', 'sender': config.processName,
               'servoName': 'head.neck', 'duration': 5000}
    config.marvinShares.skeletonRequestQueue.put(request)

    request = {'cmd': 'requestDegrees', 'sender': config.processName,
               'servoName': 'head.neck', 'degrees': mg.pitchWallWatchDegrees, 'duration': 1000, 'sequential': True}
    config.marvinShares.skeletonRequestQueue.put(request)

    request = {'cmd': 'requestDegrees', 'sender': config.processName,
               'servoName': 'head.rothead', 'duration': 1000}
    config.marvinShares.skeletonRequestQueue.put(request)


def scanWithHead(startDegrees, endDegrees, steps, lookForMarkers=None):

    if lookForMarkers is None:      # avoid mutable default value lookForMarkers=[]
        lookForMarkers = []

    # create a list of head rotations for the requested scan
    rotheadDegreesList = list(np.linspace(startDegrees, endDegrees, steps))

    # create a list of neck positions for each head yaw
    neckDegreesList = list(neckAngles)

    # before starting to loop over all rothead and neck positions find shorter way to start with
    rotheadCurrent:skeletonClasses.ServoCurrent = config.marvinShares.servoCurrentDict.get('head.rothead')
    if rotheadCurrent.degrees < 90:
        rotheadDegreesList.reverse()

    neckCurrent:skeletonClasses.ServoCurrent = config.marvinShares.servoCurrentDict.get('head.neck')
    if neckCurrent.degrees < 90:
        neckDegreesList.reverse()


    while len(rotheadDegreesList) > 0:

        requestedHeadYaw = int(rotheadDegreesList.pop())
        request = {'cmd': 'requestDegrees', 'sender': config.processName,
                   'servoName': 'head.rothead', 'degrees': requestedHeadYaw, 'duration': 250, 'sequential': True}
        config.marvinShares.skeletonRequestQueue.put(request)

        time.sleep(0.2)     # make sure head is at full stop and not shaking

        cartLocation:mg.Location = config.marvinShares.cartDict.get(mg.SharedDataItems.CART_LOCATION)
        camDegrees = requestedHeadYaw
        cartDegrees = cartLocation.yaw
        absDegreeImage = (cartDegrees + camDegrees) % 360

        # with same rothead position use a upper and lower neck position for marker scan
        for neckPos in neckDegreesList:

            # take depth only in rothead 0 position
            if requestedHeadYaw == 0 and neckPos['cam'] != mg.CamTypes.HEAD_CAM:
                continue

            request = {'cmd': 'requestDegrees', 'sender': config.processName,
                       'servoName': 'head.neck', 'degrees': neckPos['neckAngle'], 'duration': 600, 'sequential': True}
            config.marvinShares.skeletonRequestQueue.put(request)


            if neckPos['cam'] == mg.CamTypes.EYE_CAM:
                config.log(f"take eyecam image: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")

                imageName = navMap.buildImageName(cartLocation.x, cartLocation.y, absDegreeImage, neckPos['neckAngle'])
                filename = f"{config.PATH_ROOM_DATA}/{config.environment}/wallImages/{imageName}.jpg"

                request = {'cmd': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                           'cam': mg.CamTypes.EYE_CAM, 'imgPath': filename, 'show': True}
                config.marvinShares.imageProcessingQueue.put(request)

                # über rückmeldung an navManager lösen
                #if config.eyecamImage is None:
                #    config.log(f"WARNING: could not acquire an eyecam image, stop current task")
                #    return False


                config.log(f"check for marker with headdegrees: {int(requestedHeadYaw)}, cartDegrees: {config.oCart.getCartYaw()}")

                # über rückmeldung von imageProcessing an navManager lösen
                """
                markersFound = aruco.lookForMarkers("EYE_CAM", [], config.oHead.getHeadYaw())
                # result =  list of {'markerId', 'distanceCamToMarker', 'angleInImage', 'markerYaw'}

                if markersFound:
                    config.log(f"markers found: {markersFound}")
                    updateMarkerFoundResult(markersFound, 'EYE_CAM', config.oCart.getCartX(), config.oCart.getCartY(), cartDegrees, camDegrees)

                    #print(navGlobal.markerList)
                    # stop scan if we have found a requested marker
                    for markerInfo in markersFound:
                        if markerInfo['markerId'] in lookForMarkers:
                            config.log(f"requested marker found, markerId: {markerInfo['markerId']}")
                            break
                else:
                    config.log(f"no markers found")
                """

            if neckPos['cam'] == mg.CamTypes.HEAD_CAM:
                config.log(f"take depth image: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")

                request = {'cmd': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                           'cam': mg.CamTypes.HEAD_CAM, 'show': True}
                config.marvinShares.imageProcessingQueue.put(request)

                # über rückmeldung an sender lösen
                #if inmoovDepthImg is None:
                #    config.log(f"WARNING: could not acquire a depth image, stop current task")
                #    return False
                #else:
                #    config.log(f"TODO: do something with the depth image")

            neckDegreesList.reverse()       # continue with same neck position as in last rothead position

    config.log(f"scan with head done")


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

    startAngleCart = config.cartLocationLocal.yaw

    # move InMoov eye cam into capturing pose
    setupEyeCamViewDirection()

    # eval number of necessary cart rotations to get a full circle
    # use the fovH of the HEAD cam
    headFovH = mg.headCamProperties['fovH']
    numPlannedCartRotationSteps = int(360 / headFovH)
    cartRange = int(360 / (numPlannedCartRotationSteps + 1))

    # start with an empty scan plan
    config.fullScanPlan = np.zeros((navMap.MAP_WIDTH, navMap.MAP_HEIGHT), dtype=np.uint8)
    config.fullScanPlanFat = np.zeros_like(config.fullScanPlan)

    # for all orientations of the cart
    eyeFovH = mg.eyeCamProperties['fovH']
    numPlannedHeadRotationSteps = int(cartRange / eyeFovH)
    headRange = int(cartRange / (numPlannedHeadRotationSteps + 1))

    while numPlannedCartRotationSteps > 0:

        # request a cartcam picture from imageProcessing
        config.log(f"take cartcam image, degrees: {config.oCart.getCartYaw()}")
        request = {'cmd': mg.ImageProcessingCommands.TAKE_IMAGE, 'sender': config.processName,
                   'cam': mg.CamTypes.EYE_CAM}
        config.marvinShares.imageProcessingRequestQueue.put(request)


        # take several Eyecam images and one depth image with this cart orientation
        if scanWithHead(startDegrees= -cartRange/2 + headRange/2,
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
