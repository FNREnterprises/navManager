
import time
import numpy as np
#import rpyc
import cv2

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonCommandMethods

import config
import navMap


foundMarkers = []
rotheadMoveDuration = 800


def updateMarkerFoundResult(result, camera, cartX, cartY, cartDegrees, camDegrees):
    """
    :param result:  list of dict of  {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerDegrees'}
    :param camera: EYE_CAM or CART_CAM
    :param cartX:
    :param cartY:
    :param cartDegrees:
    :param camDegrees:
    :return:
    """

    for markerInfo in result:

        # evaluate marker x,y location based on cartLocation, distance and angles
        markerId = int(markerInfo['markerId'])
        degrees = markerInfo['angleToMarker'] + camDegrees      # relative to cart degrees
        distance = int(markerInfo['distanceCamToMarker'])
        markerDegrees = int(markerInfo['markerDegrees'])

        config.log(f"calcMarkerXY, cart: {cartX} / {cartY}, angleFromCart: {degrees}, dist: {distance}")

        xOffset = int(distance * np.cos(np.radians(int(degrees))))
        yOffset = int(distance * np.sin(np.radians(int(degrees))))

        # check whether the marker is already in the list
        appendMarker = False
        existingMarkerInfo = next((item for item in config.markerList if item.markerId == markerId), None)
        if existingMarkerInfo is None:
            appendMarker = True
            config.log(f"new marker found: {markerId}, distance: {distance}")
        else:
            # check for closer observation distance and use only the closest observation
            if existingMarkerInfo.distanceCamToMarker > distance:
                config.log(f"update marker information because of closer observation point, distance: {distance:.0f}")
                index = next(i for i, item in enumerate(config.markerList) if item.markerId == markerId)
                del config.markerList[index]
                appendMarker = True

        if appendMarker:
            oMarker = config.cMarker()
            oMarker.markerId = markerId
            oMarker.cameraType = camera
            oMarker.cartX = cartX
            oMarker.cartY = cartY
            oMarker.cartDegrees = cartDegrees
            oMarker.camDegrees = camDegrees     # eyecam: head degrees, cartcam = 0
            oMarker.atAngleFromCart = degrees
            oMarker.distanceCamToMarker = distance
            oMarker.markerX = cartX + xOffset
            oMarker.markerY = cartY + yOffset

            # a marker observation with distance > 1500 does not have a very accurate dockMarker.markerDegrees value
            # half the value in that case
            if distance > 1500:
                markerDegrees = round(markerDegrees/2)

            oMarker.markerDegrees = (cartDegrees + camDegrees + 90 + markerDegrees + 360) % 360

            config.markerList.append(oMarker)

            config.log(f"marker added: {oMarker.markerId}, X: {oMarker.markerX}, Y: {oMarker.markerY}, degrees: {oMarker.markerDegrees}")

    # persist list of markers
    navMap.saveMarkerList()


def scanWithHead(startDegrees, endDegrees, steps, lookForMarkers=None):

    if lookForMarkers is None:      # avoid mutable default value lookForMarkers=[]
        lookForMarkers = []

    # create a list of head rotations for the requested scan
    rotheadDegreesList = list(np.linspace(startDegrees, endDegrees, steps))

    # create a list of neck positions for each head yaw
    neckDegreesList = list(mg.neckAngles)

    # start with closer headdegrees
    if 'skeletonControl' in config.processSimulated:
        position = 90
    else:
        position = config.marvinShares.servoCurrentDict.get('head.rothead').degrees

    if position < 90:
        rotheadDegreesList.reverse()

    if 'skeletonControl' in config.processSimulated:
        position = 90
    else:
        position = config.marvinShares.servoCurrentDict.get('head.neck').degrees

    if position < 90:
        neckDegreesList.reverse()


    while len(rotheadDegreesList) > 0:

        requestedHeadYaw = int(rotheadDegreesList.pop())
        config.skeletonCommandMethods.requestDegrees(config.marvinShares.skeletonRequestQueue, config.processName, 'head.rothead', requestedHeadYaw, 250)
        time.sleep(0.2)     # make sure head is at full stop

        #cartX, cartY = config.getCartLocation()
        camDegrees = requestedHeadYaw
        cartDegrees = config.marvinShares.cartDict.get(mg.SharedDataItem.PLATFORM_IMU)['yaw']
        absDegreeImage = (cartDegrees + camDegrees) % 360


        # with same rothead position use a upper and lower neck position for marker scan
        for neckPos in neckDegreesList:

            # take depth only in rothead 0 position
            if requestedHeadYaw == 0 and neckPos['cam'] != mg.HEAD_CAM:
                continue

            #robotControl.servoMoveToDegreesBlocking('head.neck', neckDegreesList[i], 400)
            #robotHandling.servoMoveToDegrees('head.neck', neckPos['neckAngle'], 600)
            config.skeletonCommandMethods.requestDegrees(config.marvinShares.skeletonRequestQueue, config.processName,'head.neck', neckPos['neckAngles'], 250)
            # on the down move of the head we need more time for a stable picture
            #time.sleep(0.3)

            #
            if neckPos['cam'] == mg.EYE_CAM:
                config.log(f"take eyecam image: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")

                imageName = navMap.buildImageName(config.oCart.getCartX(), config.oCart.getCartY(), absDegreeImage, neckPos['neckAngle'])
                filename = f"{config.PATH_ROOM_DATA}/{config.room}/wallImages/{imageName}.jpg"

                config.waitForArucoEyeCam = True

                request = {'sender': config.processName, 'cmd': mg.ImageProcessingCommand.CHECK_FOR_ARUCO_CODE,
                           'cam': mg.EYE_CAM, 'markers': [], 'file': filename}

                config.marvinShares.imageProcessingQueue.put(request)

                timeout = time.time() + 2
                while config.waitForArucoEyeCam and time.time() < timeout:
                    time.sleep(0.1)

                if timeout:
                    return False



            if neckPos['cam'] == inmoovGlobal.HEAD_CAM:

                config.log(f"take depth image and find obstacles: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")

                if config.servers['cartControl'].simulated:
                    config.obstacleDistances = np.full(60,2000)     # simulate wall at distance 2 m
                else:
                    config.obstacleDistances = rpcSend.findObstacles(inmoovGlobal.HEAD_CAM)

                if config.obstacleDistances is None:
                    config.log(f"WARNING: could not evaluate obstacle distances, stop current task")
                    return False
                else:
                    config.log(f"set flag to process obstacleDistances")
                    config.flagAddObstaclesToMap = True

            neckDegreesList.reverse()       # continue with same neck position as in last rothead position

    config.log(f"scan with head done, degrees: {config.oCart.getCartYaw()}")
    return True



