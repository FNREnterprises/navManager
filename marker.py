
import time
import numpy as np
import rpyc
import cv2

import inmoovGlobal
import config
import rpcSend
import robotHandling
import navMap
import aruco


rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True

neckAngles = [{"neckAngle": inmoovGlobal.pitchWallWatchDegrees, "cam": inmoovGlobal.HEAD_DEPTH},
              {"neckAngle": 0, "cam": inmoovGlobal.EYE_CAM},
              {"neckAngle": -20, "cam": inmoovGlobal.EYE_CAM}]


foundMarkers = []
rotheadMoveDuration = 800

def setupEyeCam():
    """
    move head into position for taking wall pictures
    """
    config.log(f"set neck autoDetach to 5 sec")
    robotHandling.servoSetAutoDetach('head.neck', 5000)


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
    neckDegreesList = list(neckAngles)

    # start with closer headdegrees
    _, position, _ = robotHandling.servoGetPosition('head.rothead')
    if position < 90:
        rotheadDegreesList.reverse()

    _, position, _ = robotHandling.servoGetPosition('head.neck')
    if position < 90:
        neckDegreesList.reverse()


    while len(rotheadDegreesList) > 0:

        requestedHeadYaw = int(rotheadDegreesList.pop())
        #robotControl.sendMrlCommand("i01.head.rothead", "moveToBlocking", str(requestedHeaddegrees))
        #robotHandling.servoMoveToDegreesBlocking('head.rothead', requestedHeadYaw, 250)
        robotHandling.servoMoveToDegrees('head.rothead', requestedHeadYaw, 250)
        time.sleep(0.2)     # make sure head is at full stop

        #cartX, cartY = config.getCartLocation()
        camDegrees = requestedHeadYaw
        cartDegrees = config.oCart.getCartYaw()
        absDegreeImage = (cartDegrees + camDegrees) % 360


        # with same rothead position use a upper and lower neck position for marker scan
        for neckPos in neckDegreesList:

            # take depth only in rothead 0 position
            if requestedHeadYaw == 0 and neckPos['cam'] != inmoovGlobal.HEAD_DEPTH:
                continue

            #robotControl.servoMoveToDegreesBlocking('head.neck', neckDegreesList[i], 400)
            robotHandling.servoMoveToDegrees('head.neck', neckPos['neckAngle'], 600)
            # on the down move of the head we need more time for a stable picture
            #time.sleep(0.3)

            #
            if neckPos['cam'] == inmoovGlobal.EYE_CAM:
                config.log(f"take eyecam image: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")
                config.eyecamImage = rpcSend.getImage(inmoovGlobal.EYE_CAM)

                if config.eyecamImage is None:
                    config.log(f"WARNING: could not acquire an eyecam image, stop current task")
                    return False

                imageName = navMap.buildImageName(config.oCart.getCartX(), config.oCart.getCartY(), absDegreeImage, neckDegreesList[i]['neckAngle'])
                filename = f"{config.PATH_ROOM_DATA}/{config.room}/wallImages/{imageName}.jpg"
                cv2.imwrite(filename, config.eyecamImage)
                config.log(f"wallImage: {filename}")

                config.log(f"check for marker with headdegrees: {int(requestedHeadYaw)}, cartDegrees: {config.oCart.getCartYaw()}")

                markersFound = aruco.lookForMarkers("EYE_CAM", [], config.oHead.getHeadYaw())
                # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerDegrees'}

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


            if neckPos['cam'] == inmoovGlobal.HEAD_DEPTH:
                config.log(f"take depth image: rothead: {requestedHeadYaw}, neck: {neckPos['neckAngle']}")
                inmoovDepthImg = rpcSend.getImage(inmoovGlobal.HEAD_DEPTH)

                if inmoovDepthImg is None:
                    config.log(f"WARNING: could not acquire a depth image, stop current task")
                    return False
                else:
                    config.log(f"TODO: do something with the depth image")

            neckDegreesList.reverse()       # continue with same neck position as in last rothead position

    config.log(f"scan with head done, degrees: {config.oCart.getCartYaw()}")
    return True



