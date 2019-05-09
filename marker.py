
import time
import numpy as np
import rpyc
import cv2

import config
import rpcSend
import robotControl
import navMap

rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True

neckAngles = (0,-30)    # 2 different angles in degrees

foundMarkers = []
rotheadMoveDuration = 800

def setupEyeCam():
    """
    move head into position for taking wall pictures
    """
    config.log(f"set neck autoDetach to 5 sec")
    robotControl.servoSetAutoDetach('head.neck', 5000)


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
    neckDegreesList = list(neckAngles)

    # start with closer headdegrees
    _, position, _ = robotControl.servoGetPosition('head.rothead')
    if position < 90:
        rotheadDegreesList.reverse()

    _, position, _ = robotControl.servoGetPosition('head.neck')
    if position < 90:
        neckDegreesList.reverse()


    while len(rotheadDegreesList) > 0:

        requestedHeadDegree = int(rotheadDegreesList.pop())
        #robotControl.sendMrlCommand("i01.head.rothead", "moveToBlocking", str(requestedHeaddegrees))
        robotControl.servoMoveToDegreesBlocking('head.rothead', requestedHeadDegree, 250)
        time.sleep(0.2)     # make sure head is at full stop

        #cartX, cartY = config.getCartLocation()
        camDegrees = requestedHeadDegree
        cartDegrees = config.oCart.getDegrees()
        absDegreeImage = (cartDegrees + camDegrees) % 360

        if config.servers['aruco'].simulated:
            inmoovEyecamImg = np.zeros([25, 25, 3], dtype=np.uint8)
            markerList = []
        else:

            for i in range(2):

                robotControl.servoMoveToDegreesBlocking('head.neck', neckDegreesList[i], 400)
                # on the down move of the head we need more time for a stable picture
                time.sleep(0.3)

                config.log(f"take eyecam image: rothead: {requestedHeadDegree}, neck: {neckDegreesList[i]}")
                inmoovEyecamImg = rpcSend.getEyecamImage()

                if inmoovEyecamImg is None:
                    config.log(f"could not acquire an eyecam image, stop current task")
                    return False

                imageName = navMap.buildImageName(config.oCart.getX(), config.oCart.getY(), absDegreeImage, neckDegreesList[i])
                filename = f"{config.PATH_ROOM_DATA}/{config.room}/wallImages/{imageName}.jpg"
                cv2.imwrite(filename, inmoovEyecamImg)
                config.log(f"wallImage: {filename}")

                config.log(f"check for marker with headdegrees: {int(requestedHeadDegree)}, cartDegrees: {config.oCart.getDegrees()}")

                if not config.servers['aruco'].simulated:

                    markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
                    # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerDegrees'}

                    if markersFound:
                        config.log(f"markers found: {result}")
                        updateMarkerFoundResult(result, 'EYE_CAM', config.oCart.getX(), config.oCart.getY(), cartDegrees, camDegrees)

                        #print(navGlobal.markerList)
                        # stop scan if we have found a requested marker
                        for markerInfo in result:
                            if markerInfo['markerId'] in lookForMarkers:
                                config.log(f"requested marker found, markerId: {markerInfo['markerId']}")
                                break
                    else:
                        config.log(f"no markers found")

            neckDegreesList.reverse()       # continue with same neck position as in last rothead position
            #cv2.imshow("inmoovEyeCamImg", inmoovEyecamImg)
            #cv2.waitKey()

    config.log(f"scan with head done, degrees: {config.oCart.getDegrees()}")
    return True



