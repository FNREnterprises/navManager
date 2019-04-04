
import time
import os
import numpy as np
import rpyc
import _pickle as pickle
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


def updateMarkerFoundResult(result, camera, cartX, cartY, camYaw):
    """
    :param result:  list of dict of  {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerYaw'}
    :param cartX:
    :param cartY:
    :param camYaw:
    :return:
    """

    for markerInfo in result:

        # evaluate marker x,y location based on cartLocation, distance and angles
        markerId = int(markerInfo['markerId'])
        degrees = markerInfo['angleToMarker'] + camYaw
        distance = int(markerInfo['distanceCamToMarker'])
        markerYaw = int(markerInfo['markerYaw'])

        xOffset = int(distance * np.cos(np.radians(int(degrees))))
        yOffset = int(distance * np.sin(np.radians(int(degrees))))

        # check whether the marker is already in the list
        appendMarker = False
        entry = next((item for item in config.markerList if item.markerId == markerId), None)
        if entry is None:
            appendMarker = True
            config.log(f"new marker found: {markerId}, distance: {distance}")
        else:
            # check for closer observation distance and use only the closest observation
            if entry.distanceCamToMarker > distance:
                config.log(f"update marker information because of closer observation point, distance: {distance:.0f}")
                index = next(i for i, item in enumerate(config.markerList) if item.markerId == markerId)
                del config.markerList[index]
                appendMarker = True

        if appendMarker:
            oMarker = config.cMarker()
            oMarker.markerId = markerId
            oMarker.cameraType = camera
            oMarker.camX = cartX
            oMarker.camY = cartY
            oMarker.camYaw = camYaw     # cart yaw + head yaw
            oMarker.angleInImage = degrees
            oMarker.distanceCamToMarker = distance
            oMarker.markerX = cartX + xOffset
            oMarker.markerY = cartY + yOffset
            oMarker.markerYaw = (camYaw + 90 + markerYaw) % 360

            config.markerList.append(oMarker)

    # persist list of markers
    navMap.saveMarkerList()


def scanWithHead(startDegrees, endDegrees, steps, lookForMarkers=None):

    if lookForMarkers is None:      # avoid mutable default value lookForMarkers=[]
        lookForMarkers = []

    # create a list of head rotations for the requested scan
    rotheadDegreesList = list(np.linspace(startDegrees, endDegrees, steps))
    neckDegreesList = list(neckAngles)

    # start with closer headYaw
    _, position, _ = robotControl.servoGetPosition('head.rothead')
    if position < 90:
        rotheadDegreesList.reverse()

    _, position, _ = robotControl.servoGetPosition('head.neck')
    if position < 90:
        neckDegreesList.reverse()


    while len(rotheadDegreesList) > 0:

        requestedHeadDegree = int(rotheadDegreesList.pop())
        #robotControl.sendMrlCommand("i01.head.rothead", "moveToBlocking", str(requestedHeadYaw))
        robotControl.servoMoveToDegreesBlocking('head.rothead', requestedHeadDegree, 250)
        time.sleep(0.2)     # make sure head is at full stop

        #cartX, cartY = config.getCartLocation()
        camYaw = int((config.oCart.getYaw() + requestedHeadDegree) % 360)
        
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

                imageName = navMap.buildImageName(config.oCart.getX(), config.oCart.getY(), camYaw, neckDegreesList[i])
                filename = f"{config.PATH_ROOM_DATA}/{config.room}/wallImages/{imageName}.jpg"
                cv2.imwrite(filename, inmoovEyecamImg)
                config.log(f"wallImage: {filename}")

                config.log(f"check for marker with headYaw: {int(requestedHeadDegree)}, cartOrientation: {config.oCart.getYaw()}")

                if not config.servers['aruco'].simulated:

                    markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
                    # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerYaw'}

                    if markersFound:
                        config.log(f"markers found: {result}")
                        updateMarkerFoundResult(result, 'EYE_CAM', config.oCart.getX(), config.oCart.getY(), camYaw)

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

    config.log(f"scan with head done, orientation: {config.oCart.getYaw()}")
    return True



def checkWhileApproachingMarker(markerId):

    """
    when a marker has been found the cart movement may take it out of the cam focus
    check current and adjecent images
    when marker is to the right of the cart check to the left, otherwise to the right
    """
    # TODO needs to be verified, does not look right
    markerFound, result = rpcSend.lookForMarkers("EYE_CAM", [markerId])
    if markerFound:
        config.log(f"marker found, markerId: {config.markerInfo.id}, "
                   f"distance: {config.markerInfo.distance:{5}.{2}}, "
                   f"angleToMarker: {config.markerInfo.angleToMarker:{5}.{2}}, "
                   f"yawToCartTarget: {config.markerInfo.yawToCartTarget:{5}.{2}}, "
                   f"distToCartTarget: {config.markerInfo.distToCartTarget:{5}.{2}}")

