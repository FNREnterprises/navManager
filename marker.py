
import time
import os
import numpy as np
import rpyc
import json
import cv2

import config
import rpcSend
import robotControl

rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True

neckAngles = (0,-30)    # 2 different angles in degrees

foundMarkers = []
rotheadMoveDuration = 800

def setupEyeCam():
    '''
    move head into position for taking wall pictures
    '''
    config.log(f"set neck autoDetach to 5 sec")
    robotControl.servoSetAutoDetach('head.neck', 5000)


def updateMarkerFoundResult(result, cartX, cartY, wallSection):
    """
    :param result:  list of dict of  {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerOrientation'}
    :param cartX:
    :param cartY:
    :param wallSection:
    :return:
    """

    for markerInfo in result:

        # evaluate marker x,y location based on cartLocation, distance and angles
        degrees = markerInfo['angleToMarker'] + wallSection
        distance = markerInfo['distanceCamToMarker']

        xOffset = distance * np.cos(np.radians(degrees))
        yOffset = distance * np.sin(np.radians(degrees))
        # TODO: there might be more than one marker observation from different locations, verify location?
        config.markerInfo.append({
            'markerId':         markerInfo['markerId'],
            'markerLocationX':  cartX + xOffset, 
            'markerLocationY':  cartY + yOffset, 
            'markerAngle':      markerInfo['markerOrientation']})

    # persist list of markers
    filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerInfo.json"
    with open(filename, "w") as write_file:
        json.dump(config.markerInfo, write_file)



def loadMarkerInfo():

    filename = f"{config.PATH_ROOM_DATA}/{config.room}/markerInfo.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            config.markerInfo = json.load(read_file)


def scanWithHead(startDegrees, endDegrees, steps, lookForMarkers=[]):

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
        wallSection = int((config.oCart.orientation + requestedHeadDegree) % 360)
        
        if config.navManagerServers['aruco']['simulated']:
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

                ####################
                # maybe limit amount of pics by checking for existing similar position/wallSection picture
                locX = round(config.oCart.x/50)*50
                locY = round(config.oCart.y/50)*50
                section = round(wallSection/5)*5
                filename = f"{config.PATH_ROOM_DATA}/{config.room}/wallImages/{locX:03}_{locY:03}_{section:03}_{neckDegreesList[i]}.jpg"
                cv2.imwrite(filename, inmoovEyecamImg)
                config.log(f"wallImage: {filename}")
                ####################

                config.log(f"check for marker with headYaw: {int(requestedHeadDegree)}, cartOrientation: {config.oCart.orientation}")

                if not config.navManagerServers['aruco']['simulated']:

                    markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
                    # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerOrientation'}

                    if markersFound:
                        config.log(f"markers found: {result}")
                        updateMarkerFoundResult(result, config.oCart.x, config.oCart.y, wallSection)

                        #print(navGlobal.markerInfo)
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

    config.log(f"scan with head done, orientation: {config.oCart.orientation}")
    return True



def checkWhileApproachingMarker(markerId):

    ''' 
    when a marker has been found the cart movement may take it out of the cam focus
    check current and adjecent images
    when marker is to the right of the cart check to the left, otherwise to the right
    '''
    # TODO needs to be verified, does not look right
    markerFound, result = rpcSend.lookForMarkers("EYE_CAM", [markerId])
    if markerFound:
        config.log(f"marker found, markerId: {config.markerInfo.id}, "
                   f"distance: {config.markerInfo.distance:{5}.{2}}, "
                   f"angleToMarker: {config.markerInfo.angleToMarker:{5}.{2}}, "
                   f"yawToCartTarget: {config.markerInfo.yawToCartTarget:{5}.{2}}, "
                   f"distToCartTarget: {config.markerInfo.distToCartTarget:{5}.{2}}")

