
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
        foundMarkers.append({
            'markerId':         markerInfo['markerId'],
            'markerLocationX':  cartX + xOffset, 
            'markerLocationY':  cartY + yOffset, 
            'markerAngle':      markerInfo['markerOrientation']})

    # persist list of markers
    filename = f"{config.PATH_ROOM_DATA}/{config._room}/markerInfo.json"
    with open(filename, "w") as write_file:
        json.dump(foundMarkers, write_file)



def loadMarkerInfo():

    global foundMarkers

    filename = f"{config.PATH_ROOM_DATA}/{config._room}/markerInfo.json"
    if os.path.exists(filename):
        with open(filename, "r") as read_file:
            foundMarkers = json.load(read_file)
    else:
        foundMarkers = []



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

    # avoid detaching neck too soon after move
    robotControl.servoSetAutoDetach("head.neck", 2000)
    #robotControl.servoMoveToDegreesBlocking('head.neck', neckAngles[0], 200)

    time.sleep(0.3)

    while len(rotheadDegreesList) > 0:

        requestedHeadDegree = int(rotheadDegreesList.pop())
        #robotControl.sendMrlCommand("i01.head.rothead", "moveToBlocking", str(requestedHeadYaw))
        robotControl.servoMoveToDegreesBlocking('head.rothead', requestedHeadDegree, 500)
        time.sleep(0.2)     # make sure head is at full stop

        cartX, cartY = config.getCartLocation()
        wallSection = int((config.getCartOrientation() + requestedHeadDegree - 90) % 360)
        
        if rpcSend.navManagerServers['aruco']['simulated']:
            inmoovEyecamImg = np.zeros([25, 25, 3], dtype=np.uint8)
            markerList = []
        else:

            for i in range(2):

                robotControl.servoMoveToDegreesBlocking('head.neck', neckDegreesList[i],200)
                time.sleep(0.3)

                config.log(f"take eyecam image: rothead: {requestedHeadDegree}, neck: {neckDegreesList[i]}")
                inmoovEyecamImg = rpcSend.getEyecamImage()

                if inmoovEyecamImg is None:
                    config.log("could not aquire an eyecam image, stop current task")
                    return False

                ####################
                # maybe limit amount of pics by checking for existing similar position/wallSection picture
                filename = f"{config.PATH_ROOM_DATA}/{config._room}/wallImages/{cartX:03}_{cartY:03}_{wallSection:03}_{neckDegreesList[i]}.jpg"
                cv2.imwrite(filename, inmoovEyecamImg)
                ####################

                config.log(f"check for marker with headYaw: {int(requestedHeadDegree)}")

                if not rpcSend.navManagerServers['aruco']['simulated']:

                    markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
                    # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerOrientation'}

                    if markersFound:
                        updateMarkerFoundResult(result, cartX, cartY, wallSection)

                        #print(navGlobal.markerInfo)
                        # stop scan if we have found a requested marker
                        for markerInfo in result:
                            if markerInfo['markerId'] in lookForMarkers:
                                config.log(f"requested marker found, markerId: {markerInfo['markerId']}")
                                break

            neckDegreesList.reverse()       # continue with same neck position as in last rothead position
            #cv2.imshow("inmoovEyeCamImg", inmoovEyecamImg)
            #cv2.waitKey()

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

