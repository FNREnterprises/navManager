
import numpy as np
from pyrecord import Record
import cv2

import config
import robotControl
import navManager
import navMap

point = Record.create_type('point','x','y')

SIDE_MOVE_SPEED = 150
DOCK_MARKER_ID = 10
DOCK_DETAIL_MARKER_ID = 11
DOCKING_START_DISTANCE_FROM_MARKER = 500
DISTANCE_CART_CENTER_CAM = 330
MARKER_XOFFSET_CORRECTION = -15     # distance docking cave - docking detail marker might not be equal to distance docking fingers - cartcam


def tryToDock(dockInfo):

    # check for dock detail marker (11) in image
    result = config.navManagerServers["aruco"].root.findDockingDetailMarker(config.DOCKING_DETAIL_ID)

    success = bool(result[0])
    if success:
        rotation = int(result[1])
        xOffset = int(result[2])
        distance = int(result[3])
    else:
        rotation = 0
        xOffset = 0
        distance = 0
        raise(config.ArucoError(config.DOCKING_DETAIL_ID))

    config.log(f"evalDockingDetailMarkerPosition, success: {success}, rotation: {rotation:.0f}, xOffset: {xOffset:.0f}, distance: {distance:.0f}")

    #return success, rotation, xOffset, distance
    return rotation, xOffset, distance


def getDockingMarkerInfo():
    """
    return info about docker marker location or None
    :return:
    """
    for m in config.markerInfo:
        if m['markerId'] == DOCK_MARKER_ID:
            return m
    return None


def moveToDockingStartPosition(dockInfo):

    try:
        # try to move cart to a position orthogonal in front of the dockingMarker
        xCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.cos(np.radians(180-dockInfo['markerAngle']))
        yCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.sin(np.radians(180-dockInfo['markerAngle']))
        config.oTarget.x = dockInfo['markerLocationX'] - xCorr
        config.oTarget.y = dockInfo['markerLocationY'] + yCorr

        # find angle from current cart position to cart target position
        xDiff = float(config.oTarget.x) - float(config.oCart.getX())
        yDiff = float(config.oTarget.y) - float(config.oCart.getY())

        # angle = arctan(yDiff/xDiff)
        absDegree = np.degrees(np.arctan(yDiff/xDiff))
        config.log(f"dock, targetX: {config.oTarget.x:.0f}, targetY: {config.oTarget.y:.0f}, absDegree: {absDegree:.1f}")

        cartRotation = round((absDegree - config.oCart.getYaw()) % 360)
        distance = np.hypot(xDiff, yDiff)
        config.log(f"dock, cartYaw: {config.oCart.getYaw()}, absDegrees marker: {absDegree:.2f}, cartRotation: {cartRotation}, distance: {distance:.0f}")

        dockMap = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)
        navMap.addCenter(dockMap)
        navMap.addCart(dockMap)
        navMap.addMarker(dockMap, dockInfo['markerLocationX'], dockInfo['markerLocationY'], dockInfo['markerAngle'])
        navMap.addTarget(dockMap)
        navMap.addPathToTarget(dockMap)     # line from cartPosition to targetPosition

        cv2.imshow("docking", dockMap)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #distance = distance - 300   # reduce distance temporarily for safety reasons

        moveSuccess = robotControl.moveCart(absDegree, distance, 200)

        if not moveSuccess:
            navManager.setTask("notask")
            return False

        # rotate cart to face marker
        markerFaceYaw = dockInfo['markerAngle'] - 180
        rotation = markerFaceYaw - config.oCart.getYaw()
        robotControl.rotateCartRelative(rotation, 180)
        return True

    except config.CartError as e:
        config.log(f"cart exception raised: {e}")


def tryToDock(dockInfo):
    pass


def dock(show=True):
    """
    request for docking
    """
    try:
        dockInfo = getDockingMarkerInfo()
        #{'markerId','distance','markerLocationX','markerLocationY','markerAngle'}
        if dockInfo is None:
            config.log(f"no docking station detected yet")
            navManager.setTask("notask")
            return

        if moveToDockingStartPosition(dockInfo):
            tryToDock(dockInfo)


        config.log(f"dock, markerX: {dockInfo['markerLocationX']}, markerY: {dockInfo['markerLocationY']}, markerYaw: {dockInfo['markerAngle']:.0f}")

    except Exception as e:
        config.log(f"{e}")