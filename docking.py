
import numpy as np
from pyrecord import Record
import cv2

import config
import robotControl
import rpcSend
import navManager
import navMap
import cartHandling

point = Record.create_type('point','x','y')

SIDE_MOVE_SPEED = 150
DOCKING_SPEED = 60
DOCK_MARKER_ID = 10
DOCK_DETAIL_MARKER_ID = 11
DOCKING_START_DISTANCE_FROM_MARKER = 600
DISTANCE_CART_CENTER_CAM = 330
MARKER_XOFFSET_CORRECTION = -15     # distance docking cave - docking detail marker might not be equal to distance docking fingers - cartcam


def tryToDock(dockInfo):

    # check for dock detail marker (11) in image
    result = config.servers["aruco"].root.findDockingDetailMarker(config.DOCKING_DETAIL_ID)

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
    for m in config.markerList:
        if m.markerId == DOCK_MARKER_ID:
            return m
    return None


def moveToDockingStartPosition(dockMarker):

    try:
        # try to move cart to a position orthogonal in front of the dockingMarker
        xCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.cos(np.radians(dockMarker.markerDegrees + 90))
        yCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.sin(np.radians(dockMarker.markerDegrees + 90))
        config.oTarget.x = dockMarker.markerX + xCorr
        config.oTarget.y = dockMarker.markerY + yCorr

        # find angle from current cart position to cart target position
        xDiff = float(config.oTarget.x) - float(config.oCart.getX())
        yDiff = float(config.oTarget.y) - float(config.oCart.getY())

        # angle = arctan(yDiff/xDiff)
        absDegree = np.degrees(np.arctan(yDiff/xDiff))
        config.log(f"dock, targetX: {config.oTarget.x:.0f}, targetY: {config.oTarget.y:.0f}, absDegree: {absDegree:.1f}")

        cartRotation = round((absDegree - config.oCart.getDegrees()) % 360)
        distance = np.hypot(xDiff, yDiff)
        if distance < 100:
            config.log(f"cart is close to docking detail start point, do not move cart")
            if abs(cartRotation) > 4:
                config.log(f"only rotate cart by {cartRotation} degrees to face marker")
                robotControl.rotateCartRelative(cartRotation, 180)

        else:
            # move cart to docking detail position
            config.log(f"dock, cartDegrees: {config.oCart.getDegrees()}, absDegrees marker: {absDegree:.2f}, cartRotation: {cartRotation}, distance: {distance:.0f}")

            # visualise docking move
            dockMap = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)
            navMap.addCenter(dockMap)
            navMap.addCart(dockMap)
            navMap.addMarker(dockMap, dockMarker.markerX, dockMarker.markerY, dockMarker.markerDegrees)
            navMap.addTarget(dockMap)
            navMap.addPathToTarget(dockMap)     # line from cartPosition to targetPosition

            cv2.imshow("docking", dockMap)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # for a short move we can use the 4 main cart directions
            if distance < 500:
                allowedMoves=[config.Direction.FORWARD, config.Direction.BACKWARD, config.Direction.LEFT, config.Direction.RIGHT]
            else:   # otherwise force to move forward
                allowedMoves=[config.Direction.FORWARD]
            moveSuccess = robotControl.moveCart(absDegree, distance, 200, allowedMoves)

            if not moveSuccess:
                navManager.setTask("notask")
                return False

        # rotate cart to face marker, avoid a tiny rotation
        markerFacedegrees = dockMarker.markerDegrees - 90
        difference = markerFacedegrees - config.oCart.getDegrees()
        endRotation = -robotControl.signedAngleDifference(difference, 0)
        config.log(f"rotate cart to face marker, markerFacedegrees: {markerFacedegrees}, endRotation: {endRotation}")
        if abs(endRotation) > 2:
            robotControl.rotateCartRelative(endRotation, 180)
        return True

    except config.CartError as e:
        config.log(f"cart exception raised: {e}")
        navManager.setTask('notask')


def alignWithMarker(sideMoveOnly=False):
    """
    a loop of rotation and side moves to position cart directly in front of marker
    for rotation use a reduced part of the optically evaluated markerDegrees value
    :param sideMoveOnly:
    :return:
    """

    markerDegreesRotationThreshold = 2      # do not rotate if measured degrees is below threshold
    markerDegrees = markerDegreesRotationThreshold + 1  # make sure to enter alignment loop
    sideMoveThreshold = 15              # do not move sideways if measured offset is below threshold
    sideMove = sideMoveThreshold + 1    # make sure to enter alignment loop

    # verify that cart is aligned with marker
    # for docking this means we should align cart front with markerDegrees
    distance = None
    while abs(markerDegrees) > markerDegreesRotationThreshold or abs(sideMove) > sideMoveThreshold:

        if not sideMoveOnly and abs(markerDegrees) > markerDegreesRotationThreshold:
            navMap.takeCartcamImage(show=True)
            markerFound, markerInfo = rpcSend.lookForMarkers("CART_CAM", [DOCK_DETAIL_MARKER_ID])

            if markerFound:

                # partially rotate to compensate for markerkdegrees
                # 2.4.2019 using calculated markerDegrees caused overshooting or even loss of marker in cam frame
                # as we looked for DOCK_DETAIL_MARKER only use first result ([0])
                markerDegrees = markerInfo[0]['markerDegrees'] / 2
                config.log(f"first rotate cart to be orthogonal with marker {markerDegrees}")
                if abs(markerDegrees) > markerDegreesRotationThreshold:
                    cartHandling.moveCart(markerDegrees, 0, 180)
                    #robotControl.rotateCartRelative(markerDegrees, 180)

        if abs(sideMove) > sideMoveThreshold:

            navMap.takeCartcamImage(show=True)
            markerFound, markerInfo = rpcSend.lookForMarkers("CART_CAM", [DOCK_DETAIL_MARKER_ID])

            if markerFound:

                # as we looked for DOCK_DETAIL_MARKER only use first result ([0])
                angle = markerInfo[0]['angleToMarker']
                distance = markerInfo[0]['distanceCamToMarker']
                sideMove = distance * np.sin(np.radians(angle))
                config.log(f"move sideways to be directly in front of marker, angle: {angle}, distance: {distance}, sideMove: {sideMove:.0f}")

                if sideMove > sideMoveThreshold:
                    cartHandling.moveCartWithDirection(config.Direction.LEFT, abs(sideMove), SIDE_MOVE_SPEED, kinectMonitoring=False)

                if sideMove < -sideMoveThreshold:
                    cartHandling.moveCartWithDirection(config.Direction.RIGHT, abs(sideMove), SIDE_MOVE_SPEED, kinectMonitoring=False)

    return distance


def detailDockMoves(dockInfo):

    # try to align cart with marker
    distance = alignWithMarker()
    if distance is None:
        config.log(f"lost marker during detailDockMoves, docking failed in step 1")
        navManager.setTask('notask')
        return

    # we should be fairly aligned now and in front of the dock
    # move closer to dock
    moveThreshold = 20
    moveDistance = distance - 300
    if abs(moveDistance) > moveThreshold:
        #robotControl.moveCartWithDirection(config.Direction.FORWARD, moveDistance, DOCKING_SPEED, moveThreshold, kinectMonitoring=False)
        cartHandling.moveCart(config.oCart.getDegrees(), moveDistance, DOCKING_SPEED, kinectMonitoring=False)

    # step closer and verify distance
    distance = alignWithMarker()
    if distance is None:
        config.log(f"lost marker during detailDockMoves, docking failed in step2")
        navManager.setTask('notask')
        return

    moveDistance = distance - 200
    if abs(moveDistance) > moveThreshold:
        robotControl.moveCartWithDirection(config.Direction.FORWARD, moveDistance, DOCKING_SPEED, moveThreshold, kinectMonitoring=False)

    # now try to dock, arduino watches docking switch and stops cart when activated
    distance = alignWithMarker(sideMoveOnly=True)
    if distance is None:
        config.log(f"lost marker during detailDockMoves, docking failed in step3")
        navManager.setTask('notask')
        return

    moveDistance = distance - 100
    if abs(moveDistance) > moveThreshold:
        robotControl.moveCartWithDirection(config.Direction.FORWARD, moveDistance, DOCKING_SPEED, moveThreshold, kinectMonitoring=False)

    navManager.setTask('notask')



def dock(show=True):
    """
    request for docking
    """
    try:
        dockMarker = getDockingMarkerInfo()
        #{'markerId','distance','markerLocationX','markerLocationY','markerAngle'}
        if dockMarker is None:
            config.log(f"no docking station detected yet")
            navManager.setTask("notask")
            return

        moveToDockingStartPosition(dockMarker)
        config.log(f"move to docking start position")

    except Exception as e:
        config.log(f"{e}")