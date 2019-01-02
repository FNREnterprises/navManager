
import time
import numpy as np
from pyrecord import Record

import config
import robotControl

point = Record.create_type('point','x','y')

SIDE_MOVE_SPEED = 150

'''
def evalDockingDetailMarkerPosition():

    # check for dock detail marker (11) in image
    result = navGlobal.arucoRequest.root.findDockingDetailMarker(navGlobal.DOCKING_DETAIL_ID)

    success = bool(result[0])
    if success:
        rotation = int(result[1])
        xOffset = int(result[2])
        distance = int(result[3])
    else:
        rotation = 0
        xOffset = 0
        distance = 0
        raise(ArucoError(navGlobal.DOCKING_DETAIL_ID))

    navGlobal.log(f"evalDockingDetailMarkerPosition, success: {success}, rotation: {rotation:.0f}, xOffset: {xOffset:.0f}, distance: {distance:.0f}")

    #return success, rotation, xOffset, distance
    return rotation, xOffset, distance
'''



"""
def dock():
    '''
    request for docking
    check for known docking station position in room
    if not scan room for docking station
    '''
    if config.getDockingMarkerPosition() is None:
        config.setTask("searchDockingMarker"
    else:
        # try to move cart to a position orthogonal in front of the dockingMarker
        markerPos, markerOrientation = config.getDockingMarkerPosition()

        xCorr = config.DOCKING_START_DISTANCE_FROM_MARKER * np.cos(np.radians(markerOrientation))
        yCorr = config.DOCKING_START_DISTANCE_FROM_MARKER * np.sin(np.radians(markerOrientation))
        cartTargetPos = config.Point2D(markerPos.x + xCorr, markerPos.y - yCorr)

    config.log(f"assume we are currently at a position showing the docking station marker")

    # start with a request for the aruco task to take a cartcam image
    markerFound, distanceCamToMarker, xOffsetMarker, markerYawDegrees = lookForMarker("CART_CAM", config.DOCKING_MARKER_ID)

    if markerFound:
        cartMovesDockingPhase1(distanceCamToMarker, xOffsetMarker, markerYawDegrees)
       
    else:
        config.log(f"docking marker not found")
        config.setTask("notask")
        return
"""

def cartMovesDockingPhase1(distanceCamToMarker, xOffsetMarker, markerYawDegrees):

    ORTHO_STOP_DISTANCE = 200 + config.DISTANCE_CART_CENTER_CAM      # cart front 20 cm in front of marker

    config.log(f"docking phase 1 - distanceCamToMarker: {distanceCamToMarker:.0f}, xOffsetMarker {xOffsetMarker:.0f}, markerYawDegrees {markerYawDegrees:.0f}")

    # in docking phase 1 we try to position the cart orthogonal in front of the docking marker

    cartCenterPos = point(0,0)
    camPos = point(0, config.DISTANCE_CART_CENTER_CAM)

    # angleCartMarker = sin(opposite <xOffset> / hypot <distanceCamMarker>)
    angleCartMarker = np.degrees(np.sin(xOffsetMarker/distanceCamToMarker))

    # calculate marker position relativ to cart center (cartcam looks straight out)
    markerCenterPos = point(xOffsetMarker, camPos.y + np.cos(np.radians(angleCartMarker)) * distanceCamToMarker)
    config.log(f"angleCartMarker: {angleCartMarker:.1f}, markerCenterPos(relative to cart center): {markerCenterPos.x:.0f} / {markerCenterPos.y:.0f}")

    xCorr = ORTHO_STOP_DISTANCE * np.sin(np.radians(markerYawDegrees))
    yCorr = ORTHO_STOP_DISTANCE * np.cos(np.radians(markerYawDegrees))
    config.log(f"markerYaw[degrees]: {markerYawDegrees:.1f}, xCorr: {xCorr:.0f}, yCorr: {yCorr:.0f}")

    # cart target position is marker's orthogonal point at distance
    # use the offset to account for the x-difference of the docking detail marker center vs the docking marker center
    config.oTarget.x = markerCenterPos.x + xCorr + config.MARKER_XOFFSET_CORRECTION
    config.oTarget.y = markerCenterPos.y - yCorr

    config.log(f"cartTargetPos (cartCenter) = {config.oTarget.x:.0f},{config.oTarget.y:.0f}")

    cartStartRotation = np.degrees(np.arctan((config.oTarget.x - cartCenterPos.x) / (config.oTarget.y - cartCenterPos.y)))
    cartMove = 0.9 * np.hypot((config.oTarget.x - cartCenterPos.x), (config.oTarget.y - cartCenterPos.y))     # cart moved too far
    cartEndRotation = -(np.degrees(np.arctan(xCorr / yCorr)) + cartStartRotation)

    config.log(f"cartStartRotation: {cartStartRotation:.0f}, cartMove: {cartMove:.0f}, cartEndRotation: {cartEndRotation:.0f}")

    try:
        # start with initial rotation
        if abs(cartStartRotation) > 1:
            robotControl.rotateCartRelative(cartStartRotation, 150)      # blocking, raises cartException on failure

        robotControl.moveCartWithDirection(robotControl.FORWARD, cartMove, 180)    # blocking, raises cartException on failure

        if abs(cartEndRotation) > 1:
            robotControl.rotateCartRelative(cartEndRotation, 150)   # blocking, raises cartException on failure

        config.log(f"end rotation done")
        #navGlobal.setTask("dockingPhase2")

        # we should see the docking detail marker now, if not move sideways and try to find it
        for i in range(2):

            markerFound, distanceCamToMarker, xOffseetMarker, markerYawDegrees = lookForMarker("CART_CAM", config.DOCKING_DETAIL_ID)

            if markerFound:
                config.setTask("dockingPhase2")
                config.setTask("notask")
                return
            else:
                robotControl.moveCartWithDirection(robotControl.CART_MOVE_LEFT, 70, SIDE_MOVE_SPEED)

        robotControl.moveCartWithDirection(robotControl.CART_MOVE_RIGHT, 280, SIDE_MOVE_SPEED)

        for i in range(1):

            markerFound, distanceCamToMarker, xOffsetMarker, markerYawDegrees = lookForMarker("CART_CAM", config.DOCKING_DETAIL_ID)

            if markerFound:
                config.setTask("dockingPhase2")
                config.setTask("notask")
                return
            else:
                robotControl.moveCartWithDirection(robotControl.CART_MOVE_RIGHT, 70, SIDE_MOVE_SPEED)

        config.log(f"docking phase 1 failed, no detail marker found")
        config.setTask("notask")

    except config.CartError as e:
        config.log(f"cart rotation/move unsuccessful {e}")
        config.setTask("notask")
        return


def dockingPhase2():    # runs in scope of navManager
    '''
    Cart should be positioned now in front of the docking station and facing the station.
    Evaluation of detail marker returns an initial rotation and a move to left (-0) or right (>0)
    As the cart has slipping in left/right moves repeat procedures until we are well positioned
    '''
    toleratedCartOrientationOffset = 3
    TOLERATED_CART_X_OFFSET = 5
    ROTATION_SPEED_DOCKING_PHASE_2 = 120
    MOVE_SPEED_DOCKING_PHASE_2 = 200

    try:
        for i in range(5):      # number of retries for exact positioning

            config.log(f"docking phase 2 iteration step {i}")

            markerFound, distanceCamToMarker, xOffsetMarker, markerYawDegrees = lookForMarker("CART_CAM", config.DOCKING_DETAIL_ID)

            rotateCartBy = -markerYawDegrees
            distanceSideMove = xOffsetMarker + config.MARKER_XOFFSET_CORRECTION

            config.log(f"docking - offset rotation: {rotateCartBy:.0f}, distanceSideMove: {distanceSideMove:.0f}, distance to marker: {distanceCamToMarker:.0f}")

            # check for sufficient accuracy of cart position
            if abs(rotateCartBy) <= toleratedCartOrientationOffset and abs(distanceSideMove) <= TOLERATED_CART_X_OFFSET:

                robotControl.stopRobot("docking phase 2 successful")
                config.setTask("notask") #dockingPhase3")
                return
            ####################################

            # check for cart parallel in front of dock
            if abs(rotateCartBy) > toleratedCartOrientationOffset:

                config.log(f"cart orientation out of tolerance, rotate to be orthogonal to dock")
                robotControl.rotateCartRelative(rotateCartBy, ROTATION_SPEED_DOCKING_PHASE_2)    #blocking, raises cartException on failure
                
                continue        # recheck proper orientation


            # side move and correction of possibly occurring rotation offset
            if abs(distanceSideMove) > TOLERATED_CART_X_OFFSET:

                startOrientation = config.oCart.orientation
                if distanceSideMove < 0:
                    config.log(f"side move for docking, distanceSideMove: {distanceSideMove:.0f}, move CART_MOVE_LEFT(4)")
                    robotControl.moveCartWithDirection(robotControl.CART_MOVE_LEFT, abs(distanceSideMove), MOVE_SPEED_DOCKING_PHASE_2)
                else:
                    config.log(f"side move for docking, distanceSideMove: {distanceSideMove:.0f}, move CART_MOVE_RIGHT(5)")
                    robotControl.moveCartWithDirection(robotControl.CART_MOVE_RIGHT, abs(distanceSideMove), MOVE_SPEED_DOCKING_PHASE_2)

                # if side move changed orientation rotate back to be orthogonal to marker
                rotationBySideMove = startOrientation - config.oCart.orientation
                config.log(f"compensate for cart orientation change by side move: {rotationBySideMove:.0f}")
                robotControl.rotateCartRelative(rotationBySideMove, ROTATION_SPEED_DOCKING_PHASE_2)    #blocking, raises cartException on failure

    except config.ArucoError as e:
        config.log(f"docking detail marker not found in image, stopping docking phase 2")
        config.setTask("notask")
        return

    except config.CartError as e:
        config.log(f"cart rotation unsuccessful")
        config.setTask("notask")
        return

    # after number of tries without proper position give up
    robotControl.stopRobot(f"dockingPhase2, retries for positioning exhausted")
    time.sleep(0.5)
    config.log(f"dockingPhase2 failed, rotateCartBy: {rotateCartBy:.0f}, distanceSideMove: {distanceSideMove:.0f}")
    config.setTask("notask")


def dockingPhase3():
    '''
    verify marker position and move slowly stepwise forward until the cart is docked
    '''

    try:
        rotation, distance, xOffset = evalDockingDetailMarkerPosition()

        config.log(f"dockingPhase3, distance to marker: {distance:.0f}, xOffset: {xOffset:.0f}")

        for i in range(6):

            robotControl.moveCartWithDirection(robotControl.FORWARD, 30, 80)

            while config.oCart.moving:
                if config.isCartDocked():
                    robotControl.stopRobot("dockingPhase3 successful, cart is docked")
                    config.setTask("notask")
                    return
                else:
                    time.sleep(0.05)
            time.sleep(0.4)

        config.log(f"docking failed")
        config.setTask("notask")

    except config.ArucoError as e:
        config.log(f"docking detail marker not found in image, stopping docking phase 2")
        config.setTask("notask")
        return

    except config.CartError as e:
        config.log(f"cart rotation unsuccessful")
        config.setTask("notask")
        return
