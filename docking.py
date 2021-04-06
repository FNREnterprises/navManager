
import numpy as np
#from pyrecord import Record
import cv2

import config
import robotHandling
import rpcSend
import navManager
import navMap
import cartHandling
import aruco


SIDE_MOVE_SPEED = 150
DOCKING_SPEED = 60
DOCK_MARKER_ID = 10
DOCK_DETAIL_MARKER_ID = 11
DOCKING_START_DISTANCE_FROM_MARKER = 600
DISTANCE_CART_CENTER_CAM = 330
DOCKING_ROTATION_THRESHOLD = 1
DOCKING_MOVE_THRESHOLD = 5

'''
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
'''

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
        config.log(f"docking.moveToDockingStartPosition")
        config.log(f"docking marker, X: {dockMarker.markerX}, Y: {dockMarker.markerY}, markerYaw: {dockMarker.markerYaw}")
        config.log(f"requested orthogonal distance from docking marker: {DOCKING_START_DISTANCE_FROM_MARKER}")

        # try to move cart to a position orthogonal in front of the dockingMarker
        # orthogonal position is <distanceCartToTarget> away from marker
        xCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.cos(np.radians(dockMarker.markerYaw + 90))
        yCorr = DOCKING_START_DISTANCE_FROM_MARKER * np.sin(np.radians(dockMarker.markerYaw + 90))
        config.oTarget.setCartX(dockMarker.markerX + xCorr)
        config.oTarget.setCartY(dockMarker.markerY + yCorr)

        # at target rotate cart to face marker
        # calculate angle from target position to marker position
        endDegrees = np.degrees(np.arctan2(xCorr, yCorr))

        # find abs degreesCartToTarget from current cart position to target position
        xDiff = config.oTarget.getCartX() - config.oCart.getCartX()
        yDiff = config.oTarget.getCartY() - config.oCart.getCartY()
        degreesCartToTarget = int(round(np.degrees(np.arctan2(yDiff,xDiff))))
        distanceCartToTarget = int(round(np.hypot(xDiff, yDiff)))

        # in addition calculate the end rotation for the cart to face the marker
        xDiffTargetToMarker = dockMarker.markerX - config.oTarget.getCartX()
        yDiffTargetToMarker = dockMarker.markerY - config.oTarget.getCartY()
        degreesTargetToMarker = int(round(np.degrees(np.arctan2(yDiffTargetToMarker,xDiffTargetToMarker))))
        distanceTargetToMarker = int(round(np.hypot(xDiffTargetToMarker, yDiffTargetToMarker)))

        # log values
        config.log(f"cart,   X: {config.oCart.getCartX()}, Y: {config.oCart.getCartY()}, degrees: {config.oCart.getCartYaw()}")
        config.log(f"target, X: {config.oTarget.getCartX()}, Y: {config.oTarget.getCartY()}, degreesCartToTarget: {degreesCartToTarget}")
        config.log(f"move distanceCartToTarget: {distanceCartToTarget}, degreesCartToTarget: {degreesCartToTarget}")
        config.log(f"cart end degreesCartToTarget: {degreesTargetToMarker}, distanceCartToTarget cart to marker: {distanceTargetToMarker}")

        # visualise docking move
        dockMap = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)
        navMap.addCenter(dockMap)
        navMap.addCart(dockMap, config.oCart.getCartX(), config.oCart.getCartY(), config.oCart.getCartYaw(), config.oCart.mapColor)
        navMap.addMarker(dockMap, dockMarker.markerX, dockMarker.markerY, dockMarker.markerYaw)
        navMap.addCart(dockMap, config.oTarget.getCartX(), config.oTarget.getCartY(), degreesTargetToMarker, config.oTarget.mapColor)
        navMap.addPathToTarget(dockMap)     # line from cartPosition to targetPosition

        cv2.imshow("docking", dockMap)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # check for cart position close to target
        if distanceCartToTarget < 100:
            config.log(f"cart is already close to docking detail start point, do not move cart")
            degreesCartToTarget = config.oCart.getCartYaw()
        else:

            if distanceCartToTarget < 500:
                # for a short move we can use the 4 main cart directions
                config.log(f"distanceCartToTarget: {distanceCartToTarget}, short distanceCartToTarget, use FORWARD,BACKWARD,LEFT or RIGHT to get there")
                allowedMoves=[config.Direction.FORWARD, config.Direction.BACKWARD, config.Direction.LEFT, config.Direction.RIGHT]
            else:   # otherwise force to move forward
                config.log(f"distanceCartToTarget: {distanceCartToTarget}, for longer moves use FORWARD only")
                allowedMoves=[config.Direction.FORWARD]
            if cartHandling.createMoveSequence(degreesCartToTarget, distanceCartToTarget, 200,
                                               DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD,
                                               allowedMoves, cartMoveMonitoring=False):
                if not cartHandling.moveCart():
                    navManager.setTask("notask")
                    return False

        # end rotation to face marker
        relativeEndRotation = config.signedAngleDifference(config.oCart.getCartYaw(), degreesTargetToMarker)
        config.log(f"relative end rotation to face marker: {relativeEndRotation}")
        if abs(relativeEndRotation) > 2:
            cartHandling.rotateCartRelative(relativeEndRotation)

        return True

    except config.CartError as e:
        config.log(f"cart exception raised: {e}")
        navManager.setTask('notask')


def alignWithMarker(sideMoveOnly=False):
    """
    a loop of rotation and side moves to position cart directly in front of marker
    for rotation use a reduced part of the optically evaluated markerYaw value
    :param sideMoveOnly:
    :return:
    """

    markerYawRotationThreshold = 1      # do not rotate if measured degrees is below threshold
    markerYaw = markerYawRotationThreshold + 1  # make sure to enter alignment loop
    sideMoveThreshold = 5              # do not move sideways if measured offset is below threshold
    sideMove = sideMoveThreshold + 1    # make sure to enter alignment loop

    # verify that cart is aligned with marker
    # for docking this means we should align cart front with markerYaw
    distance = None
    while abs(markerYaw) > markerYawRotationThreshold or abs(sideMove) > sideMoveThreshold:

        adjustCart = False
        if not sideMoveOnly and abs(markerYaw) > markerYawRotationThreshold:

            navMap.takeCartcamImage(show=True)
            markerFound, markerInfo = aruco.lookForMarkers("CART_CAM", [DOCK_DETAIL_MARKER_ID])

            if markerFound:

                # partially rotate to compensate for markerkdegrees
                # 2.4.2019 using calculated markerYaw caused overshooting or even loss of marker in cam frame
                # as we looked for DOCK_DETAIL_MARKER only use first result ([0])
                markerYaw = markerInfo[0]['markerYaw'] / 2
                distance = markerInfo[0]['distanceCamToMarker']
                config.log(f"first rotate cart to be orthogonal with marker {markerYaw}")
                if abs(markerYaw) > markerYawRotationThreshold:
                    if cartHandling.createMoveSequence(config.oCart.getCartYaw() - markerYaw, 0, 180,
                                                       DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD):
                        adjustCart = True
                        moveSuccess = cartHandling.moveCart()
                    #cartHandling.rotateCartRelative(markerYaw)

        if abs(sideMove) > sideMoveThreshold:

            navMap.takeCartcamImage(show=True)
            markerFound, markerInfo = aruco.lookForMarkers("CART_CAM", [DOCK_DETAIL_MARKER_ID])

            if markerFound:

                # as we looked for DOCK_DETAIL_MARKER only use first result ([0])
                angle = markerInfo[0]['angleInImage']
                distance = markerInfo[0]['distanceCamToMarker']
                sideMove = distance * np.sin(np.radians(angle))
                config.log(f"move sideways to be directly in front of marker, angle: {angle}, distance: {distance}, sideMove: {sideMove:.0f}")

                if sideMove > sideMoveThreshold:
                    adjustCart = True
                    cartHandling.moveCartWithDirection(config.Direction.LEFT, abs(sideMove), SIDE_MOVE_SPEED,
                                                       DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD, cartMoveMonitoring=False)

                if sideMove < -sideMoveThreshold:
                    adjustCart = True
                    cartHandling.moveCartWithDirection(config.Direction.RIGHT, abs(sideMove), SIDE_MOVE_SPEED,
                                                       DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD, cartMoveMonitoring=False)

        if not adjustCart:
            break
    return distance


def detailDockMoves():

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
        if cartHandling.createMoveSequence(config.oCart.getCartYaw(), moveDistance, DOCKING_SPEED, cartMoveMonitoring=False):
            cartHandling.moveCart()

    # step closer and verify distance
    distance = alignWithMarker()
    if distance is None:
        config.log(f"lost marker during detailDockMoves, docking failed in step2")
        navManager.setTask('notask')
        return

    moveDistance = distance - 200
    if abs(moveDistance) > moveThreshold:
        cartHandling.moveCartWithDirection(config.Direction.FORWARD, moveDistance, DOCKING_SPEED,
                                           DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD, cartMoveMonitoring=False)

    # now try to dock, arduino watches docking switch and stops cart when activated
    distance = alignWithMarker(sideMoveOnly=True)
    if distance is None:
        config.log(f"lost marker during detailDockMoves, docking failed in step3")
        navManager.setTask('notask')
        return

    moveDistance = distance - 100
    if abs(moveDistance) > moveThreshold:
        cartHandling.moveCartWithDirection(config.Direction.FORWARD, moveDistance, DOCKING_SPEED,
                                           DOCKING_ROTATION_THRESHOLD, DOCKING_MOVE_THRESHOLD, cartMoveMonitoring=False)

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
        config.log(f"move to docking start position done")

        detailDockMoves()

    except Exception as e:
        config.log(f"{e}")


"""
def calculateCartMovesForDockingPhase1(corners):
    cartCenterPos = point2D(0, 0)
    camPos = point2D(0, DISTANCE_CART_CENTER_CAM)

    marker = markers["dockingMarker"]

    vec = aruco.estimatePoseSingleMarkers(corners, marker.length, config.cartcamMatrix,
                                          config.cartcamDistortionCoeffs)  # For a single marker

    distanceCamToMarker = vec[1][0, 0, 2]
    xOffsetMarker = vec[1][0, 0, 0]
    config.log(f"distanceCamToMarker: {distanceCamToMarker:.0f}, xOffsetMarker: {xOffsetMarker:.0f}")
    # log(f"vec[1] {vec[1]}")

    # angle of marker center in cam image, atan of x-offset/distanceCamToMarker
    markerAngleInImage = np.degrees(np.arctan(-xOffsetMarker / distanceCamToMarker))

    # calculate marker position relativ to cart center (cartcam looks straight out)
    markerCenterPos = point2D(xOffsetMarker, camPos.y + np.cos(np.radians(markerAngleInImage)) * distanceCamToMarker)
    config.log(
        f"markerAngleInImage: {markerAngleInImage:.1f}, markerCenterPos(relative to cart center): {markerCenterPos.x:.0f} / {markerCenterPos.y:.0f}")

    # eval the marker's yaw (the yaw of the marker itself evaluated from the marker corners)
    rmat = cv2.Rodrigues(vec[0])[0]
    yrp = rotationMatrixToEulerAngles(rmat)
    markerYaw = -np.degrees(yrp[0])  # ATTENTION, this is the yaw of the marker evaluated from the image

    # orthoAngle = markerAngleInImage + markerYaw
    orthoAngle = markerYaw
    xCorr = (marker.orthoStopDist) * np.sin(np.radians(orthoAngle))
    yCorr = (marker.orthoStopDist) * np.cos(np.radians(orthoAngle))
    config.log(
        f"markerYaw: {markerYaw:.1f}, orthoAngle: {orthoAngle:.1f}, xCorr: {xCorr:.0f}, yCorr: {yCorr:.0f}")

    # cart target position is marker's orthogonal point at distance
    # use the offset to account for the x-difference of the docking detail marker center vs the docking marker center
    cartTargetPos = point(markerCenterPos.x + xCorr + MARKER_XOFFSET_CORRECTION, markerCenterPos.y - yCorr)

    log(f"cartTargetPos (cartCenter) = {cartTargetPos.x:.0f},{cartTargetPos.y:.0f}")

    cartStartRotation = np.degrees(np.arctan(cartTargetPos.x / cartTargetPos.y))
    cartMove = np.hypot(cartTargetPos.x, cartTargetPos.y)
    cartEndRotation = -(np.degrees(np.arctan(xCorr / yCorr)) + cartStartRotation)

    return [cartStartRotation, cartMove, cartEndRotation]


def calculateCartMovesForDockingPhase2(corners):
    '''
    cart is expected to be in front of the docking station
    calculate detail moves with the DOCKING_DETAIL_MARKER
    '''

    marker = markers["dockingDetail"]

    vec = aruco.estimatePoseSingleMarkers(corners, marker.length, config.cartcamMatrix,
                                          config.cartcamDistortionCoeffs)  # For a single marker

    distanceCamToMarker = vec[1][0, 0, 2]
    xOffsetMarker = vec[1][0, 0, 0]

    # eval the marker's yaw (the yaw of the marker itself evaluated from the marker corners)
    rmat = cv2.Rodrigues(vec[0])[0]
    yrp = rotationMatrixToEulerAngles(rmat)
    markerYaw = np.degrees(yrp[0])  # ATTENTION, this is the yaw of the marker evaluated from the image

    # rotate only if we are not orthogonal to the marker
    rotation = 0
    if abs(markerYaw) > 2:
        rotation = markerYaw

    config.log(f"rotation: {rotation:.0f}, xOffsetMarker: {xOffsetMarker:.0f}, distanceCamToMarker: {distanceCamToMarker:.0f}")

    return rotation, xOffsetMarker, distanceCamToMarker
"""