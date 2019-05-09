
"""
cartHandling.py
used to do all cart moves
supports definition of move sequences and continuation of an interrupted move sequence
"""
import time
import numpy as np
from dataslots import with_slots        # with_slots in combination with dataclass will generate the slot list
from dataclasses import dataclass       # dataclass simplifies class definition
from enum import Enum, unique

import config
import rpcSend
import guiUpdate

ROTATION_SPEED = 150
MOVE_TOLERANCE = 10
ROTATION_TOLERANCE = 1
MAX_MOVE_RETRIES = 3

directionDegrees = [0,0,-45,45,90,-90,180,-135,135]

@unique
class cMoveState(Enum):
    """
    list of move states for single move and move sequence
    """
    MOVE_PENDING = 0
    MOVE_IN_PROGRESS = 1
    MOVE_INTERRUPTED = 2
    MOVE_FINISHED = 3
    MOVE_FAILED = 4

@unique
class cMoveSequenceState(Enum):
    """
    list of move states for single move and move sequence
    """
    MOVESEQUENCE_PENDING = 0
    MOVESEQUENCE_IN_PROGRESS = 1
    MOVESEQUENCE_FINISHED = 2
    MOVESEQUENCE_FAILED = 3

@with_slots     # prevents adding attibutes, does not allow for default values
@dataclass
class cCartMove:
    direction: config.Direction
    speed: int
    requested: int
    remaining: int
    tolerance: int
    moveStatus: cMoveState
    numContinues: int
    kinectMonitoring: bool

@with_slots     # prevents adding attibutes, does not allow for default values
@dataclass
class cCartMoveSequence:
    numMoves: int
    moves: []
    sequenceStatus: cMoveSequenceState
    currMove: int


    def addMove(self, direction, speed, value, tolerance, kinectMonitoring=False):
        oCartMove = cCartMove(direction, speed, value, value, tolerance, cMoveState.MOVE_PENDING, MAX_MOVE_RETRIES, kinectMonitoring)
        self.numMoves += 1
        self.moves.append(oCartMove)
        config.log(f"move added to move sequence: {direction}, {value}")


    def updateMove(self, progress: int, final: bool):

        config.log(f"updateMoveSequence, progress: {progress}, cartStopped: {final}")
        thisMove = self.moves[self.currMove]
        thisMove.remaining = thisMove.requested - progress

        if final:
            if abs(thisMove.remaining) < thisMove.tolerance:
                thisMove.moveStatus = cMoveState.MOVE_FINISHED
                config.log(f"move finished, remaining: {thisMove.remaining}, step: {self.moves[self.currMove]}")
            else:
                thisMove.moveStatus = cMoveState.MOVE_INTERRUPTED
                config.log(f"move interrupted, remaining: {thisMove.remaining}, step: {self.moves[self.currMove]}")
        else:
            thisMove.moveStatus = cMoveState.MOVE_IN_PROGRESS


    def startOrContinueMoveSequence(self):
        self.sequenceStatus = cMoveSequenceState.MOVESEQUENCE_IN_PROGRESS
        thisMove = self.moves[self.currMove]
        if thisMove.direction in [config.Direction.ROTATE_LEFT, config.Direction.ROTATE_RIGHT]:
            rpcSend.requestRotation(thisMove.direction, thisMove.speed, thisMove.requested)
        else:
            rpcSend.requestMove(thisMove.direction, thisMove.speed, thisMove.requested)
        config.log(f"send move cart, direction {thisMove.direction}, speed: {thisMove.speed}, requested: {thisMove.requested}")


    def continueMove(self):
        thisMove = self.moves[self.currMove]
        if thisMove.numContinues > 3:
            config.log(f"no success with {self.numContinues-2} retries, giving up")
            thisMove.moveStatus = cMoveState.MOVE_FAILED
            return
        # count number of retries
        thisMove.numContinues += 1
        # set requested to remaining and retry
        thisMove.requested = thisMove.remaining
        self.startOrContinueMoveSequence()


    def nextMove(self):
        if self.currMove == self.numMoves-1:
            self.sequenceStatus = cMoveSequenceState.MOVESEQUENCE_FINISHED
            return
        else:
            self.currMove += 1
            config.log(f"continue with next move in sequence")
            self.startOrContinueMoveSequence()

# workaround for frozen class and default values
def newMoveSequence():
    return cCartMoveSequence(0, [], sequenceStatus=cMoveSequenceState.MOVESEQUENCE_PENDING, currMove=0)


def evalBestMoveDirection(angle, allowedMoves):
    """
    for the possible move directions find shortest path
    :param angle: relative moveAngle, -180 .. 180
    :param allowedMoves: FORWARD, BACKWARD, LEFT, RIGHT
    :return: moveDirection, startRotation, endRotation
    """
    F = config.Direction.FORWARD
    B = config.Direction.BACKWARD
    L = config.Direction.LEFT
    R = config.Direction.RIGHT

    moveTable = [
        {'startAngle':    0, 'endAngle':   45, 'moveDir': [F,L,B,R]},
        {'startAngle':   45, 'endAngle':   90, 'moveDir': [L,F,B,R]},
        {'startAngle':   90, 'endAngle':  135, 'moveDir': [L,B,F,R]},
        {'startAngle':  135, 'endAngle':  180, 'moveDir': [B,L,F,R]},
        {'startAngle': -180, 'endAngle': -135, 'moveDir': [B,R,F,L]},
        {'startAngle': -135, 'endAngle':  -90, 'moveDir': [R,B,F,L]},
        {'startAngle':  -90, 'endAngle':  -45, 'moveDir': [R,F,B,L]},
        {'startAngle':  -45, 'endAngle':    0, 'moveDir': [F,R,B,L]}
    ]
    moveDirection = config.Direction.FORWARD
    rotationForMove = angle
    endRotation = 0

    for entry in moveTable:
        if angle >= entry['startAngle'] and angle <= entry['endAngle']:
            for cartMoveDirection in entry['moveDir']:
                if cartMoveDirection in allowedMoves:
                    moveDirection = cartMoveDirection
                    break
            break

    if moveDirection == config.Direction.FORWARD:
        rotationForMove = angle
    elif moveDirection == config.Direction.LEFT:
        rotationForMove = angle - 90
        endRotation = 90 - angle
    elif moveDirection == config.Direction.BACKWARD:
        rotationForMove = angle - 180
        endRotation = 180 if 90 > angle < 270 else 0
    elif moveDirection == config.Direction.RIGHT:
        rotationForMove = angle + 90
        endRotation = 90 + angle
    else:
        config.log(f"evalBestMoveDirection, no moveDirection found")

    if rotationForMove > 180:
        rotationForMove = rotationForMove - 360
    if rotationForMove < -180:
        rotationForMove = rotationForMove + 360

    config.log(f"evalBestMoveDirection: {moveDirection}, startRotation: {rotationForMove}, endRotation: {endRotation}")

    return moveDirection, rotationForMove, endRotation



def setTargetLocation(distance, relDegrees):
    """
    distance in mm
    :param distance:
    :param relDegrees:
    :return:
    """
    dx = round(distance * np.cos(np.radians(relDegrees)))
    dy = round(distance * np.sin(np.radians(relDegrees)))
    config.oTarget.x = config.oCart.getX() + dx
    config.oTarget.y = config.oCart.getY() + dy

    # add to move queue
    # appendToMoveQueue({'fromX': config.oCart.getX(), 'fromY': config.oCart.getY(), 'toX': config.oTarget.x,'toY': config.oTarget.y})


def rotateCartAbsolute(degrees):
    """
    stub for rotate cart, speed is ROTATION_SPEED
    :param degrees:
    :return:
    """
    config.log(f"rotate cart absolut to {degrees}")
    moveCart(degrees, 0, ROTATION_SPEED)


def rotateCartRelative(angle):
    config.log(f"rotate cart for {angle}")
    moveCart(config.oCart.getDegrees() + angle, 0, ROTATION_SPEED)


# TODO: really needed?
def moveCartWithDirection(moveDirection: config.Direction, distanceMm, speed, kinectMonitoring=True):
    config.log(f"move cart with direction: {moveDirection}, dist: {distanceMm}, speed: {speed}, useKinect: {kinectMonitoring}")
    allowedMoves = [moveDirection]
    degrees = (config.oCart.getDegrees() + directionDegrees[moveDirection.value]) % 360
    moveCart(degrees, distanceMm, speed, allowedMoves)


def moveCart(degrees, distance, speed, allowedMoves=None, kinectMonitoring=True) -> bool:

    if allowedMoves is None:    # construct to avoid side effects of mutable default values
        allowedMoves = [config.Direction.FORWARD]

    cartDegrees = config.oCart.getDegrees()
    config.log(f"goto degree: {int(degrees)}, distance: {int(distance)}, currDegrees: {cartDegrees}")

    angle = config.signedAngleDifference(cartDegrees, degrees)
    moveDirection, rotationForMove, endRotation = evalBestMoveDirection(angle, allowedMoves)

    config.log(f"create move sequence")
    config.oMoveSteps = newMoveSequence()

    if abs(rotationForMove) > 1:
        if rotationForMove > 0:
            config.oMoveSteps.addMove(config.Direction.ROTATE_LEFT, ROTATION_SPEED, abs(rotationForMove), ROTATION_TOLERANCE)
        else:
            config.oMoveSteps.addMove(config.Direction.ROTATE_RIGHT, ROTATION_SPEED, abs(rotationForMove), ROTATION_TOLERANCE)

    # limit distance to 2.5 m
    if distance > 2500:
        config.log(f"robotControl, distance reduced to 2500 mm")
        distance = 2500

    if distance > MOVE_TOLERANCE:
        config.oMoveSteps.addMove(moveDirection, speed, distance, MOVE_TOLERANCE, kinectMonitoring)

    if abs(endRotation) > 1:
        if endRotation > 0:
            config.oMoveSteps.addMove(config.Direction.ROTATE_LEFT, ROTATION_SPEED, abs(endRotation), ROTATION_TOLERANCE)
        else:
            config.oMoveSteps.addMove(config.Direction.ROTATE_RIGHT, ROTATION_SPEED, abs(endRotation), ROTATION_TOLERANCE)

    # start move sequence
    config.oMoveSteps.startOrContinueMoveSequence()

    ############################
    # monitor move sequence
    ############################
    while True:

        # check on current move in sequence
        thisMove = config.oMoveSteps.moves[config.oMoveSteps.currMove]
        thisSequence = config.oMoveSteps

        if thisMove.moveStatus == cMoveState.MOVE_IN_PROGRESS:
            time.sleep(0.1)

        if thisMove.moveStatus == cMoveState.MOVE_INTERRUPTED:
            # if a move did not finish try to continue it
            if config.oMoveSteps.numContinues < 5:
                config.log(f"move step interrupted, try to continue")
                config.oMoveSteps.continueMove()
            else:
                # if retries do not help stop the sequence
                config.log(f"move step {thisMove.direction} FAILED")
                thisMove.moveStatus = cMoveState.MOVE_INTERRUPTED
                config.oMoveSteps.sequenceStatus = cMoveSequenceState.MOVESEQUENCE_FAILED

        if thisMove.moveStatus == cMoveState.MOVE_FAILED:
            # if a single move failed stop the sequence
            config.log(f"move step {thisMove.direction} FAILED")
            config.oMoveSteps.sequenceStatus = cMoveSequenceState.MOVESEQUENCE_FAILED

        if thisMove.moveStatus == cMoveState.MOVE_FINISHED:
            # if a single move is donw check for last move in sequence
            if thisSequence.currMove == thisSequence.numMoves-1:
                thisSequence.sequenceStatus = cMoveSequenceState.MOVESEQUENCE_FINISHED
            else:
                thisSequence.nextMove()

        # check on move sequence
        if config.oMoveSteps.sequenceStatus == cMoveSequenceState.MOVESEQUENCE_FAILED:
            config.log(f"move sequence failed")
            return False

        if config.oMoveSteps.sequenceStatus == cMoveSequenceState.MOVESEQUENCE_FINISHED:
            config.log(f"move sequence done")
            return True


def updateCartInfo(cartX, cartY, degrees, cartMoving, cartRotating):
    config.oCart.setX(cartX)
    config.oCart.setY(cartY)
    config.oCart.setDegrees(degrees)
    config.oCart.moving = cartMoving
    config.oCart.rotating = cartRotating
    config.oCart.update = time.time()
    config.log(f"updateCartInfo received: {config.oCart.getX()}, {config.oCart.getY()}, {config.oCart.getDegrees()},  {config.oCart.moving}, {config.oCart.rotating}", publish=False)
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})
