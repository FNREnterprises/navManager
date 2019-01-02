
import time
import numpy as np

import config
import rpcSend
import guiUpdate

MRL_REST_API = "http://192.168.0.17:8888/api/service/"

# Values for movements of cart
STOP = 0
FORWARD = 1 
DIAGONAL_RIGHT_FORWARD = 2
DIAGONAL_LEFT_FORWARD = 3
CART_MOVE_LEFT = 4 
CART_MOVE_RIGHT = 5
BACKWARD = 6
DIAGONAL_RIGHT_BACKWARD = 7 
DIAGONAL_LEFT_BACKWARD = 8
ROTATE_ANTICLOCKWISE = 9 
ROTATE_CLOCKWISE = 10

directionDegrees = [0,0,-45,45,90,-90,180,-135,135]

getframe_expr = 'sys._getframe({}).f_code.co_name'


def appendToMoveQueue(d):

    # remove oldest element in queue if queue is full
    if len(config.robotMovesQueue) == config.robotMovesQueue.maxlen:
        config.robotMovesQueue.popleft()
    config.robotMovesQueue.append(d)


def servoMoveToPosition(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)


def servoMoveToPositionBlocking(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)
    timeout = time.time() + duration/1000.0 + 3
    while True:
        # check local position as we get servo updates automatically
        time.sleep(0.1)
        _, currPosition, _ = config.navManagerServers['servoControl']['conn'].root.exposed_getPosition(servoName)
        if time.time() > timeout:
            config.log(f"servoMoveToPositionBlocking timeout, requestedPosition: {position}, current position: {currPosition}")
            return
        if abs(currPosition - position) < 2:
            return


def servoMoveToDegrees(servoName, degrees, duration=1000):
    rpcSend.servoRequestDeg(servoName, degrees, duration)


def servoMoveToDegreesBlocking(servoName, degrees, duration=1000):
    rpcSend.servoRequestDeg(servoName, degrees, duration)
    timeout = time.time() + duration/1000.0 + 3
    while True:
        time.sleep(0.1)
        currDegrees, _, moving = rpcSend.servoGetPosition(servoName)
        if time.time() > timeout:
            config.log(f"servoMoveToDegreesBlocking timeout, requestedDegrees: {degrees}, current degrees: {currDegrees}")
            return
        if abs(currDegrees - degrees) < 2:
            return


def servoGetPosition(servoName):
    return rpcSend.servoGetPosition(servoName)


def servoSetAutoDetach(servoName, duration):
    rpcSend.servoSetAutoDetach(servoName, duration)


def servoRest(servoName):
    rpcSend.moveToRestDegrees(servoName)


def servoRestAll():
    rpcSend.servoRestAll()


def servoStop(servoName):
    rpcSend.servoStop(servoName)


def servoStopAll():
    rpcSend.servoStopAll()


def servoControl(servo, method, value=90, duration=1000):
    '''
    use rpyc commands to communicate with inmoovControl
    '''
    if config.simulateServoControl:
        pass
    else:
        if servo[:4] == 'i01.':
            servo = servo[4:]

        if method == 'moveToPosition':
            config.log(f"--> change to servoMoveToPosition")
            servoMoveToPosition(servo, value, duration)

        elif method == 'moveToPositionBlocking':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToPositionBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'moveToDegrees':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToPositionBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'moveToDegreesBlocking':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToDegreesBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'getCurrentPos':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoGetPosition in {caller}")
            servoGetPosition(servo)

        elif method == 'setAutoDetach':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoSetAutoDetach in {caller}")
            servoSetAutoDetach(servo, value)

        elif method == 'rest':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoRest in {caller}")
            servoRest(servo)

        else:
            config.log(f"servoControl, unhandled method '{method}'")

    #navGlobal.log(f"sendMrlCommand, {url}, duration: {time.time()-start:.2f}")

"""
def moveServo(servoName, degrees, duration=1000):
    '''
    move a servo to the requested position if we are not already close to it
    '''
    '''
    if config.simulateMrl:
        return

    currPos = queryMrlService(servo, "getCurrentPos")
    if abs(currPos - requestedPos) > 3:
        sendMrlCommand(servo, "moveTo", str(requestedPos))
    '''
    # exposed_requestServoDegrees(self, servoName, degrees, duration)
    config.inmoovRequest.root.requestServoDegrees(servoName, degrees, duration)
"""

"""
def moveServoBlocking(servoName, degrees, duration=1000):
    '''
    move a servo to the requested position if we are not already close to it
    '''
    '''
    if config.simulateMrl:
        return

    currPos = queryMrlService(servo, "getCurrentPos")
    if abs(currPos - requestedPos) > 3:
        sendMrlCommand(servo, "moveToBlocking", str(requestedPos))
    '''
    # exposed_requestServoDegrees(self, servoName, degrees, duration)
    config.inmoovRequest.root.requestServoDegrees(servoName, degrees, duration)
    moving = True
    while moving:
        time.sleep(0.1)
        currDegrees, currPosition, moving = config.inmoovRequest.root.getPosition(servoName)


# do not use, always get headYaw from inmoovControl
def getHeadYaw():

    if config.simulateMrl:
        headYaw = 90
    else:        
        headYaw = queryMrlService("i01.head.rothead","getCurrentPos")

    config.setCurrentHeadYaw(headYaw)
    return headYaw
"""


def stopRobot(reason):

    config.log(f"stop Robot received, reason: {reason}")

    if config.navManagerServers['cartControl']['simulated']:
        config.log(f"stop cart")
    else:
        rpcSend.servoStopAll()
        time.sleep(1)

    config.saveCartLocation()


def signedAngleDifference(start, end):
    '''
    calculate angle difference in range -180 .. 180 between start and end orientation in range 0 .. 360
    '''
    diff = end - start
    d = abs(diff) % 360
    value = 360 - d if d > 180 else d
    sign = 1 if (0 <= diff <= 180) or (-180 >= diff >= -360) else -1
    return sign * value


def rotateCartRelative(relativeAngle, speed):

    if relativeAngle > 180:
        relativeAngle -= 360

    if relativeAngle < -180:
        relativeAngle += 360

    if config.navManagerServers['cartControl']['simulated']:
        config.oTarget.orientation = ((config.oCart.orientation + relativeAngle) % 360)
        config.log(f"simulated cart rotation by {relativeAngle}")
        return True
    else:

        # calculate target orientation
        config.oTarget.orientation = ((config.oCart.orientation + relativeAngle) % 360)
        config.log(
            f"robotControl.rotateCartRelative: rotate cart from {config.oCart.orientation:.0f} to {config.oTarget.orientation:.0f}")

        # request rotation from cart
        config.oCart.rotating = True
        config.navManagerServers['cartControl']['conn'].root.exposed_rotateRelative(relativeAngle, speed)
        
        # wait for cart stopped, limit wait
        timeout = time.time() + 15
        while config.oCart.rotating and time.time() < timeout:
            time.sleep(0.1)
        if time.time() > timeout:
            config.log(f"timeout rotation")
            config.setTask("notask")

        rpcSend.queryCartInfo()
        #navGlobal.log(f"after cart command rotateRelative, isCartRotating: {navGlobal.isCartRotating()}")
        if abs(getRemainingRotation()) < 3:
            config.log(f"cart rotation successful, stopped at {config.oCart.orientation:.0f}")
            return True
        else:
            msg = f"robotControl,rotateCart: cart stopped unexpectedly at {config.oCart.orientation}, request was {config.oTarget.orientation:.0f}"
            config.log(msg)
            raise config.CartError(msg)


def setTargetLocation(distance, relDegrees):
    """
    careful, carts orientation 0 is to the right while map 0 is 90 degrees
    :param distance:
    :param relDegrees:
    :return:
    """
    dx = distance * np.cos(np.radians(relDegrees))
    dy = distance * np.sin(np.radians(relDegrees))
    config.oTarget.x = int(config.oCart.x + dx)
    config.oTarget.y = int(config.oCart.y + dy)

    # add to move queue
    appendToMoveQueue({'fromX': config.oCart.x, 'fromY': config.oCart.y, 'toX': config.oTarget.x,'toY': config.oTarget.y})


def rotateCartAbsolute(angle, speed):
    config.log(f"rotate cart absolut to {angle}")
    rotation = signedAngleDifference(config.oCart.orientation, angle)
    rotateCartRelative(rotation, speed)


def moveCart(absDegree, distance, speed):

    cartOrientation = config.oCart.orientation
    config.log(f"goto degree: {int(absDegree)}, distance: {int(distance)}, currOrientation: {cartOrientation}")

    rotation = signedAngleDifference(cartOrientation, absDegree)

    config.log(f"request cart rotation by: {int(rotation)} degrees")
    if config.navManagerServers['cartControl']['simulated']:
        rotateCartRelative(rotation, 150)
        setTargetLocation(distance, absDegree)
        return True
    else:
        if rotateCartRelative(int(rotation), 150):

            # limit distance to 2 m
            if distance > 2000:
                config.log(f"robotControl, distance reduced to 2000 mm")
                distance = 2000

            #setTargetLocation(distance, degree)
            #config.oTarget.show = True
            #guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

            #posX, posY = config.getCartLocation()
            #config.log(f"move from x,y: {config.oCart.x:.0f},{config.oCart.y:.0f} to {config.oTarget.x:.0f},{config.oTarget.y:.0f}")

            if moveCartWithDirection(1, distance, speed):    #FORWARD
                return True
            else:
                config.log(f"move failed")
                return False

        else:
            config.log(f"rotation failed")
            return False


#def move(speed, direction, distanceMm):
def moveCartWithDirection(direction, distanceMm, speed):

    try:
        degrees = config.oCart.orientation + directionDegrees[direction] % 360
        setTargetLocation(distanceMm, degrees)

        config.oTarget.show = True
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

        config.log(f"moveByDir: dir: {direction}, cartDeg: {config.oCart.orientation}, moveDeg: {degrees}, distance: {distanceMm}, start: {config.oCart.x:.0f},{config.oCart.y:.0f}, target: {config.oTarget.x:.0f},{config.oTarget.y:.0f}")
        try:
            rpcSend.requestMove(direction, speed, distanceMm)

        except Exception as e:
            raise config.CartError(f"moveCartWithDirection: {e}")

        # wait for cart stopped (rpcReceive gets cart updates during move)
        while config.oCart.moving:
            time.sleep(0.1)

        if getRemainingDistance() > 50:

            # requested position not reached, retry or stop task??
            #posX, posY = config.getCartLocation()
            #target = config.getTargetLocation()
            config.log(f"move not completed, remaining distance: {getRemainingDistance()}")
            msg = f"ERROR move stopped at position: {config.oCart.x:.0f}/{config.oCart.y:.0f}, requested: {config.oTarget.x:.0f}/{config.oTarget.y:.0f}"
            config.log(msg)
            raise config.CartError(msg)
        else:
            config.log(f"move target reached: {config.oTarget.x} / {config.oTarget.y}")
            return True

    except config.CartError as e:
        config.log(f"cart move problem occurred {e}")
        return False


def getRemainingDistance():
    rpcSend.queryCartInfo()
    remaining = np.hypot(config.oTarget.x - config.oCart.x, config.oTarget.y - config.oCart.y)
    config.log(f"target: {config.oTarget.x}, {config.oTarget.y}, cart pos: {config.oCart.x}, {config.oCart.y}, remaining: {remaining}")
    return remaining


def getRemainingRotation():
    a = config.oTarget.orientation - config.oCart.orientation
    a = (a + 180) % 360 - 180
    config.log(f".getRemainingRotation: targetOrientation {config.oTarget.orientation:.0f}, cartOrientation: {config.oCart.orientation:.0f}, diff: {a:.0f}")
    return a


def approachTarget(target):
    config.log(f"TODO robotControl.approachTarget {target}")
    time.sleep(5)
    return True


def showTime():

    # moveCartByDirection(direction (FORWARD,BACKWARD..), distanceMm, speed (100..250)
    # rotateCartRelative(relativeAngle (+/- 360), speed):
    # moveServo(servo, requestedPos, duration=1000):
    # moveServoBlocking(servo, requestedPos, duration=1000):
    # rotateCartRelative(relativeAngle, speed)  + anticlock, - clockwise

    moveCartWithDirection(BACKWARD, 1000, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(FORWARD, 200, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(FORWARD, 1000, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(FORWARD, 200, 200)
    rotateCartRelative(-90, 200)
