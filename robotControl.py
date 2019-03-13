
import time
import numpy as np

import config
import rpcSend
import guiUpdate
import navManager

MRL_REST_API = "http://192.168.0.17:8888/api/service/"

''' use config.Direction instead
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
'''

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
    rpcSend.servoRest(servoName)


def servoRestAll():
    rpcSend.servoRestAll()


def servoStop(servoName):
    rpcSend.servoStop(servoName)


def servoStopAll():
    rpcSend.servoStopAll()


def servoControl(servo, method, value=90, duration=1000):
    """
    use rpyc commands to communicate with inmoovControl
    """
    if config.navManagerServers['servoControl']['simulated']:
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


def stopRobot(reason):

    config.log(f"stop Robot received, reason: {reason}")

    if config.navManagerServers['cartControl']['simulated']:
        config.log(f"stop cart")
    else:
        rpcSend.servoStopAll()
        time.sleep(1)


def signedAngleDifference(start, end):
    """
    calculate angle difference in range -180 .. 180 between start and end orientation in range 0 .. 360
    """
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
        config.oTarget.orientation = ((config.oCart.getYaw() + relativeAngle) % 360)
        config.log(f"simulated cart rotation by {relativeAngle}")
        return True
    else:

        # calculate target orientation
        config.oTarget.orientation = round((config.oCart.getYaw() + relativeAngle) % 360)
        config.log(
            f"robotControl.rotateCartRelative: rotate cart from {config.oCart.getYaw():.0f} to {config.oTarget.orientation:.0f}")

        # request rotation from cart
        config.oCart.rotating = True
        config.navManagerServers['cartControl']['conn'].root.exposed_rotateRelative(relativeAngle, speed)
        
        # wait for cart stopped, limit wait
        timeout = time.time() + abs(relativeAngle) * 0.12 + 3
        while config.oCart.rotating and time.time() < timeout:
            time.sleep(0.1)
        if time.time() > timeout:
            config.log(f"timeout rotation")
            navManager.setTask("notask")

        rpcSend.queryCartInfo()
        #navGlobal.log(f"after cart command rotateRelative, isCartRotating: {navGlobal.isCartRotating()}")
        if abs(getRemainingRotation()) < 3:
            config.log(f"cart rotation successful, stopped at {config.oCart.getYaw():.0f}")
            return True
        else:
            msg = f"robotControl,rotateCart: cart stopped unexpectedly at {config.oCart.getYaw()}, request was {config.oTarget.orientation:.0f}"
            config.log(msg)
            raise config.CartError(msg)


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
    appendToMoveQueue({'fromX': config.oCart.getX(), 'fromY': config.oCart.getY(), 'toX': config.oTarget.x,'toY': config.oTarget.y})


def rotateCartAbsolute(angle, speed):
    config.log(f"rotate cart absolut to {angle}")
    rotation = signedAngleDifference(config.oCart.getYaw(), angle)
    rotateCartRelative(rotation, speed)


def moveCart(absDegree, distance, speed):

    cartOrientation = config.oCart.getYaw()
    config.log(f"goto degree: {int(absDegree)}, distance: {int(distance)}, currOrientation: {cartOrientation}")

    rotation = signedAngleDifference(cartOrientation, absDegree)

    config.log(f"request cart rotation by: {int(rotation)} degrees")
    if config.navManagerServers['cartControl']['simulated']:
        rotateCartRelative(rotation, 150)
        setTargetLocation(distance, absDegree)
        return True
    else:
        if rotateCartRelative(int(rotation), 150):

            time.sleep(0.5)     # not sure, delay in messaging caused to see it not moving in the next step?

            # limit distance to 2.5 m
            if distance > 2500:
                config.log(f"robotControl, distance reduced to 2500 mm")
                distance = 2500

            #setTargetLocation(distance, degree)
            #config.oTarget.show = True
            #guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

            #posX, posY = config.getCartLocation()
            #config.log(f"move from x,y: {config.oCart.x:.0f},{config.oCart.y:.0f} to {config.oTarget.x:.0f},{config.oTarget.y:.0f}")

            if moveCartWithDirection(config.Direction.FORWARD, distance, speed):    #FORWARD
                return True
            else:
                config.log(f"move failed")
                return False

        else:
            config.log(f"rotation failed")
            return False


#def move(speed, direction, distanceMm):
def moveCartWithDirection(moveDirection: config.Direction, distanceMm, speed):

    try:
        degrees = config.oCart.getYaw() + directionDegrees[moveDirection.value] % 360
        setTargetLocation(distanceMm, degrees)

        config.oTarget.show = True
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

        config.log(f"moveByDir: dir: {moveDirection}, cartDeg: {config.oCart.getYaw()}, moveDeg: {degrees}, distance: {distanceMm}, start: {config.oCart.getX():.0f},{config.oCart.getY():.0f}, target: {config.oTarget.x:.0f},{config.oTarget.y:.0f}")
        try:
            rpcSend.requestMove(moveDirection, speed, distanceMm)

        except Exception as e:
            raise config.CartError(f"moveCartWithDirection: {e}")

        # wait for cart stopped (rpcReceive gets cart updates during move)
        while config.oCart.moving:
            time.sleep(0.1)

        if getRemainingDistance() > 50:

            # requested position not reached, retry or stop task??
            #posX, posY = config.getCartLocation()
            #target = config.getTargetLocation()
            config.log(f"move not completed, remaining distance: {getRemainingDistance():.0f}")
            msg = f"ERROR move stopped at position: {config.oCart.getX():.0f}/{config.oCart.getY():.0f}, requested: {config.oTarget.x:.0f}/{config.oTarget.y:.0f}"
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
    remaining = np.hypot(config.oTarget.x - config.oCart.getX(), config.oTarget.y - config.oCart.getY())
    config.log(f"target: {config.oTarget.x}, {config.oTarget.y}, cart pos: {config.oCart.getX()}, {config.oCart.getY()}, remaining: {remaining:.0f}")
    return remaining


def getRemainingRotation():
    a = config.oTarget.orientation - config.oCart.getYaw()
    a = (a + 180) % 360 - 180
    config.log(f"remaining rotation: targetOrientation {config.oTarget.orientation:.0f}, cartOrientation: {config.oCart.getYaw():.0f}, diff: {a:.0f}")
    return a


def approachTarget(target):
    config.log(f"TODO robotControl.approachTarget {target}")
    time.sleep(5)
    return True


def showCartMoves():

    # moveCartByDirection(direction (FORWARD,BACKWARD..), distanceMm, speed (100..250)
    # rotateCartRelative(relativeAngle (+/- 360), speed):
    # moveServo(servo, requestedPos, duration=1000):
    # moveServoBlocking(servo, requestedPos, duration=1000):
    # rotateCartRelative(relativeAngle, speed)  + anticlock, - clockwise

    moveCartWithDirection(config.Direction.BACKWARD, 1000, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(config.Direction.FORWARD, 200, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(config.Direction.FORWARD, 1000, 200)
    rotateCartRelative(90, 200)
    moveCartWithDirection(config.Direction.FORWARD, 200, 200)
    rotateCartRelative(-90, 200)
