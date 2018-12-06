
import time

import config
import rpcSend

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

getframe_expr = 'sys._getframe({}).f_code.co_name'


def servoMoveToPosition(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)


def servoMoveToPositionBlocking(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)
    timeout = time.time() + duration/1000.0 + 3
    while True:
        # check local position as we get servo updates automatically
        time.sleep(0.1)
        _, currPosition, _ = rpcSend.navManagerServers['servoControl']['conn'].exposed_getPosition(servoName)
        if time.time() > timeout:
            config.log(f"servoMoveToPositionBlocking timeout, requestedPosition: {position}, current position: {currPosition}")
            return
        if abs(currPosition - position) < 2:
            return


def servoMoveToDegrees(servoName, degrees, duration):
    rpcSend.servoRequestDeg(servoName, degrees, duration)


def servoMoveToDegreesBlocking(servoName, degrees, duration=1000):
    rpcSend.servoRequestDeg(servoName, degrees, duration=1000)
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
            config.log("--> change to servoMoveToPosition")
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
            config.log(f"--> change to servoMoveToPositionBlocking in {caller}")
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

    if rpcSend.navManagerServers['cartControl']['simulated']:
        config.log("stop cart")
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

    if rpcSend.navManagerServers['cartControl']['simulated']:
        config.setTargetOrientation((config.getCartOrientation() + relativeAngle) % 360)
        config.log(f"simulated cart rotation by {relativeAngle}")
        return True
    else:

        # calculate target orientation
        config.setTargetOrientation((config.getCartOrientation() + relativeAngle) % 360)
        config.log(
            f"robotControl.rotateCartRelative: rotate cart from {config.getCartOrientation():.0f} to {config.getTargetOrientation():.0f}")

        # request rotation from cart
        rpcSend.navManagerServers['cartControl']['conn'].exposed_rotateRelative(relativeAngle, speed)
        
        # wait for cart stopped
        while True:
            config.queryCartInfo()
            #navGlobal.log(f"after cart command rotateRelative, isCartRotating: {navGlobal.isCartRotating()}")
            if config.isCartRotating():
                time.sleep(0.2)
            else:
                if abs(config.getRemainingRotation()) < 3:
                    config.log(f"cart rotation successful, stopped at {config.getCartOrientation():.0f}")
                    return True
                else:
                    msg = f"robotControl,rotateCart: cart stopped unexpectedly at {config.getCartOrientation()}, request was {config.getTargetOrientation():.0f}"
                    config.log(msg)
                    raise config.CartError(msg)
        

def rotateCartAbsolute(angle, speed):
    rotation = signedAngleDifference(config.getCartOrientation(), angle)
    rotateCartRelative(rotation, speed)


def moveCart(degree, distance, speed):

    cartOrientation = config.getCartOrientation()
    config.log(f"goto degree: {int(degree)}, distance: {int(distance)}, currOrientation: {cartOrientation}")

    rotation = signedAngleDifference(cartOrientation, degree)

    config.log(f"request cart rotation by: {int(rotation)} degrees")
    if rpcSend.navManagerServers['cartControl']['simulated']:
        rotateCartRelative(rotation, 150)
        target = config.evalTargetPos(distance, degree)
        config.setCartLocation(target.x, target.y)
        return True
    else:
        if rotateCartRelative(int(rotation), 150):

            # limit distance to 2 m
            if distance > 2000:
                config.log(f"robotControl, distance reduced to 2000 mm")
                distance = 2000

            target = config.evalTargetPos(distance, degree)

            posX, posY = config.getCartLocation()
            config.log(f"move from x,y: {posX:.0f},{posY:.0f} to {target.x:.0f},{target.y:.0f}")

            if moveCartWithDirection(1, distance, speed):    #FORWARD
                return True
            else:
                config.log("move failed")
                return False

        else:
            config.log("rotation failed")
            return False


#def move(speed, direction, distanceMm):
def moveCartWithDirection(direction, distanceMm, speed):

    try:
        rpcSend.requestMove(direction, speed, distanceMm)

    except Exception as e:
        raise config.CartControlError(f"moveCartWithDirection: {e}")

    # wait for movement done
    while config.isCartMoving():
         time.sleep(0.2)

    if config.getRemainingDistance(distanceMm) > 50:
                    
        # requested position not reached, retry or stop task??
        posX, posY = config.getCartLocation()
        target = config.getTargetLocation()
        config.log(f"move not completed, remaining distance: {config.getRemainingDistance(distanceMm)}")
        msg = f"ERROR move stopped at position: {posX:.0f}/{posY:.0f}, requested: {target.x:.0f}/{target.y:.0f}"
        config.log(msg)
        raise config.CartError(msg)

    #config.saveCartLocation()


def gotoLocation(location):
    return


def approachTarget(target):
    config.log("TODO robotControl.approachTarget {target}")
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
