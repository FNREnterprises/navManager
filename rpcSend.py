
import time
import rpyc
import functools

import inmoovGlobal
import config
import guiUpdate
import navManager
import winsound



def handleCartException(function):
    """
    A decorator that wraps the passed in function and logs
    exceptions should one occur
    """
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        if connected('cartControl'):
            try:
                return function(*args, **kwargs)
            except Exception as e:
                config.log(f"connection issue with cartControl, {e}")
                config.servers['cartControl'].conn = None
                # re-raise the exception
                # raise
        else:
            config.log(f"no connection with cartControl")

    return wrapper


def handleRobotException(function):
    """
    A decorator that wraps the passed in function and logs
    exceptions should one occur
    """
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        if connected('robotControl'):
            try:
                return function(*args, **kwargs)
            except Exception as e:
                config.log(f"connection issue with robotControl, {e}")
                config.servers['robotControl'].conn = None
        else:
            config.log(f"no connection with robotControl")

    return wrapper


def simulated(server):
    if config.servers[server].simulated:
        config.log(f"{server} simulated")
        return True
    return False


def connected(server):
    if config.servers[server].conn is None or config.servers[server].simulated:
        #config.log(f"no {server} connection established")
        return False
    return True


def setSimulationMask(cartControl, robotControl):
    config.servers['cartControl'].simulated = cartControl
    config.servers['robotControl'].simulated = robotControl


def terminateSlaveServer(server):
    # request slave termination from taskOrchestrator
    try:
        config.taskOrchestrator.root.exposed_stopServer(server)
    except Exception as e:
        config.log(f"could not request task stop '{server}' from taskOrchestrator,{e}")


def neededServersRunning(serversNeeded):
    serversRunning = []
    for server in serversNeeded:
        if config.servers[server].connectionState == 'ready':
            serversRunning.append(server)
        if config.servers[server].simulated:
            serversRunning.append(server)
    diff = list(set(serversNeeded) - set(serversRunning))
    if len(diff) > 0:
        config.log(f"missing running servers for requested command: {diff}")
        navManager.setTask("notask")
        return False
    return True


def queryCartInfo_ori():
    """
    query cart task for current values
    cartDegrees, cartLocationX, cartLocationY, cartMoving, cartRotating
    """
    if connected('cartControl'):
        #cartDegrees, cartLocationX, cartLocationY, cartMoving, cartRotating = config.servers['cartControl'].conn.root.exposed_getCartInfo()
        x,y,o,config.oCart.moving,config.oCart.rotating = config.servers['cartControl'].conn.root.exposed_getCartInfo()
        config.oCart.setCartX(x)
        config.oCart.setCartY(y)
        config.oCart.setCartYaw(o)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})
        config.log(f"new CartInfo: x: {config.oCart.getCartX()}, y: {config.oCart.getCartY()}, degrees: {config.oCart.getCartYaw()}, moving: {config.oCart.moving}, rotating: {config.oCart.rotating}")
    else:
        config.log(f"no connection with cartControl")

@handleCartException
def queryCartInfo():
    x,y,o,config.oCart.moving,config.oCart.rotating = config.servers['cartControl'].conn.root.exposed_getCartInfo()
    config.oCart.setCartX(x)
    config.oCart.setCartY(y)
    config.oCart.setCartYaw(o)
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})
    config.log(f"new CartInfo: x: {config.oCart.getCartX()}, y: {config.oCart.getCartY()}, degrees: {config.oCart.getCartYaw()}, moving: {config.oCart.moving}, rotating: {config.oCart.rotating}")

def queryBatteries_ori():
    if connected('cartControl'):

        try:
            config.batteryStatus = config.servers['cartControl'].conn.root.getBatteryStatus()
        except Exception as e:
            config.servers['cartControl'].conn = None
            config.log(f"connection with cartControl lost, {e}")


        if config.batteryStatus is not None:
            #config.log(f"received battery info")
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.BATTERY_UPDATE})

            if config.batteryStatus['percent'] < 20:
                winsound.PlaySound('sound.wav', winsound.SND_FILENAME)

@handleCartException
def queryBatteries():

    config.batteryStatus = config.servers['cartControl'].conn.root.getBatteryStatus()

    if config.batteryStatus is not None:
        #config.log(f"received battery info")
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.BATTERY_UPDATE})

        if config.batteryStatus['percent'] < 20:
            winsound.PlaySound('sound.wav', winsound.SND_FILENAME)


def getImage(cam):
    """
    cam is one of the constants defined in inmoovGlobal
    """
    if connected('cartControl'):

        try:
            return rpyc.classic.obtain(config.servers['cartControl'].conn.root.takeImage(cam))
        except Exception as e:
            config.log(f"could not acquire image from cam: {cam}: {e}")
            return None

    else:
        return None


def requestHeadOrientation():

    if connected('cartControl'):

        try:
            #config.oHead.setHeadOrientation(config.servers['cartControl'].conn.root.requestHeadOrientation())
            orientation = config.servers['cartControl'].conn.root.requestHeadOrientation()
            config.oHead.setHeadOrientation(orientation)
            #config.log(f"headOrientation, yaw: {config.oHead.yaw}, roll: {config.oHead.roll}, pitch: {config.oHead.pitch}")
            return orientation

        except Exception as e:
            config.log(f"could not request head orientation from cart: {e}")
            return None
    else:
        return None


def checkForObstacle():

    try:
        obstacleDist = config.servers['cartControl'].conn.root.exposed_getObstacleDistances()
    except Exception as e:
        config.log(f"could not get obstacle distances {e}")
        obstacleDist = None

    return obstacleDist

"""
def stopCheckingForObstacle():

    try:
        config.servers['kinect'].conn.root.exposed_stopMonitoring()

    except Exception as e:
        config.log(f"could not request stopping obstacle monitoring with depth cam {e}")
"""

def requestRotation(direction, speed, relativeAngle):

    # request rotation from cart
    config.oCart.rotating = True
    if direction == config.Direction.ROTATE_LEFT:
        config.servers['cartControl'].conn.root.exposed_rotateRelative(relativeAngle, speed)
    else:
        config.servers['cartControl'].conn.root.exposed_rotateRelative(-relativeAngle, speed)


def requestMove(direction, speed, distanceMm, distanceMonitoring):

    if connected('cartControl'):
        if distanceMonitoring:
            config.servers['cartControl'].conn.root.exposed_startMonitoring()
        else:
            config.servers['cartControl'].conn.root.exposed_stopMonitoring()

    if connected('cartControl'):
        config.oCart.moving = True
        config.servers['cartControl'].conn.root.exposed_move(direction.value, int(speed), int(distanceMm))


def adjustCartPosition(x, y, degrees):

    # navManager local cart position correction (not sent to the cart yet)
    config.oCart.xCorr += x
    config.oCart.yCorr += y
    config.oCart.degreesCorr += degrees

    if connected('cartControl'):
        config.servers['cartControl'].conn.root.exposed_adjustCartPosition(config.oCart.xCorr, config.oCart.yCorr, config.oCart.degreesCorr)
        config.oCart.xCorr = 0
        config.oCart.yCorr = 0
        config.oCart.degreesCorr = 0


def servoRequestPos(servoName, position, duration):
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestServoPos(servoName, position, duration)


def servoRequestDeg(servoName, degrees, duration):
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestServoDegrees(servoName, degrees, duration)


def servoGetPosition(servoName):
    if connected("robotControl"):
        position, degrees, moving = config.servers['robotControl'].conn.root.exposed_getPosition(servoName)
        return position, degrees, moving


def servoSetAutoDetach(servoName, duration):
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_setAutoDetach(servoName, duration)


def servoRest(servoName):
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_moveToRestDegrees(servoName)


def servoRestAll():
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestRestAll()


def servoStop(servoName):
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestServoStop(servoName)


def servoStopAll():
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestAllServosStop()


def requestMovePose():
    if connected("robotControl"):
        config.servers['robotControl'].conn.root.exposed_requestMovePose()


@handleRobotException
def pinHigh(pinList):
    config.log(f"request pin high: {pinList}")
    config.servers['robotControl'].conn.root.exposed_pinHigh(pinList)


@handleRobotException
def pinLow(pinList):
    config.log(f"request pin low: {pinList}")
    config.servers['robotControl'].conn.root.exposed_pinLow(pinList)


@handleCartException
def publishLifeSignalToCart():
    config.servers['cartControl'].conn.root.exposed_clientLifeSignal(config.localIp, config.MY_RPC_PORT)

@handleRobotException
def publishLifeSignalToRobot():
    config.servers['robotControl'].conn.root.exposed_clientLifeSignal(config.localIp, config.MY_RPC_PORT)

def publishLifeSignal():
    publishLifeSignalToCart()
    publishLifeSignalToRobot()