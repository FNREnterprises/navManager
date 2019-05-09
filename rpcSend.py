
import time
import rpyc

import config
import guiUpdate
import navManager
import winsound



def simulated(server):
    if config.servers[server].simulated:
        config.log(f"{server} simulated")
        return True
    return False


def connected(server):
    if config.servers[server].conn is None:
        config.log(f"no {server} connection established")
        return False
    return True


def setSimulationMask(kinect, aruco, cartControl, servoControl):

    config.servers['kinect'].simulated = kinect
    config.servers['aruco'].simulated = aruco
    config.servers['cartControl'].simulated = cartControl
    config.servers['servoControl'].simulated = servoControl



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


def queryCartInfo():
    """
    query cart task for current values
    cartDegrees, cartLocationX, cartLocationY, cartMoving, cartRotating
    """
    if connected('cartControl'):
        #cartDegrees, cartLocationX, cartLocationY, cartMoving, cartRotating = config.servers['cartControl'].conn.root.exposed_getCartInfo()
        x,y,o,config.oCart.moving,config.oCart.rotating = config.servers['cartControl'].conn.root.exposed_getCartInfo()
        config.oCart.setX(x)
        config.oCart.setY(y)
        config.oCart.setDegrees(o)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})
        config.log(f"new CartInfo: x: {config.oCart.getX()}, y: {config.oCart.getY()}, degrees: {config.oCart.getDegrees()}, moving: {config.oCart.moving}, rotating: {config.oCart.rotating}")
    else:
        config.log(f"no connection with cartControl")


def queryBatteries():
    if connected('cartControl'):

        config.batteryStatus = config.servers['cartControl'].conn.root.getBatteryStatus()

        if config.batteryStatus is not None:
            #config.log(f"received battery info")
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.BATTERY_UPDATE})

            if config.batteryStatus['percent'] < 20:
                winsound.PlaySound('sound.wav', winsound.SND_FILENAME)


def getEyecamImage():
    if connected('aruco'):
        try:
            return rpyc.classic.obtain(config.servers['aruco'].conn.root.getEyecamImage())
        except Exception as e:
            config.log(f"could not get eyecam image: {e}")
            return None
    else:
        return None


def getCartcamImage():
    if connected('aruco'):
        try:
            return rpyc.classic.obtain(config.servers['aruco'].conn.root.getCartcamImage())
        except Exception as e:
            config.log(f"could not get cartcam image: {e}")
            return None
    else:
        return None


def getDepth(degrees):
    if connected('kinect'):
        try:
            return rpyc.classic.obtain(config.servers['kinect'].conn.root.exposed_getDepth(degrees))

        except Exception as e:
            config.log(f"could not get depth image: {e}")
            return None

    else:
        return None


def checkForObstacle():

    try:
        config.servers['kinect'].conn.root.exposed_startMonitoring()
    except Exception as e:
        config.log(f"could not request monitoring {e}")

    # obstacles should get published
    time.sleep(5)

    try:
        config.servers['kinect'].conn.root.exposed_stopMonitoring()

    except Exception as e:
        config.log(f"could not request monitoring {e}")


def requestRotation(direction, speed, relativeAngle):

    # request rotation from cart
    config.oCart.rotating = True
    if direction == config.Direction.ROTATE_LEFT:
        config.servers['cartControl'].conn.root.exposed_rotateRelative(relativeAngle, speed)
    else:
        config.servers['cartControl'].conn.root.exposed_rotateRelative(-relativeAngle, speed)


def requestMove(direction, speed, distanceMm):

    if connected('cartControl'):
        config.oCart.moving = True
        config.servers['cartControl'].conn.root.exposed_move(direction.value, int(speed), int(distanceMm))


def adjustCartPosition(x, y, o):

    # navManager local cart position correction (not sent to the cart yet)
    config.oCart.xCorr += x
    config.oCart.yCorr += y
    config.oCart.oCorr += o

    if connected('cartControl'):
        config.servers['cartControl'].conn.root.exposed_adjustCartPosition(config.oCart.xCorr, config.oCart.yCorr, config.oCart.oCorr)
        config.oCart.xCorr = 0
        config.oCart.yCorr = 0
        config.oCart.oCorr = 0


def lookForMarkers(camera, markerIdList):
    """
    returns a list of dict with attribs:
    {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerDegrees'}
    :param camera: EYE_CAM or CART_CAM
    :param markerIdList: list of markerId's to look for, may be empty to return all markers found
    :return:
    """
    if connected('aruco'):
        try:
            result = rpyc.classic.obtain(config.servers['aruco'].conn.root.exposed_findMarkers(camera, markerIdList, config.oCart.getDegrees()))
        except Exception as e:
            config.log(f"could not request marker evaluation: {e}")
            return False

        #result = config.servers['aruco'].conn.root.exposed_findMarkers(camera, markerId)
        return len(result) > 0, result
    else:
        return False


def servoRequestPos(servoName, position, duration):
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestServoPos(servoName, position, duration)


def servoRequestDeg(servoName, degrees, duration):
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestServoDegrees(servoName, degrees, duration)


def servoGetPosition(servoName):
    if connected("servoControl"):
        position, degrees, moving = config.servers['servoControl'].conn.root.exposed_getPosition(servoName)
        return position, degrees, moving


def servoSetAutoDetach(servoName, duration):
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_setAutoDetach(servoName, duration)


def servoRest(servoName):
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_moveToRestDegrees(servoName)


def servoRestAll():
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestRestAll()


def servoStop(servoName):
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestServoStop(servoName)


def servoStopAll():
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestAllServosStop()


def requestMovePose():
    if connected("servoControl"):
        config.servers['servoControl'].conn.root.exposed_requestMovePose()


def pinHigh(pinList):
    if connected("servoControl"):
        config.log(f"request pin high: {pinList}")
        config.servers['servoControl'].conn.root.exposed_pinHigh(pinList)

def pinLow(pinList):
    if connected("servoControl"):
        config.log(f"request pin low: {pinList}")
        config.servers['servoControl'].conn.root.exposed_pinLow(pinList)

def powerKinect(newState):
    if connected("cartControl"):
        config.log(f"request power kinect: {newState}")
        config.servers['cartControl'].conn.root.exposed_powerKinect(newState)
