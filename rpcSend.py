
import os
import time
import rpyc

import config
import rpcReceive
import guiUpdate
import guiLogic




def connected(server):
    if config.navManagerServers[server]['simulated']:
        config.log(f"{server} simulated")
        return False
    if config.navManagerServers[server]['conn'] is None:
        config.log(f"no {server} connection established")
        return False
    return True


def setSimulationMask(kinect, aruco, cartControl, servoControl):

    config.navManagerServers['kinect']['simulated'] = kinect
    config.navManagerServers['aruco']['simulated'] = aruco
    config.navManagerServers['cartControl']['simulated'] = cartControl
    config.navManagerServers['servoControl']['simulated'] = servoControl



def startSlaveServer(server, taskOrchestrator):

    print()
    oServer = config.objectview(config.navManagerServers[server])
    if oServer.simulated:
        config.log(f"task {server} is set as simulated")
        return True

    # ask taskOrchestrator to start a server
    config.log(f"request server start from taskOrchestrator '{server}'")
    serverAlreadyRunning = False
    try:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'try'})
        ip, port, serverAlreadyRunning = config.taskOrchestrator.root.exposed_startServer(server)
        if not serverAlreadyRunning:
            oServer.startRequested = time.time()
        else:
            oServer.startRequested = time.time() - oServer.startupTime

    except Exception as e:
        config.log(f"starting server '{server}' failed, exception: {str(e)}")
        return False

    #oServer.ip = ip
    #oServer.port = port

    # allow task some time to startup if not already running
    if serverAlreadyRunning:
        config.log(f"task '{server}' was already running")
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'up'})
    else:
        config.log(f"task {server} was not running, wait {oServer.startupTime} seconds for getting ready")
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'wait'})
        #time.sleep(oServer.startup)


def terminateSlaveServer(server):
    # request slave termination from taskOrchestrator
    try:
        config.taskOrchestrator.root.exposed_stopServer(server)
    except Exception as e:
        config.log(f"could not request task stop '{server}' from taskOrchestrator,{e}")


def neededServersRunning(serversNeeded):
    serversRunning = []
    for server in serversNeeded:
        if config.navManagerServers[server]['conn'] is not None:
            serversRunning.append(server)
        if config.navManagerServers[server]['simulated']:
            serversRunning.append(server)
    diff = list(set(serversNeeded) - set(serversRunning))
    if len(diff) > 0:
        config.log(f"missing running servers for requested command: {diff}")
        config.setTask("notask")
        return False
    return True



def restartServers():
    for server in config.navManagerServers:
        terminateSlaveServer(server)
        startSlaveServer(server, config.taskOrchestrator)


def queryCartInfo():
    '''
    query cart task for current values
    cartOrientation, cartLocationX, cartLocationY, cartMoving, cartRotating
    '''
    if connected('cartControl'):
        #cartOrientation, cartLocationX, cartLocationY, cartMoving, cartRotating = config.navManagerServers['cartControl']['conn'].root.exposed_getCartInfo()
        config.oCart.orientation, config.oCart.x, config.oCart.y, config.oCart.moving, config.oCart.rotating = config.navManagerServers['cartControl']['conn'].root.exposed_getCartInfo()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO.value})
        config.log(f"queryCartInfo: {config.oCart.orientation}, {config.oCart.x}, {config.oCart.y}, {config.oCart.moving}, {config.oCart.rotating}")
    else:
        config.oCart.orientation, config.oCart.x, config.oCart.y, config.oCart.moving, config.oCart.rotating = (0, 0, 0, False, False)


def getEyecamImage():
    if connected('aruco'):
        try:
            return rpyc.classic.obtain(config.navManagerServers['aruco']['conn'].root.getEyecamImage())
        except Exception as e:
            config.log(f"could not get eyecam image: {e}")
            return None
    else:
        return None


def getCartcamImage():
    if connected('aruco'):
        try:
            return rpyc.classic.obtain(config.navManagerServers['aruco']['conn'].root.getCartcamImage())
        except Exception as e:
            config.log(f"could not get cartcam image: {e}")
            return None
    else:
        return None


def getDepth(orientation):
    if connected('kinect'):
        try:
            return rpyc.classic.obtain(config.navManagerServers['kinect']['conn'].root.exposed_getDepth(orientation))
        except Exception as e:
            config.log(f"could not get depth image: {e}")
            return None
    else:
        return None


def requestMove(direction, speed, distanceMm):
    if connected('cartControl'):
        config.oCart.moving = True
        config.navManagerServers['cartControl']['conn'].root.exposed_move(int(direction), int(speed), int(distanceMm))


def adjustCartPosition(x, y, o):

    # navManager local cart position correction (not sent to the cart yet)
    config.cartPositionCorrection['x'] += x
    config.cartPositionCorrection['y'] += y
    config.cartPositionCorrection['o'] += o

    if connected('cartControl'):
        config.navManagerServers['cartControl']['conn'].root.exposed_adjustCartPosition(x, y, o)
        config.cartPositionCorrection['x'] = 0
        config.cartPositionCorrection['y'] = 0
        config.cartPositionCorrection['o'] = 0


def lookForMarkers(camera, markerId):
    '''
    returns a list of dict with attribs:
    {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerOrientation'}
    :param camera: EYE_CAM or CART_CAM
    :param markerId: list of markerId's to look for, may be empty to return all markers found
    :return:
    '''
    if connected('aruco'):
        result = config.navManagerServers['aruco']['conn'].root.exposed_findMarkers(camera, markerId)
        markerFound = len(result) > 0
        return markerFound, result
    else:
        return False


def servoRequestPos(servoName, position, duration):
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_requestServoPos(servoName, position, duration)


def servoRequestDeg(servoName, degrees, duration):
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_requestServoDegrees(servoName, degrees, duration)


def servoGetPosition(servoName):
    if connected("servoControl"):
        position, degrees, moving = config.navManagerServers['servoControl']['conn'].root.exposed_getPosition(servoName)
        return position, degrees, moving


def servoSetAutoDetach(servoName, duration):
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_setAutoDetach(servoName, duration)


def servoRest(servoName):
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_moveToRestDegrees(servoName)


def servoRestAll():
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_requestRestAll()


def servoStop(servoName):
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_requestServoStop(servoName)


def servoStopAll():
    if connected("servoControl"):
        config.navManagerServers['servoControl']['conn'].root.exposed_requestAllServosStop()


def pinHigh(pinList):
    if connected("servoControl"):
        config.log(f"request pin high: {pinList}")
        config.navManagerServers['servoControl']['conn'].root.exposed_pinHigh(pinList)

def pinLow(pinList):
    if connected("servoControl"):
        config.log(f"request pin low: {pinList}")
        config.navManagerServers['servoControl']['conn'].root.exposed_pinLow(pinList)