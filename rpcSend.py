
import os
import time
import numpy as np
import rpyc

import config
import rpcReceive

# involved computers
marvin = "192.168.0.17"
pcjm = "192.168.0.14"


# rpc
MY_IP = pcjm
navManager = None
MY_RPC_PORT = 20010

# NOTE master for ip and ports is the taskOrchestrator
# Kinect must be started first, otherwise driver complains
navManagerServers = { 'kinect':       {'simulated': False, 'startup': 10, 'ip': marvin, 'port': 20003, 'conn': None},
                      'cartControl':  {'simulated': False, 'startup':15, 'ip': marvin, 'port': 20001, 'conn': None},
                      'aruco':        {'simulated': False, 'startup': 10, 'ip': marvin, 'port': 20002, 'conn': None},
                      'servoControl': {'simulated': False, 'startup':20, 'ip': marvin, 'port': 20004, 'conn': None}
                      }

taskOrchestrator = None


def updateConnection(server, conn):
    global navManagerServers
    navManagerServers[server]['conn'] = conn

def setSimulationMask(kinect, aruco, cartControl, servoControl):

    global navManagerServers

    navManagerServers['kinect']['simulated'] = kinect
    navManagerServers['aruco']['simulated'] = aruco
    navManagerServers['cartControl']['simulated'] = cartControl
    navManagerServers['servoControl']['simulated'] = servoControl


def startServers():

    global taskOrchestrator

    # check for running task orchestrator on subsystem
    subSystemIp = "192.168.0.17"
    config.log(f"trying to contact taskOrchestrator on subsystem {subSystemIp}:20000")

    #with xmlrpc.ServerProxy(f"http://{subSystemIp}:20000", transport = RequestsTransport()) as proxy:
    try:
        taskOrchestrator = rpyc.connect(subSystemIp, 20000, service = rpcReceive.navManagerListener)
    except Exception as e:
        config.log("could not connect to taskOrchestrator, {e}")
        os._exit(1)

    running = False
    config.log("connect successful, try to getLifeSignal")
    try:
        running = taskOrchestrator.root.exposed_getLifeSignal()
    except Exception as e:
        config.log(f"failed to get response, taskOrchestrator on {subSystemIp} not running?")
        os._exit(2)
    if not running:
        config.log(f"could not get life signal from task orchestrator on subsystem {subSystemIp}")
        os._exit(3)

    for server in navManagerServers:
        startSlaveServer(server, taskOrchestrator)


def startSlaveServer(server, taskOrchestrator):

    global navManagerServers

    print()
    oServer = config.objectview(navManagerServers[server])
    if oServer.simulated:
        config.log(f"task {server} is set as simulated")
        return True

    # ask taskOrchestrator to start a server
    #with xmlrpc.client.ServerProxy(f"http://{marvin}:20000") as proxy:
    config.log(f"request task start from taskOrchestrator '{server}'")
    taskAlreadyRunning = False
    try:
        ip, port, taskAlreadyRunning = taskOrchestrator.root.exposed_startTask(server)

    except Exception as e:
        config.log(f"starting server '{server}' failed, exception: {str(e)}")
        os._exit(0)

    #oServer.ip = ip
    #oServer.port = port

    # allow task some time to startup if not already running
    if taskAlreadyRunning:
        config.log(f"task '{server}' was already running")
    else:
        config.log(f"task {server} was not running, wait {oServer.startup} seconds for getting ready")
        time.sleep(oServer.startup)

    config.log(f"try to connect with '{server}' at {oServer.ip}, {oServer.port}")
    try:
        oServer.conn = rpyc.connect(oServer.ip, oServer.port, service = rpcReceive.navManagerListener)
    except Exception as e:
        config.log(f"could not connect with '{server}', exception: {str(e)}")


    config.log(f"my PID {os.getpid()}")

    config.log(f"request life signal from '{server}'")
    good = False
    try:
        good = oServer.conn.root.exposed_getLifeSignal(os.getpid())
    except Exception as e:
        config.log(f"requesting life signal from '{server}' failed, exception: {str(e)}")
        taskOrchestrator.root.exposed_stopTask(server)
        oServer.conn = None
    if good:
        config.log(f"life signal request from '{server}' successful")

        # place to get basic information from subtasks
        if server == 'servoControl':
            # get current servo information
            jsonMsg = oServer.conn.root.exposed_getServoCurrentList()
            rpcReceive.initServoCurrent(jsonMsg)


def terminateSlaveServer(server):
    # request slave termination from taskOrchestrator
    try:
        taskOrchestrator.exposed_stopTask(server)
    except Exception as e:
        config.log(f"could not request task stop '{server}' from taskOrchestrator,{e}")


def neededTasksRunning(tasksNeeded):
    tasksRunning = []
    for task in tasksNeeded:
        if navManagerServers[task]['conn'] is not None:
            tasksRunning.append(task)
    diff = list(set(tasksNeeded) - set(tasksRunning))
    if len(diff) > 0:
        config.log(f"missing running tasks for requested command: {diff}")
        config.setTask("notask")
        return False
    return True


def queryCartInfo():
    '''
    query cart task for current values
    '''
    if navManagerServers['cartControl']['simulated'] or navManagerServers['cartControl']['conn'] is None:
        cartOrientation, cartLocationX, cartLocationY, cartMoving, cartRotating = (0, 0, 0, False, False)
    else:
        cartOrientation, cartLocationX, cartLocationY, cartMoving, cartRotating = navManagerServers['cartControl']['conn'].root.exposed_getCartInfo()
    config.setCartInfo((cartOrientation, cartLocationX, cartLocationY, cartMoving, cartRotating))


def getEyecamImage():
    return rpyc.classic.obtain(navManagerServers['aruco']['conn'].root.getEyecamImage())
    #transferImg = navManagerServers['aruco']['conn'].root.exposed_getEyecamImage()
    #rawImg = np.fromstring(transferImg.data, dtype=np.uint8)
    #img = np.reshape(rawImg,(640,480,3))
    #return img


def getCartcamImage():
    return rpyc.classic.obtain(navManagerServers['aruco']['conn'].root.getCartcamImage())
    #transferImg = navManagerServers['aruco']['conn'].root.exposed_getCartcamImage()
    #rawImg = np.fromstring(transferImg.data, dtype=np.uint8)
    #img = np.reshape(rawImg,(640,480,3))
    #return img


def getDepth(orientation):
    return navManagerServers['kinect']['conn'].root.exposed_getDepth(orientation)


def requestMove(direction, speed, distanceMm):
    #config.cartRequest.root.move(direction, speed, distanceMm)
    navManagerServers['cartControl']['conn'].root.exposed_move(int(direction), int(speed), int(distanceMm))


def lookForMarkers(camera, markerId):
    '''
    returns a list of dict with attribs:
    {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerOrientation'}
    :param camera: EYE_CAM or CART_CAM
    :param markerId: list of markerId's to look for, may be empty to return all markers found
    :return:
    '''

    result = navManagerServers['aruco']['conn'].root.exposed_findMarkers(camera, markerId)

    markerFound = len(result) > 0

    return markerFound, result


def getLifeSignal(server):
    answer = False
    try:
        answer = navManagerServers[server]['conn'].root.exposed_getLifeSignal(os.getpid())
    except Exception as e:
        config.log(f"exception in getLifeSignal from {server}: {e}")

        # set connection to None
        navManagerServers[server]['conn'] = None
        return False

    return answer


def servoRequestPos(servoName, position, duration):
    navManagerServers['servoControl']['conn'].root.exposed_requestServoPos(servoName, position, duration)


def servoRequestDeg(servoName, degrees, duration):
    navManagerServers['servoControl']['conn'].root.exposed_requestServoDegrees(servoName, degrees, duration)


def servoGetPosition(servoName):
    position, degrees, moving = navManagerServers['servoControl']['conn'].root.exposed_getPosition(servoName)
    return position, degrees, moving


def servoSetAutoDetach(servoName, duration):
    navManagerServers['servoControl']['conn'].root.exposed_setAutoDetach(servoName, duration)


def servoRest(servoName):
    navManagerServers['servoControl']['conn'].root.exposed_moveToRestDegrees(servoName)


def servoRestAll():
    navManagerServers['servoControl']['conn'].root.exposed_requestRestAll()


def servoStop(servoName):
    navManagerServers['servoControl']['conn'].root.exposed_requestServoStop(servoName)


def servoStopAll():
    navManagerServers['servoControl']['conn'].root.exposed_requestAllServosStop()


def watchServers():
    """
    for all active servers send regularly a heartbeat request
    :return:
    """
    global navManagerServers

    # loop here
    while True:
        for server in navManagerServers:
            oServer = config.objectview(navManagerServers[server])

            if oServer.simulated:
                continue

            if oServer.conn is None:

                # we have no connection handle, try to connect
                try:
                    config.log(f"try to connect with '{server}' ")
                    oServer.conn = rpyc.connect(oServer.ip, oServer.port, service = rpcReceive.navManagerListener)
                except Exception as e:
                    config.log(f"could not connect with '{server}', exception: {str(e)}")

            else:
                #config.log(f"request Life signal from subsystem")
                good = False
                try:
                    good = getLifeSignal(server)
                except Exception as e:
                    config.log(f"getting live signal from {server} failed")
                    taskOrchestrator.root.exposed_stopTask(server)
                    oServer.conn = None

            # place for updating gui
            #if not good:
            #   config.addToUpdateQueue({'type': config.CONNECTION_UPDATE, 'state': config.WAIT_FOR_CONNECTION})

        time.sleep(config.SERVER_WATCH_INTERVAL)
