""" Position InMoov robot in front of an Aruco Marker
"""

import os
import sys
import time
import logging
import rpyc
import threading
from PyQt5 import QtWidgets

import config
import marker

import rpcReceive
import rpcSend
import navTasks
import navMap
import guiLogic
import guiUpdate
import watchDog

#taskOrchestrator = None
def setTask(newTask):

    # we can have a stack of started tasks and can return to the last requested task
    if newTask == "pop":

        #log(f"taskStack before pop: {taskStack}")

        # if we have no stacked tasks set task to notask
        if len(config.taskStack) < 2:
            config.log(f"can not pop, task stack is empty")
            config.task = "notask"
            return
        else:
            config.taskStack.pop()
            config.task = config.taskStack[-1]
            config.log(f"newTask: {config.task}, taskStack after pop: {config.taskStack}")
            return

    # if we have run into a problem or successfully finished all open tasks we have notask
    if newTask == "notask":
        if config.task != 'notask':
            config.log(f"new task requested: {newTask}")
        config.taskStack = []
    else:
        config.taskStack.append(newTask)
        print()     # make new task visible in the log

    config.task = newTask

    if len(config.taskStack) > 0:
        config.log(f"taskStack: {config.taskStack}")


def setup():

    config.createObjectViews()

    # set initial task (or "notask")
    setTask("notask")

    # optional: set simulation status of subtasks
    #rpcSend.setSimulationMask(kinect=False, aruco=False, cartControl=False, servoControl=False)
    rpcSend.setSimulationMask(kinect=False, aruco=False, cartControl=False, servoControl=False)

    # wait for log listener to start up
    time.sleep(1)


    # check for running task orchestrator on subsystem
    subSystemIp = "192.168.0.17"
    config.log(f"trying to contact taskOrchestrator on subsystem {subSystemIp}:20000")

    try:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': 'taskOrchestrator', 'state': 'try'})
        config.taskOrchestrator = rpyc.connect(subSystemIp, 20000, service = rpcReceive.rpcListener)
    except Exception as e:
        config.log(f"could not connect with taskOrchestrator, {e}")
        os._exit(1)

    running = False
    config.log(f"connect with taskOrchestrator successful, try to getLifeSignal")
    good = False
    try:
        good = config.taskOrchestrator.root.exposed_getLifeSignal(config.MY_IP, config.MY_PORT)
    except Exception as e:
        config.log(f"failed to get response, taskOrchestrator on {subSystemIp} not running?")
        os._exit(2)

    if good:
        config.log(f"life signal from task orchestrator received")
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': 'taskOrchestrator', 'state': 'ready'})
    else:
        config.log(f"could not get life signal from task orchestrator on subsystem {subSystemIp}")
        os._exit(3)

    for server in config.navManagerServers:
        if not config.navManagerServers[server]['simulated']:
            config.log(f"start server thread for {server}")
            serverThread = threading.Thread(target=watchDog.watchConnection, args={server})
            serverThread.setName(server)
            serverThread.start()


    navMap.loadMapInfo()

    # try to load existing floor plan
    if config.fullScanDone and navMap.loadFloorPlan(config.room):
        navMap.loadScanLocations()
        marker.loadMarkerInfo()
    else:
        setTask('createFloorPlan')

    if config.floorPlan is not None:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO.value})

    # wait for ready messages from servers
    for _ in range(15):
        allReady = True
        for server in config.navManagerServers:
            if not config.navManagerServers[server]['serverReady']:
                allReady = False

        if not allReady:
            time.sleep(1)

    # check for assigned task
    while True:
        navTasks.checkForTasks()
        time.sleep(0.1)


def restartServer(server):
    config.navManagerServers[server]['conn'] = None
    config.navManagerServers[server]['serverReady'] = False
    config.navManagerServers[server]['startRequested'] = time.time()
    config.taskOrchestrator.root.restartServer(server)
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})


def startQtGui():

    # start gui (this starts also the servo update thread)
    app = QtWidgets.QApplication(sys.argv)
    ui = guiLogic.gui(None)
    ui.show()
    sys.exit(app.exec_())



if __name__ == "__main__":

    windowName = "pcjm//navManager"
    os.system("title " + windowName)
    #hwnd = win32gui.FindWindow(None, windowName)
    #win32gui.MoveWindow(hwnd, 2000,0,1200,1200,True)

    ##########################################################
    # initialization
    # Logging, renaming old logs for reviewing ...
    baseName = "log/navManager"
    oldName = f"{baseName}9.log"
    if os.path.isfile(oldName):
        os.remove(oldName)
    for i in reversed(range(9)):
        oldName = f"{baseName}{i}.log"
        newName = f"{baseName}{i+1}.log"
        if os.path.isfile(oldName):
            os.rename(oldName, newName)
    oldName = f"{baseName}.log"
    newName = f"{baseName}0.log"
    if os.path.isfile(oldName):
        try:
            os.rename(oldName, newName)
        except Exception as e:
            config.log(f"can not rename {oldName} to {newName}")

    logging.basicConfig(
        filename="log/navManager.log",
        level=logging.INFO,
        format='%(asctime)s - %(message)s',
        filemode="w")

    config.log("navManager started")
    # start the navigation thread (setup and loop)
    navThread = threading.Thread(target=setup, args={})
    navThread.setName("navThread")
    navThread.start()

    # start map update thread navMap.updateFloorPlan
    mapThread = threading.Thread(target=navMap.updateFloorPlanThread, args={})
    mapThread.setName('mapThread')
    mapThread.start()

    # startQtGui()
    guiThread = threading.Thread(target=startQtGui, args={})
    guiThread.setName('guiThread')
    guiThread.start()

    from rpyc.utils.server import ThreadedServer
    print(f"start listening on port {config.MY_PORT}")
    listener = ThreadedServer(rpcReceive.rpcListener, port=config.MY_PORT)
    listener.start()

