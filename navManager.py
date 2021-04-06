""" Position InMoov robot in front of an Aruco Marker
"""

import os
import sys
import time
import threading
import queue
import subprocess

from marvinglobal import marvinglobal as mg
from marvinglobal import marvinShares

import config
import environment

import navTasks
import navMap
import guiLogic
import guiUpdate
#import threadWatchConnections
import threadProcessImages
import fullScanAtPosition

#taskOrchestrator = None
def setTask(newTask):

    # we can have a stack of started tasks and can return to the previous task
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

    # try to make it without this
    #config.createObjectViews()

    # set initial task (or "notask")
    setTask("notask")

    #navMap.loadMarkerList()

    # try to load existing floor plan
    if config.fullScanDone and navMap.loadFloorPlan(config.environment):
        navMap.loadScanLocations()
        navMap.loadMarkerList()
    else:
        config.log(f"could not load a floor plan")
        setTask('noTask')

    if config.floorPlan is not None:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})

    # wait for ready messages from servers
    for _ in range(15):
        allReady = True
        for server in config.servers:
            if not config.servers[server].simulated:
                if not config.servers[server].connectionState == 'ready':
                    allReady = False

        if not allReady:
            time.sleep(1)

    # check for assigned task
    while True:
        navTasks.checkForTasks()
        time.sleep(0.1)





if __name__ == "__main__":


    ##########################################################
    # connect with shared data
    config.marvinShares = marvinShares.MarvinShares()
    if not config.marvinShares.sharedDataConnect(config.processName):
        config.log(f"could not connect with marvinData")
        os._exit(10)

    # add own process to shared process list
    config.marvinShares.updateProcessDict(config.processName)

    # check for running cart control
    if "cartControl" not in config.marvinShares.processDict.keys():
        config.log(f"navManager needs a running cartControl, trying to start it")

        config.marvinShares.startProcess("cartControl")

        timeoutSeconds = 10
        timeout = time.time() + timeoutSeconds
        while "cartControl" not in config.marvinShares.processDict.keys() and time.time() < timeout:
            time.sleep(1)

        if time.time() > timeout:
            config.log(f"cartControl did not start up within {timeoutSeconds}, going down")
            os._exit(1)


    #config.roomDataLocal = config.marvinShares.environmentDict.get(mg.SharedDataItems.ENVIRONMENT_ROOM)
    config.cartLocationLocal = config.marvinShares.cartDict.get(mg.SharedDataItems.CART_LOCATION)

    # load last used room information
    config.environment = environment.Room()
    config.scanLocations = environment.ScanLocations()
    config.markerList = environment.MarkerList()

    if not navMap.loadFloorPlan(config.roomDataLocal.roomName):
        config.log(f"no current room data found, create a new floor plan")

        msg = {'cmd': mg.NavManagerCommands.SCAN_ROOM, 'sender': config.processName}
        config.marvinShares.navManagerRequestQueue.put(msg)


    # start the navigation thread (setup and loop)
    #navThread = threading.Thread(target=setup, args={})
    #navThread.setName("navThread")
    #navThread.start()

    # start map update thread navMap.updateFloorPlan
    mapThread = threading.Thread(target=threadProcessImages.loop, args={})
    mapThread.setName('threadProcessImages')
    mapThread.start()

    config.log(f"{config.processName} ready, waiting for requests")
    config.log(f"---------------")
    request = {}

    # wait for requests, update process list
    while True:

        try:
            config.marvinShares.updateProcessDict(config.processName)
            request = config.marvinShares.navManagerRequestQueue.get(block=True, timeout=1)
        except queue.Empty: # in case of empty queue update processDict only
            continue
        except TimeoutError: # in case of timeout update processDict only
            continue
        except Exception as e:
            config.log(f"exception in waiting for navManager request, {e=}, going down")
            config.marvinShares.removeProcess(config.processName)
            os._exit(11)

        config.log(f"navManagerRequestQueue, request received: {request}")


        cmd = request['cmd']
        if cmd == mg.NavManagerCommands.SCAN_ROOM:
            navMap.createFloorPlan()
            msg = {'cmd': mg.NavManagerCommands.FULL_SCAN_AT_POSITION, 'sender': config.processName}
            config.marvinShares.navManagerRequestQueue.put(msg)
            #navManager.setTask("fullScanAtPosition")

        elif cmd == mg.NavManagerCommands.FULL_SCAN_AT_POSITION:
            fullScanAtPosition.fullScanAtPosition([])       # check on all markers

        elif cmd == mg.NavManagerCommands.TAKE_IMAGE_RESULT:
            if not request['success']:
                # stop current task
                config.task = None

        elif cmd == mg.NavManagerCommands.ARUCO_CHECK_RESULT:
            if not request['success']:
                # stop current task
                config.task = None


        else:
           config.log(f"navManager, unknown request received: {request}")



