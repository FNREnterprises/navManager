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

import navTasks
import navMap
import fullScanAtPosition

#taskOrchestrator = None
def setTask(requestor, newTask):

    config.log(f"new task set by {requestor}: {newTask}")
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


def navThread():

    """
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
        config.marvinShares.mapGuiUpdateQueue.put({'type': mg.SharedDataItems.ENVIRONMENT_ROOM})

    config.marvinShares.mapGuiUpdateQueue.put({'type': mg.SharedDataItems.CART_STATE})
    """
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

    config.roomDataLocal = config.getRoomData()
    config.scanLocationListLocal = config.getScanLocationList()
    config.markerListLocal = config.getMarkerList()

    # consume "outdated" entries from navManager request queue
    while True:
        try:
            request = config.marvinShares.navManagerRequestQueue.get(block=True, timeout=0.1)
        except queue.Empty:  # in case of empty queue update processDict only
            break
        except TimeoutError:  # in case of timeout update processDict only
            break


    if not navMap.loadFloorPlan(config.roomDataLocal.roomName):
        config.log(f"no current room data found, create a new floor plan")

        msg = {'msgType': mg.NavManagerCommands.SCAN_ROOM, 'sender': config.processName}
        config.marvinShares.navManagerRequestQueue.put(msg)


    # start the navigation thread (setup and loop)
    navThread = threading.Thread(target=navThread, args={})
    navThread.setName("navThread")
    navThread.start()

    # start map update thread navMap.updateFloorPlan
    #mapThread = threading.Thread(target=threadProcessImages.loop, args={})
    #mapThread.setName('threadProcessImages')
    #mapThread.start()

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

        cmd = request['msgType']
        if cmd == mg.NavManagerCommands.SCAN_ROOM:
            navMap.createFloorPlan()
            msg = {'msgType': mg.NavManagerCommands.FULL_SCAN_AT_POSITION, 'sender': config.processName}
            config.marvinShares.navManagerRequestQueue.put(msg)
            #navManager.setTask("fullScanAtPosition")

        elif cmd == mg.NavManagerCommands.FULL_SCAN_AT_POSITION:
            setTask(f"cmd by queue","fullScanAtPosition")       # check on all markers

        elif cmd == mg.NavManagerCommands.TAKE_IMAGE_RESULT:
            #config.log(f"handle cmd TAKE_IMAGE_RESULT")
            if config.camRequest[request['camType']] is None:
                config.log(f"TAKE_IMAGE_RESULT received without request")
            else:
                config.log(f"TAKE_IMAGE_RESULT {request['camType']=}, {request['imageId']=}, {request['success']=}")
                if request['success']:
                    #config.log(f"set success in camRequest")
                    config.camRequest[request['camType']].setSuccess()
                else:
                    #config.log(f"set failure in camRequest")
                    config.camRequest[request['camType']].setFailure()

        else:
           config.log(f"navManager, unknown request received: {request}")



