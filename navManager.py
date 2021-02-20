""" Position InMoov robot in front of an Aruco Marker
"""

import os
import sys
import time
import logging

#from multiprocessing.managers import SyncManager

import threading

import queue
from typing import List

from marvinglobal import marvinglobal as mg
from marvinglobal import marvinShares
from marvinglobal import skeletonClasses


import config

import navTasks
import navMap
import marker

#class ShareManager(SyncManager): pass


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

def addToProcessSimulated(simulated:List):
    config.processSimulated = list(set(config.processSimulated + simulated))

def removeFromProcessSimulated(simulated:List):
    for s in simulated:
        config.processSimulated.remove(s)



def setup():

    config.marvinShares = marvinShares.MarvinShares()  # shared data

    if not config.marvinShares.sharedDataConnect(config.processName):
        config.log(f"could not connect with marvinData")
        os._exit(3)

    # add own process to shared process list
    config.marvinShares.updateProcessDict(config.processName)


    # set initial task (or "notask")
    setTask("notask")

    # optional: set simulation status of subtasks
    addToProcessSimulated(['cartControl', 'skeletonControl'])


    navMap.loadMapInfo()

    # try to load existing floor plan
    if config.fullScanDone and navMap.loadFloorPlan(config.room):
        navMap.loadScanLocations()
        navMap.loadMarkerList()
    else:
        config.log(f"could not load a floor plan")
        setTask('createFloorPlan')

    if config.floorPlan is not None:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

#    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO})

    # check for assigned task
    while True:
        navTasks.checkForTasks()
        time.sleep(0.1)



if __name__ == "__main__":

    ##########################################################
    # initialization
#    config.initLogging()

    config.log("navManager started")

    setup()

    while True:

        # wait for incoming task requests
        # cyclic update of process dict while running
        request = (config.processName, mg.NavManagerCommand.NEW_TASK, 'noTask')

        try:
            config.marvinShares.updateProcessDict(config.processName)
            request = config.marvinShares.navManagerRequestQueue.get(block=True, timeout=1)
        except queue.Empty: # in case of empty queue update processDict only
            continue
        except TimeoutError: # in case of timeout update processDict only
            continue
        except Exception as e:
            config.log(f"exception in waiting for navManager requests, {e=}, going down")
            os._exit(11)

        try:
            if request['cmd'] == mg.NavManagerCommand.NEW_TASK:
                newTask = request[2]
                config.log(f"navManagerRequestQueue, request for new task received: {request}")

            if request['cmd'] == mg.NavManagerCommand.ARUCO_CHECK_RESULT:
                config.arucoCodes = request[2]
                config.log(f"navManagerRequestQueue, result for aruco check received: {request}")

                # add Markers to marker list
                config.log(f"result for marker search with headDegree: {request['headYaw']}, cartDegree: {request['headYaw']}")

                markersFound = len(request['markers']) > 0    #aruco.lookForMarkers("EYE_CAM", [], config.oHead.getHeadYaw())
                # result =  list of {'markerId', 'distanceCamToMarker', 'angleToMarker', 'markerDegrees'}

                if markersFound:
                    config.log(f"markers found: {markersFound}")
                    marker.updateMarkerFoundResult(markersFound, 'EYE_CAM', config.oCart.getCartX(), config.oCart.getCartY(),
                                            request['headYaw'], request['cartYaw'])

                    # print(navGlobal.markerList)
                    # stop scan if we have found a requested marker
                    for markerInfo in markersFound:
                        if markerInfo['markerId'] in config.lookForMarkers:
                            config.log(f"requested marker found, markerId: {markerInfo['markerId']}")
                            break
                else:
                    config.log(f"no markers found")





        except Exception as e:
            config.log(f"invalid request received: {request}")


    # start the navigation thread (setup and loop)
    #navThread = threading.Thread(target=setup, args={})
    #navThread.setName("navThread")
    #navThread.start()

    # start map update thread navMap.updateFloorPlan
    #mapThread = threading.Thread(target=threadProcessImages.loop, args={})
    #mapThread.setName('threadProcessImages')
    #mapThread.start()

    # startQtGui()
    #guiLogic.startQtGui()

