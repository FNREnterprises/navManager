
import time
import cv2
import os

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonCommandMethods
from marvinglobal import cartCommandMethods

#import inmoovGlobal
import config
#import rpcSend
import navMap
import navManager
import docking


def neededServersRunning(serversNeeded):
    serversRunning = config.marvinShares.processDict.keys()
    diff = list(set(serversNeeded) - set(serversRunning))
    if len(diff) > 0:
        config.log(f"missing running servers for requested command: {diff}")
        navManager.setTask("notask")
        return False
    return True


def checkForTasks():
    """
    this function gets called every 100 ms
    :return:
    """

    if config.task != config.prevTask:
        config.prevTask = config.task
        config.log(f"checkForTasks sees new task: {config.task}")

    if config.task == "restartServers":
#        for server in config.servers:
#            threadWatchConnections.tryToRestartServer(server)
         navManager.setTask("notask")

    elif config.task == "createFloorPlan":
        if neededServersRunning(['cartControl','skeletonControl','imageProcessing']):
            if navMap.createFloorPlan():
                navManager.setTask("fullScanAtPosition")
            else:
                config.log(f"could not create floor plan")
                navManager.setTask("notask")

    elif config.task == "fullScanAtPosition":
        if neededServersRunning(['cartControl','skeletonControl','imageProcessing']):

            if navMap.fullScanAtPosition():
                navManager.setTask("rover")
            else:
                config.log(f"could not do a full scan")
                navManager.setTask("notask")

    elif config.task == "rover":
        navMap.rover()

    elif config.task == "rebuildMap":
        navMap.rebuildMap()
        navManager.setTask("notask")

    elif config.task == "checkForPerson":
        '''
        get an eyecam image and try to find people in it
        '''
        neededProcesses = ['aruco']
        camDegrees = config.oCart.getCartYaw() + config.servoCurrent['head.rothead']['degrees']
        '''
        if not config.netsLoaded:
            analyzeImage.init()

        if rpcSend.neededServersRunning(neededProcesses):
            img = rpcSend.getImage(inmoovGlobal.EYE_CAM)
            if img is not None:

                frameFace, bboxes = analyzeImage.getFaceBox(img)
                if not bboxes:
                    print("No face detected")
                    navManager.setTask("notask")
                else:
                    cv2.imshow("inmoovEyeCamImage", img)
                    cv2.waitKey()
                    cv2.destroyAllWindows()

            else:
                config.log(f"could not take an eyecam image")
        '''
        navManager.setTask("notask")


    elif config.task == "takeCartcamImage":
        '''
        take cart cam image and show aruco marker result
        '''
        if neededServersRunning(['imageProcessing']):
            request = {'sender': config.processName, 'cmd': mg.ImageProcessingCommand.CHECK_FOR_ARUCO_CODE,
                   'cam': mg.CART_CAM, 'markers': []}
            config.marvinShares.imageProcessingQueue.put(request)
        navManager.setTask("notask")


    elif config.task == "takeEyecamImage":
        '''
        take eyecam image and show aruco marker result
        '''
        if neededServersRunning(['imageProcessing']):
            request = {'sender': config.processName, 'cmd': mg.ImageProcessingCommand.CHECK_FOR_ARUCO_CODE, 'cam': mg.EYE_CAM, 'markers': []}
            config.marvinShares.imageProcessingQueue.put(request)
        navManager.setTask("notask")


    elif config.task == "takeHeadcamImage":
        '''
        take headcam rgb image and show aruco marker result
        '''
        if neededServersRunning(['imageProcessing']):
            request = {'sender': config.processName, 'cmd': mg.ImageProcessingCommand.CHECK_FOR_ARUCO_CODE, 'cam': mg.HEAD_CAM, 'markers': []}
            config.marvinShares.imageProcessingQueue.put(request)
        navManager.setTask("notask")


    elif config.task == "findObstacles":
        '''
        ask cartControl to return the obstacle line
        '''
        #obstacleDistances = rpcSend.findObstacles(inmoovGlobal.HEAD_CAM)
        # TODO show obstacle line
        config.log(f"TODO obstacleDistances: {obstacleDistances}")
        navManager.setTask("notask")


    elif config.task == "checkForObstacles":

        neededProcesses = ['cartControl']
        if neededServersRunning(['cartControl']):

            #obstacleDist = rpcSend.checkForObstacle()

            #TODO show obstacle line

            navManager.setTask("notask")


    elif config.task == "dock":
        docking.dock()
        navManager.setTask("notask")

#    elif config.task == "restartServers":
#        config.log(f"restartServers requested")
#        for s in config.servers:
#            threadWatchConnections.tryToRestartServer(s)
#        navManager.setTask("notask")

    elif config.task == "stop":
        config.cartCommandMethods.stopCart(config.marvinShares.cartRequestQueue, "manual stop request from gui")
        navManager.setTask("notask")


    elif config.task == "restPosition":
        neededProcesses = ['skeletonControl']
        if all(item in neededProcesses for item in config.marvinShares.processDict.keys()):
            config.skeletonCommandMethods.allServoRest(config.marvinShares.skeletonRequestQueue,config.processName)
            time.sleep(2)

        navManager.setTask("notask")

    elif config.task == "cartMovePose":
        neededProcesses = ['skeletonControl']
        if all(item in neededProcesses for item in config.marvinShares.processDict.keys()):
            #robotHandling.cartMovePose()
            time.sleep(2)

        navManager.setTask("notask")

#    elif config.task == "queryCartInfo":
#        rpcSend.queryCartInfo()
#        navManager.setTask("notask")

#    elif config.task == "requestD415Depth":
#        depth = rpcSend.getImage(inmoovGlobal.HEAD_CAM)
#        if depth is None:
#            config.log(f"could not get depth data")
#        else:
#            config.log(f"depth data received")
#        navManager.setTask("notask")

#    elif config.task == "requestHeadOrientation":
#        yrp = rpcSend.requestHeadOrientation()
#        config.oHead.setHeadOrientation(yrp)
#        config.log(f"headImu values: {yrp}, calibrated yaw: {config.oHead.getHeadYaw()}")
#        navManager.setTask("notask")



    elif config.task == "exit":
        config.log(f"manual exit selected")
        os._exit(4)

    else:
        navManager.setTask("notask")

