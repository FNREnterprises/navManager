
import time
import cv2
import os

import config
import navMap

import navManager
import docking
#import threadWatchConnections
from marvinglobal import marvinglobal as mg
import fullScanAtPosition

def neededProcessesRunning(processes) -> bool:
    for process in processes:
        if not process in config.marvinShares.processDict.keys():
            config.log(f"task {config.task} requires process {process} but it is not running, request ignored")
            navManager.setTask("needed processes not running", "notask")
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
    else:
        return

    if config.task == "restartServers":
        #for server in config.servers:
        #    threadWatchConnections.tryToRestartServer(server)
        navManager.setTask("task restartServers", "notask")

    elif config.task == "createFloorPlan":
        neededProcesses = ['cartControl','skeletonControl']
        if neededProcessesRunning(neededProcesses):

            if navMap.createFloorPlan():
                navManager.setTask("task createFloorPlan", "fullScanAtPosition")
            else:
                config.log(f"could not create floor plan")
                navManager.setTask("task createFloorPlan", "notask")

    elif config.task == "fullScanAtPosition":

        neededProcesses = ['cartControl', 'skeletonControl', 'imageProcessing']
        if neededProcessesRunning(neededProcesses):

            if fullScanAtPosition.fullScanAtPosition():
                navManager.setTask("task fullScanAtPosition", "rover")
        else:
            config.log(f"fullScanAtPosition needs {neededProcesses}")
            navManager.setTask("task fullScanAtPosition", "notask")

    elif config.task == "rover":
        navMap.rover()

    elif config.task == "rebuildMap":
        navMap.rebuildMap()
        navManager.setTask("task rebuildMap", "notask")

    elif config.task == "checkForPerson":
        '''
        get an eyecam image and try to find people in it
        '''
        neededProcesses = ['aruco']
        camDegrees = config.cart.getCartYaw() + config.servoCurrent['head.rothead']['degrees']
        '''
        if not config.netsLoaded:
            analyzeImage.init()

        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getImage(inmoovGlobal.EYE_CAM)
            if img is not None:

                frameFace, bboxes = analyzeImage.getFaceBox(img)
                if not bboxes:
                    print("No face detected")
                    navManager.setTask("task checkForPerson", "notask")
                else:
                    cv2.imshow("inmoovEyeCamImage", img)
                    cv2.waitKey()
                    cv2.destroyAllWindows()

            else:
                config.log(f"could not take an eyecam image")
        '''
        navManager.setTask(f"task {config.task}", "notask")


    elif config.task == "takeCartcamImage":
        '''
        take cart cam image and show aruco marker result
        '''
        navMap.takeCartcamImage()
        navManager.setTask(f"task {config.task}", "notask")


    elif config.task == "takeEyecamImage":
        '''
        take eyecam image and show aruco marker result
        '''
        navMap.takeEyecamImage()
        navManager.setTask("task takeEyecamImage", "notask")


    elif config.task == "takeHeadcamImage":
        '''
        take headcam rgb image and show aruco marker result
        '''
        navMap.takeHeadcamImage()
        navManager.setTask("task takeHeadcamImage", "notask")


    elif config.task == "takeDepthcamImage":
        '''
        get depth image from cartControl
        '''
        navMap.takeDepthcamImage()
        navManager.setTask(f"task {config.task}", "notask")


    elif config.task == "checkForObstacles":

        neededProcesses = ['cartControl']
        if neededProcessesRunning(neededProcesses):

            #obstacleDist = rpcSend.checkForObstacle()

            #TODO show obstacle line

            navManager.setTask(f"task {config.task}", "notask")


    elif config.task == "dock":
        docking.dock()
        navManager.setTask(f"task {config.task}", "notask")


    elif config.task == "stop":
        # stop cart
        if "cartControl" in config.marvinShares.processDict.keys():
            msg = {'msgType': mg.CartCommands.STOP_CART, 'sender': config.processName,
                   'info': "robot stop request from navManager"}
            config.marvinShares.cartRequestQueue.put(msg)
        # stop servos
        if "skeltonControl" in config.marvinShares.processDict.keys():
            msg = {'msgType': "allServoStop", 'sender': config.processName,
                   'info': "robot stop request from navManager"}
            config.marvinShares.skeletonRequestQueue.put(msg)
        navManager.setTask(f"task {config.task}", "notask")

    elif config.task == "restPosition":
        neededProcesses = ['robotControl']
        if rpcSend.neededServersRunning(neededProcesses):

            robotHandling.servoRestAll()
            time.sleep(2)

        navManager.setTask(f"task {config.task}", "notask")

    elif config.task == "cartMovePose":
        neededProcesses = ['robotControl']
        if rpcSend.neededServersRunning(neededProcesses):

            robotHandling.cartMovePose()
            time.sleep(2)

        navManager.setTask(f"task {config.task}", "notask")

    elif config.task == "queryCartInfo":
        rpcSend.queryCartInfo()
        navManager.setTask(f"task {config.task}", "notask")

    elif config.task == "requestD415Depth":
        depth = rpcSend.getImage(inmoovGlobal.HEAD_DEPTH)
        if depth is None:
            config.log(f"could not get depth data")
        else:
            config.log(f"depth data received")
        navManager.setTask(f"task {config.task}", "notask")

    elif config.task == "requestHeadOrientation":
        yrp = rpcSend.requestHeadOrientation()
        config.head.setHeadOrientation(yrp)
        config.log(f"headImu values: {yrp}, calibrated yaw: {config.head.getHeadYaw()}")
        navManager.setTask(f"task {config.task}", "notask")



    elif config.task == "exit":
        config.log(f"manual exit selected")
        os._exit(4)

    else:
        navManager.setTask(f"unhandled task {config.task}", "notask")

