
import time
import cv2
import os

import inmoovGlobal
import config
import rpcSend
import navMap
import robotHandling
import navManager
import docking
#import threadWatchConnections



def checkForTasks():
    """
    this function gets called every 100 ms
    :return:
    """

    if config.task != config.prevTask:
        config.prevTask = config.task
        config.log(f"checkForTasks sees new task: {config.task}")

    if config.task == "restartServers":
        for server in config.servers:
            threadWatchConnections.tryToRestartServer(server)
        navManager.setTask("notask")

    elif config.task == "createFloorPlan":
        neededTasks = ['cartControl','robotControl']
        if rpcSend.neededServersRunning(neededTasks):

            if navMap.createFloorPlan():
                navManager.setTask("fullScanAtPosition")
            else:
                config.log(f"could not create floor plan")
                navManager.setTask("notask")

    elif config.task == "fullScanAtPosition":

        neededTasks = ['cartControl','robotControl']
        if rpcSend.neededServersRunning(neededTasks):

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
        neededTasks = ['aruco']
        camDegrees = config.oCart.getCartYaw() + config.servoCurrent['head.rothead']['degrees']
        '''
        if not config.netsLoaded:
            analyzeImage.init()

        if rpcSend.neededServersRunning(neededTasks):
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
        navMap.takeCartcamImage()
        navManager.setTask("notask")


    elif config.task == "takeEyecamImage":
        '''
        take eyecam image and show aruco marker result
        '''
        navMap.takeEyecamImage()
        navManager.setTask("notask")


    elif config.task == "takeHeadcamImage":
        '''
        take headcam rgb image and show aruco marker result
        '''
        navMap.takeHeadcamImage()
        navManager.setTask("notask")


    elif config.task == "takeDepthcamImage":
        '''
        get depth image from cartControl
        '''
        navMap.takeDepthcamImage()
        navManager.setTask("notask")


    elif config.task == "checkForObstacles":

        neededTasks = ['cartControl']
        if rpcSend.neededServersRunning(neededTasks):

            obstacleDist = rpcSend.checkForObstacle()

            #TODO show obstacle line

            navManager.setTask("notask")


    elif config.task == "dock":
        docking.dock()
        navManager.setTask("notask")

    elif config.task == "restartServers":
        config.log(f"restartServers requested")
        for s in config.servers:
            threadWatchConnections.tryToRestartServer(s)
        navManager.setTask("notask")

    elif config.task == "stop":
        robotHandling.stopRobot("manual stop request from gui")
        navManager.setTask("notask")


    elif config.task == "restPosition":
        neededTasks = ['robotControl']
        if rpcSend.neededServersRunning(neededTasks):

            robotHandling.servoRestAll()
            time.sleep(2)

        navManager.setTask("notask")

    elif config.task == "cartMovePose":
        neededTasks = ['robotControl']
        if rpcSend.neededServersRunning(neededTasks):

            robotHandling.cartMovePose()
            time.sleep(2)

        navManager.setTask("notask")

    elif config.task == "queryCartInfo":
        rpcSend.queryCartInfo()
        navManager.setTask("notask")

    elif config.task == "requestD415Depth":
        depth = rpcSend.getImage(inmoovGlobal.HEAD_DEPTH)
        if depth is None:
            config.log(f"could not get depth data")
        else:
            config.log(f"depth data received")
        navManager.setTask("notask")

    elif config.task == "requestHeadOrientation":
        yrp = rpcSend.requestHeadOrientation()
        config.oHead.setHeadOrientation(yrp)
        config.log(f"headImu values: {yrp}, calibrated yaw: {config.oHead.getHeadYaw()}")
        navManager.setTask("notask")



    elif config.task == "exit":
        config.log(f"manual exit selected")
        os._exit(4)

    else:
        navManager.setTask("notask")

