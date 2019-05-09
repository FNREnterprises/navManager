
import time
import cv2
import os

import config
import rpcSend
import navMap
import robotControl
import navManager
import marker
import docking
import watchDog
import cartHandling


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
            watchDog.tryToRestartServer(server)
        navManager.setTask("notask")

    elif config.task == "createFloorPlan":
        neededTasks = ['kinect','aruco','cartControl','servoControl']
        if rpcSend.neededServersRunning(neededTasks):

            if navMap.createFloorPlan():
                navManager.setTask("fullScanAtPosition")
            else:
                config.log(f"could not create floor plan")
                navManager.setTask("notask")

    elif config.task == "fullScanAtPosition":

        neededTasks = ['kinect','aruco','cartControl','servoControl']
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
        camDegrees = config.oCart.getDegrees() + config.servoCurrent['head.rothead']['degrees']
        '''
        if not config.netsLoaded:
            analyzeImage.init()

        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getEyecamImage()
            if img is not None:

                frameFace, bboxes = analyzeImage.getFaceBox(img)
                if not bboxes:
                    print("No face detected")
                    navManager.setTask("notask")
                else:
                    cv2.imshow("inmoovEyeCamImg", img)
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
        neededTasks = ['aruco']
        cartDegrees = config.oCart.getDegrees()
        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getCartcamImage()
            markersFound, result = rpcSend.lookForMarkers("CART_CAM", [])
            config.log(f"lookForMarkers result: {markersFound},{result}")
            if img is not None:
                if markersFound:
                    #config.log(f"markers found: {result}")
                    marker.updateMarkerFoundResult(result, 'CART_CAM', config.oCart.getX(), config.oCart.getY(), cartDegrees, 0)

                cv2.imshow("inmoovCartcamImg", img)
                cv2.waitKey(3000)
                cv2.destroyAllWindows()
            else:
                config.log(f"could not take a cartcam image")

            navManager.setTask("notask")
        else:
            config.log(f"aruco task not ready yet")


    elif config.task == "takeEyecamImage":
        '''
        take eyecam image and show aruco marker result
        '''
        neededTasks = ['aruco']
        cartDegrees = config.oCart.getDegrees()
        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getEyecamImage()
            markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
            config.log(f"lookForMarkers result: {markersFound},{result}")
            if img is not None:
                if markersFound:
                    #config.log(f"markers found: {result}")
                    marker.updateMarkerFoundResult(result, 'EYE_CAM', config.oCart.getX(), config.oCart.getY(), cartDegrees, 0)

                cv2.imshow("inmoovEyecamImg", img)
                cv2.waitKey(3000)
                cv2.destroyAllWindows()
            else:
                config.log(f"could not take an eyecam image")

            navManager.setTask("notask")
        else:
            config.log(f"aruco task not ready yet")

    elif config.task == "getDepthImage":
        '''
        simply try to access the kinect and show the obstacle map
       '''
        neededTasks = ['kinect']
        if rpcSend.neededServersRunning(neededTasks):

            obstacles = rpcSend.getDepth(90)
            if obstacles is not None:
                config.log(f"obstacles: {len(obstacles)}")
                navMap.createObstacleMap(obstacles, (config.oCart.getX(), config.oCart.getY()), config.oCart.getDegrees(), show=True)
            else:
                config.log(f"could not acquire a depth image / obstacles")

            navManager.setTask("notask")

    elif config.task == "checkForObstacles":

        neededTasks = ['kinect']
        if rpcSend.neededServersRunning(neededTasks):

            rpcSend.checkForObstacle()

            navManager.setTask("notask")


    elif config.task == "approachTarget":
        robotControl.approachTarget(config.oTarget)


    elif config.task == "dock":
        docking.dock()
        navManager.setTask("notask")

    elif config.task == "restartServers":
        config.log(f"restartServers requested")
        rpcSend.restartServers()
        navManager.setTask("notask")

    elif config.task == "stop":
        robotControl.stopRobot("manual stop request from gui")
        navManager.setTask("notask")


    elif config.task == "restPosition":
        neededTasks = ['servoControl']
        if rpcSend.neededServersRunning(neededTasks):

            robotControl.servoRestAll()
            time.sleep(2)

        navManager.setTask("notask")

    elif config.task == "cartMovePose":
        neededTasks = ['servoControl']
        if rpcSend.neededServersRunning(neededTasks):

            robotControl.cartMovePose()
            time.sleep(2)

        navManager.setTask("notask")


    elif config.task == "moveCart":
        #robotControl.moveCartWithDirection(config.Direction.FORWARD, 200, 200)
        # Apr 19, new cart move handling
        # backward 200
        cartHandling.moveCartWithDirection(config.Direction.FORWARD, 200, 200, kinectMonitoring=True)
        cartHandling.moveCartWithDirection(config.Direction.BACKWARD, 200, 200, kinectMonitoring=True)
        cartHandling.moveCartWithDirection(config.Direction.LEFT, 200, 200, kinectMonitoring=True)
        cartHandling.moveCartWithDirection(config.Direction.RIGHT, 200, 200, kinectMonitoring=True)
        cartHandling.rotateCartRelative(30)
        cartHandling.rotateCartRelative(-30)
        cartHandling.rotateCartAbsolute(60)
        cartHandling.rotateCartAbsolute(-60)
        cartHandling.moveCart(40,200, 200)
        cartHandling.moveCart(-40,200, 200)

        navManager.setTask("notask")

    elif config.task == "queryCartInfo":
        rpcSend.queryCartInfo()
        navManager.setTask("notask")

    elif config.task == "activateKinectPower":
        rpcSend.powerKinect(True)
        navManager.setTask("notask")

    elif config.task == "rotateCartAbsolute":
        cartHandling.rotateCartAbsolute(config.oCart.getDegrees() + 30)
        navManager.setTask("notask")

    elif config.task == "exit":
        config.log(f"manual exit selected")
        os._exit(4)

    else:
        navManager.setTask("notask")

