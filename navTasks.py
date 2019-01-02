
import os
import cv2

import config
import rpcSend
import navMap
import robotControl
import docking

def checkForTasks():

    if config.task == "restartServers":
        rpcSend.restartServers()
        config.setTask("notask")

    elif config.task == "createFloorPlan":
        neededTasks = ['kinect','aruco','cartControl','servoControl']
        if rpcSend.neededServersRunning(neededTasks):

            if navMap.createFloorPlan():
                config.setTask("fullScanAtPosition")
            else:
                config.log(f"could not create floor plan")
                config.setTask("notask")

    elif config.task == "fullScanAtPosition":

        neededTasks = ['kinect','aruco','cartControl','servoControl']
        if rpcSend.neededServersRunning(neededTasks):

            if navMap.fullScanAtPosition():
                config.setTask("rover")
            else:
                config.log(f"could not do a full scan")
                config.setTask("notask")

    elif config.task == "rover":
        navMap.rover()

    elif config.task == "rebuildMap":
        navMap.rebuildMap()
        config.setTask("notask")

    elif config.task == "showTime":
        robotControl.showTime()
        config.setTask("notask")

    elif config.task == "showEyecamImage":
        '''
        simply try to access the inmoov eye cam and show the image
        def from_string(s): 
        f = StringIO(s) 
        arr = format.read_array(f) 
        return arr 
        '''
        neededTasks = ['aruco']
        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getEyecamImage()
            markersFound, result = rpcSend.lookForMarkers("EYE_CAM", [])
            config.log(f"lookForMarkers result: {markersFound},{result}")
            if img is not None:
                cv2.imshow("inmoovEyeCamImg", img)
                cv2.waitKey()
                cv2.destroyAllWindows()

            else:
                config.log(f"could not take an eyecam image")

            config.setTask("notask")


    elif config.task == "showCartcamImage":
        '''
        simply try to access the cart cam and show the image
       '''
        neededTasks = ['aruco']
        if rpcSend.neededServersRunning(neededTasks):
            img = rpcSend.getCartcamImage()
            markersFound, result = rpcSend.lookForMarkers("CART_CAM", [])
            config.log(f"lookForMarkers result: {markersFound},{result}")
            if img is not None:
                cv2.imshow("inmoovCartcamImg", img)
                cv2.waitKey()
                cv2.destroyAllWindows()
            else:
                config.log(f"could not take a cartcam image")

            config.setTask("notask")


    elif config.task == "getDepthImage":
        '''
        simply try to access the kinect and show the obstacle map
       '''
        neededTasks = ['kinect']
        if rpcSend.neededServersRunning(neededTasks):

            obstacles = rpcSend.getDepth(0)
            if obstacles is not None:
                config.log(f"obstacles: {len(obstacles)}")
                navMap.createObstacleMap(obstacles, (config.oCart.x, config.oCart.y), config.oCart.orientation,  show=True)
            else:
                config.log(f"could not acquire a depth image / obstacles")

            config.setTask("notask")

    elif config.task == "findMarker":
        '''
        starts a search for markers in the room with the inmoov eye cam
        #try:
        markerFound = marker.fullScanForMarker()
        #except Exception as e:
        #    navGlobal.log(f"exception during fullScanForMarker()")
        #    print('Error on line {}'.format(sys.exc_info()[-1].tb_lineno), type(e).__name__, e)
        #    markerFound = False
        #    navGlobal.setTask("stop")
    
        navGlobal.log(f"fullScanForMarker done, markerFound: {markerFound}")
        robotControl.sendMrlCommand("i01.head.neck", "enableAutoDisable", "true")
        robotControl.sendMrlCommand("i01.head.rothead", "enableAutoDisable", "true")
    
        if markerFound:
            navGlobal.setTask("approachTarget")
        else:
            if navGlobal.getCartBlocked():
                navGlobal.log(f"cart blocked")
                navGlobal.setTask("notask")
            else:
                navGlobal.setTask("rover")
        '''
    elif config.task == "approachTarget":
        robotControl.approachTarget(config.oTarget)


    elif config.task == "dock":
        docking.dock()
        #/navGlobal.setTask("in docking sequence")

    elif config.task == "dockingPhase2":
        config.log(f"call robotControl.dockingPhase2")
        docking.dockingPhase2()

    elif config.task == "dockingPhase3":
        docking.dockingPhase3()

    elif config.task == "restartServers":
        config.log(f"restartServers requested")
        rpcSend.startServers()
        config.setTask("notask")

    elif config.task == "stop":
        robotControl.stopRobot()
        config.setTask("notask")

    elif config.task == "moveForward50":
        robotControl.moveCartWithDirection(robotControl.FORWARD, 500, 200)
        config.setTask("notask")

    elif config.task == "moveBackward50":
        robotControl.moveCartWithDirection(robotControl.BACKWARD, 500, 200)
        config.setTask("notask")

    elif config.task == "moveHead":
        robotControl.servoMoveToDegreesBlocking('head.rothead', 30, 250)
        robotControl.servoMoveToDegreesBlocking('head.rothead', -30, 250)
        config.setTask("notask")

    elif config.task == "setPin41":
        rpcSend.pinHigh([41])
        config.setTask("notask")

    elif config.task == "clearPin41":
        rpcSend.pinLow([41])
        config.setTask("notask")

    elif config.task == "exit":
        config.log(f"manual exit selected")
        os._exit(4)

    else:
        config.setTask("notask")

