''' Position InMoov robot in front of an Aruco Marker
'''

import os
import time
import logging
import cv2
import threading
#import xmlrpc.client as xmlrpc
#import requests
import rpyc

import config
import marker
import robotControl

import rpcReceive
import rpcSend
import docking
import navMap


taskOrchestrator = None

def tryRotationContinuation(obstacleInfo):

    if any(oI['direction'] + '_' + oI['position'] == 'forward_left' for oI in obstacleInfo):
    #if any(oI['direction'] == 'forward' for oI in obstacleInfo):
        config.log('blocking front left')
        config.navManagerServers['cartControl']['conn'].exposed_move(config.DIAGONAL_RIGHT_BACKWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'forward_right' for oI in obstacleInfo):
        config.log('blocking front right')
        config.navManagerServers['cartControl']['conn'].exposed_move(config.DIAGONAL_LEFT_BACKWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'backward_left' for oI in obstacleInfo):
        config.log('blocking back left')
        config.navManagerServers['cartControl']['conn'].exposed_move(config.DIAGONAL_RIGHT_FORWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'backward_right' for oI in obstacleInfo):
        config.log('blocking back right')
        config.navManagerServers['cartControl']['conn'].exposed_move(config.DIAGONAL_LEFT_FORWARD, 150, 10)

'''

if markerFound != None:

    # rotate cart towards marker
    # relativeRotation in full degrees
    relativeRotation = int(navGlobal.markerInfo.yawToCartTarget - 90)
    navGlobal.log(f"relative cart rotation: {relativeRotation}")
        
    # rotate cart to point at marker
    robotControl.cartRequest.root.rotateRelative(bytes(str(relativeRotation), 'ascii'))

    
    imgPath = '//MARVIN/Aruco/aruco' + str(headYaw) + '.jpg'
    img = cv2.imread(imgPath, cv2.IMREAD_GRAYSCALE)

    imgWindow = "headCamera " + str(headYaw) + " degree"
    cv2.imshow(imgWindow, img)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()


    # turn head to look at marker (90 is cart target position
    newHeadYaw = 90 - navGlobal.markerInfo.yawToCartTarget + navGlobal.markerInfo.yawToMarker
    navGlobal.log(f"look at marker after cart rotation: {newHeadYaw}, {navGlobal.markerInfo.yawToCartTarget}, {navGlobal.markerInfo.yawToMarker}")
    robotControl.sendMrlCommand("i01.head.rothead", "moveTo", str(newHeadYaw))
    
    # use kinect depth data for distance?
    # move toward cartTarget
    # T O D O common definition of direction, speed
    # def exposed_move(self, direction, speed, distance=10):
    distance = 30 if navGlobal.markerInfo.distToCartTarget > 30 else navGlobal.markerInfo.distToCartTarget
    navGlobal.log(f"move to cartTarget, distance: {distance}")
    robotControl.cartRequest.root.move(navGlobal.FORWARD, 200, distance) #bytes('1' + str(relativeRotation), 'ascii'))

    # assuming we are approaching the cart target position
    while True:

        navGlobal.log("wait for cart movement to be finished")
        time.sleep(5)
        # verify marker
        markerFound = marker.arucoRequest.root.findMarker(navGlobal.currentHeadYaw)
        if markerFound != None:

            logging.info("marker still found")
            marker.updateMarkerFoundResult(markerFound)
            if navGlobal.markerInfo.distToCartTarget < 5:
                robotControl.cartRequest.root.stop()

                # rotate toward marker
                relativeRotation = navGlobal.markerInfo.yawToMarker - 90
                newHeadYaw = 90
                navGlobal.log(f"turning cart toward marker, relative Rotation: {relativeRotation}")
                robotControl.cartRequest.root.rotateRelative(relativeRotation)
                robotControl.sendMrlCommand("i01.head.rothead", "moveTo", str(newHeadYaw))
                navGlobal.log("success ???")

            else:
                # continue move
                distance = 30 if navGlobal.markerInfo.distToCartTarget > 30 else navGlobal.markerInfo.distToCartTarget
                navGlobal.log(f"move to cartTarget, distance: {distance}")
                robotControl.cartRequest.root.move(navGlobal.FORWARD, 200, distance) #bytes('1' + str(relativeRotation), 'ascii'))

        else:
            # check with small head adjustment again
            startYaw = robotControl.queryMrlService("i01.head.rothead","getCurrentPos")
            if startYaw < 90:
                newHeadYaw = startYaw - navGlobal.HEAD_YAW_INCREMENT
            else:
                newHeadYaw = startYaw + navGlobal.HEAD_YAW_INCREMENT
    
            robotControl.sendMrlCommand("i01.head.rothead", "moveTo", str(newHeadYaw))
            markerFound = marker.arucoRequest.root.findMarker(navGlobal.currentHeadYaw)

            navGlobal.log("on my way to cartTarget marker lost")
            raise SystemExit()
'''

def setup():

    global taskOrchestrator

    # set initial task (or "notask")
    config.setTask("notask")

    # optional: set simulation status of subtasks
    rpcSend.setSimulationMask(kinect=False, aruco=False, cartControl=False, servoControl=False)
    #config.setSimulationMask(kinect=True, aruco=True, cartControl=True, servoControl=True)

    testMap = False
    if testMap:
        navMap._obstacles = cv2.imread(f"{config.PATH_ROOM_DATA}/unknown/depthImages/000.jpg")

    #navGlobal.setDockingMarkerPosition()
    #navGlobal.setTask("dock")
    #navGlobal.setTask("dockingPhase2")

    # wait for log listener to start up
    time.sleep(1)

    rpcSend.startServers()

    #robotControl.stopRobot("stop robot on initializing navManager")

    '''
    navGlobal.cartLocation.x=51
    navGlobal.cartLocation.y=101
    navGlobal.cartOrientation=180
    navGlobal._fullScanDone = False
    navGlobal.saveCartLocation()
    '''
    config.loadMapInfo()

    # try to load existing floor plan
    if config._fullScanDone and navMap.loadFloorPlan(config._room):
        navMap.loadScanLocations()
        marker.loadMarkerInfo()
    else:
        config.setTask('createFloorPlan')

    findNextScanLocationTest = False
    if findNextScanLocationTest:
        config.setCartOrientation(200)
        navMap.findNewScanLocation()

    checkMove = False
    if checkMove:
        mapDegrees = 180
        speed = 200
        distanceMm = 200
        robotControl.moveCart(mapDegrees, distanceMm, speed)
        # wait for movement done
        while config.isCartMoving():
             time.sleep(0.2)
        raise SystemExit(0)

    # start connection watcher thread
    mapThread = threading.Thread(target=rpcSend.watchServers, args={})
    config.log("start heartbeatRequester")
    mapThread.start()

    loop()



def loop():

    # check for assigned task
    while True:
        if config.getTask() == "restartTasks":
            rpcSend.startServers()
            config.setTask("notask")

        elif config.getTask() == "createFloorPlan":
            neededTasks = ['kinect','aruco','cartControl','servoControl']
            if rpcSend.neededTasksRunning(neededTasks):

                if navMap.createFloorPlan():
                    config.setTask("rover")
                else:
                    config.setTask("notask")

        elif config.getTask() == "showTime":
            robotControl.showTime()
            config.setTask("notask")

        elif config.getTask() == "showEyecamImage":
            '''
            simply try to access the inmoov eye cam and show the image
            def from_string(s): 
            f = StringIO(s) 
            arr = format.read_array(f) 
            return arr 
            '''
            neededTasks = ['aruco']
            if rpcSend.neededTasksRunning(neededTasks):
                img = rpcSend.getEyecamImage()
                if img is not None:
                    cv2.imshow("inmoovEyeCamImg", img)
                    cv2.waitKey()
                    cv2.destroyAllWindows()
                else:
                    config.log(f"could not take an eyecam image")

                config.setTask("notask")


        elif config.getTask() == "showCartcamImage":
            '''
            simply try to access the cart cam and show the image
           '''
            neededTasks = ['aruco']
            if rpcSend.neededTasksRunning(neededTasks):
                img = rpcSend.getCartcamImage()
                if img is not None:
                    cv2.imshow("inmoovCartcamImg", img)
                    cv2.waitKey()
                    cv2.destroyAllWindows()
                else:
                    config.log(f"could not take a cartcam image")

                config.setTask("notask")


        elif config.getTask() == "showDepthImage":
            '''
            simply try to access the kinect and show the image
           '''
            neededTasks = ['kinect']
            if rpcSend.neededTasksRunning(neededTasks):

                img = rpcSend.getDepth(0)
                if img is not None:
                    cv2.imshow("inmoovCartcamImg", img)
                    cv2.waitKey()
                    cv2.destroyAllWindows()
                else:
                    config.log(f"could not get a depth image")

                config.setTask("notask")

        elif config.getTask() == "findMarker":
            '''
            starts a search for markers in the room with the inmoov eye cam
            #try:
            markerFound = marker.fullScanForMarker()
            #except Exception as e:
            #    navGlobal.log("exception during fullScanForMarker()")
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
                    navGlobal.log("cart blocked")
                    navGlobal.setTask("notask")
                else:
                    navGlobal.setTask("rover")
            '''
        elif config.getTask() == "approachTarget":
            robotControl.approachTarget(config.target)


        elif config.getTask() == "rover":
            navMap.rover()

        elif config.getTask() == "dock":
            docking.dock()
            #/navGlobal.setTask("in docking sequence")

        elif config.getTask() == "dockingPhase2":
            config.log(f"call robotControl.dockingPhase2")
            docking.dockingPhase2()

        elif config.getTask() == "dockingPhase3":
            docking.dockingPhase3()

        elif config.getTask() == "restartTasks":
            config.log(f"restartTasks requested")
            rpcSend.startServers()
            config.setTask("notask")

        elif config.getTask() == "stop":
            robotControl.stopRobot()
            config.setTask("notask")

        elif config.getTask() == "exit":
            config.log("manual exit selected")
            raise SystemExit(0)

        elif config.getTask() == "notask":
            print()
            print("possible tasks:")
            for t in config.tasks:
                print(f"  {t}")
            choice = input(f"current task: {config.getTask()}; enter new task: ")
            config.setTask(choice)
            config.log(f"manually selected task {choice}")

        else:
            config.setTask("notask")

        time.sleep(1)




if __name__ == "__main__":

    windowName = "pcjm//navManager"
    os.system("title " + windowName)
    #hwnd = win32gui.FindWindow(None, windowName)
    #win32gui.MoveWindow(hwnd, 2000,0,1200,1200,True)

    ##########################################################
    # initialization
    # Logging
    logging.basicConfig(
        filename="../navManager.log",
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(message)s',
        filemode="w")

    # start the navigation thread (setup and loop)
    navThread = threading.Thread(target=setup, args={})
    navThread.setName("navThread")
    navThread.start()

    # start map update thread navMap.updateFloorPlan
    mapThread = threading.Thread(target=navMap.updateFloorPlan, args={})
    config.log("start map updater")
    mapThread.start()

#    from rpyc.utils.server import ThreadedServer
#    server = ThreadedServer(rpcReceive.navManagerListener, port=20010)
#    server.start()
    # start xmlrpc receiver
    #rpcReceive.xmlrpcListener()
