''' Position InMoov robot in front of an Aruco Marker
'''

import os
import sys
import time
import logging
import rpyc
import threading
from PyQt5 import QtWidgets

import config
import marker
import robotControl

import rpcReceive
import rpcSend
import navTasks
import navMap
import guiLogic
import guiUpdate
import watchDog

#taskOrchestrator = None

def tryRotationContinuation(obstacleInfo):

    if any(oI['direction'] + '_' + oI['position'] == 'forward_left' for oI in obstacleInfo):
    #if any(oI['direction'] == 'forward' for oI in obstacleInfo):
        config.log('blocking front left')
        config.navManagerServers['cartControl']['conn'].root.exposed_move(config.DIAGONAL_RIGHT_BACKWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'forward_right' for oI in obstacleInfo):
        config.log('blocking front right')
        config.navManagerServers['cartControl']['conn'].root.exposed_move(config.DIAGONAL_LEFT_BACKWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'backward_left' for oI in obstacleInfo):
        config.log('blocking back left')
        config.navManagerServers['cartControl']['conn'].root.exposed_move(config.DIAGONAL_RIGHT_FORWARD, 150, 10)

    if any(oI['direction'] + '_' + oI['position'] == 'backward_right' for oI in obstacleInfo):
        config.log('blocking back right')
        config.navManagerServers['cartControl']['conn'].root.exposed_move(config.DIAGONAL_LEFT_FORWARD, 150, 10)

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

        navGlobal.log(f"wait for cart movement to be finished")
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
                navGlobal.log(f"success ???")

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

            navGlobal.log(f"on my way to cartTarget marker lost")
            raise SystemExit()
'''

class cCart:

    __x = 0
    __y = 0
    __o = 0
    xCorr = 0
    yCorr = 0
    oCorr = 0
    moving = False
    rotating = False
    blocked = False
    docked = False
    updateTime = time.time()

    def setX(self, x):
        self.__x = x

    def getX(self):
        return self.__x + self.xCorr

    def setY(self, y):
        self.__y = y

    def getY(self):
        return self.__y + self.yCorr

    def setO(self, y):
        self.oy = y

    def getO(self):
        return self.__o + self.oCorr

#cartInfo = {'x': 0, 'y': 0, 'orientation': 0, 'moving:': False, 'rotating': False, 'blocked': False, 'docked': False, 'updateTime': time.time()}
oCart = cCart()

# navManager local cart position correction, gets set by fullScanAtPosition and reset after sending it to the cart
cartPositionCorrection = {'x': 0, 'y': 0, 'o': 0}


def setup():

    config.createObjectViews()

    # set initial task (or "notask")
    config.setTask("notask")

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
    config.log(f"connect successful, try to getLifeSignal")
    good = False
    try:
        good = config.taskOrchestrator.root.exposed_getLifeSignal(config.MY_IP, config.MY_PORT)
    except Exception as e:
        config.log(f"failed to get response, taskOrchestrator on {subSystemIp} not running?")
        os._exit(2)

    if good:
        config.log(f"life signal from task orchestrator received")
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': 'taskOrchestrator', 'state': 'up'})
    else:
        config.log(f"could not get life signal from task orchestrator on subsystem {subSystemIp}")
        os._exit(3)

    for server in config.navManagerServers:
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
        config.setTask('createFloorPlan')

    if config.floorPlan is not None:
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO.value})

#    config.setTask('rebuildMap')

    # check for assigned task
    while True:
        navTasks.checkForTasks()
        time.sleep(0.1)


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
    baseName = "../navManager"
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
        os.rename(oldName, newName)

    logging.basicConfig(
        filename="../navManager.log",
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

