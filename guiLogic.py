
import numpy as np
import cv2

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt, pyqtSlot

import config
import gui
import guiUpdate
import navMap
import watchDog
import navManager

class gui(QtWidgets.QMainWindow, gui.Ui_MainWindow):

    def __init__(self, *args, **kwargs):
        QtWidgets.QDialog.__init__(self, *args, **kwargs)

        self.setupUi(self)

        self.threads = []

        self.move(100,100)

        # mark simulated subTasks
        if config.servers['aruco'].simulated:
            self.aruco.setCheckState(Qt.PartiallyChecked)
        if config.servers['cartControl'].simulated:
            self.cartControl.setCheckState(Qt.PartiallyChecked)
        if config.servers['servoControl'].simulated:
            self.servoControl.setCheckState(Qt.PartiallyChecked)
        if config.servers['kinect'].simulated:
            self.kinect.setCheckState(Qt.PartiallyChecked)

        # button group for robot commands
        self.button_group = QtWidgets.QButtonGroup(self)

        for i, b in enumerate(config.tasks):
            button = QtWidgets.QPushButton(self)
            button.setText(b)
            button.setGeometry(500, i*30+20, 150, 25)
            self.button_group.addButton(button)

        self.button_group.buttonClicked.connect(self.on_button_clicked)


        # button group for restarting apps
        self.restart_group = QtWidgets.QButtonGroup(self)
        for i, b in enumerate(config.servers):
            button = QtWidgets.QPushButton(self)
            button.setText("Restart")
            button.setToolTip(b)
            button.setGeometry(130, i*20+70, 60, 18)
            self.restart_group.addButton(button)

        self.restart_group.buttonClicked.connect(self.on_restart_clicked)


        # start thread for checking update queue
        guiUpdateThread = guiUpdate.GuiUpdateThread()
        guiUpdateThread.updateGui.connect(self.updateGui)
        self.threads.append(guiUpdateThread)
        guiUpdateThread.start()


    def on_button_clicked(self, b):
        #print(f"newTask requested: {b.text()}")
        navManager.setTask(b.text())

    def on_restart_clicked(self, b):
        server = b.toolTip()
        config.log(f"server to restart: {server}")
        watchDog.tryToRestartServer(server)

    def on_showCart_stateChanged(self):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showTarget_stateChanged(self):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showScanLocation_stateChanged(self):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showMarkers_stateChanged(self):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showMovePath_stateChanged(self):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})


    def connectionMarkup(self, state):
        checked = False
        style = ""

        if state == "ready":
            style = "color: white; background: green"
            checked = True
        elif state == "connected":
            style = "color: black; background: chartreuse"
        elif state == "try":
            style = "color: black; background: yellow"
        elif state == "down":
            style = "color: white; background: red"
        else:
            config.log(f"unknown connection state {state}")
        return checked, style


    # this is called by guiUpdate ...emit...
    @pyqtSlot(int, object)
    def updateGui(self, updateType, data):

        #print(f"in updateGui, {updateType}, {data}")
        if updateType == guiUpdate.updType.CONNECTION_UPDATE.value:

            oData = config.objectview(data)
            config.log(f"updateGui, CONNECTION_UPDATE, server: {oData.server}, state: {oData.state}")
            checked, style = self.connectionMarkup(oData.state)

            if oData.server == 'taskOrchestrator':
                self.taskOrchestrator.setChecked(checked)
                self.taskOrchestrator.setStyleSheet(style)
            elif oData.server == 'aruco':
                self.aruco.setChecked(checked)
                self.aruco.setStyleSheet(style)
            elif oData.server == 'kinect':
                self.kinect.setChecked(checked)
                self.kinect.setStyleSheet(style)
            elif oData.server == 'servoControl':
                self.servoControl.setChecked(checked)
                self.servoControl.setStyleSheet(style)
            elif oData.server == 'cartControl':
                self.cartControl.setChecked(checked)
                self.cartControl.setStyleSheet(style)
            else:
                config.log(f"guiLogic, CONNECTION_UPDATE received for unknown server {oData.server}")


        if updateType == guiUpdate.updType.MAP.value:

            #config.log(f"start gui map update")
            colImg = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)

            # add cart position and orientation
            if self.showCart.isChecked():
                mapX, mapY = navMap.evalMapLocation(config.oCart.getX(), config.oCart.getY())
                #config.log(f"cart pos: {config.oCart.x}/{config.oCart.y}, map: {mapX}/{mapY}")
                navMap.addCart(colImg)

                # draw hair cross at 0,0
                hairCrossColor = (0,0,255)
                cv2.line(colImg, (495,500), (505, 500), hairCrossColor , 1)
                cv2.line(colImg, (500, 495), (500, 505), hairCrossColor, 1)


            # show target if requested
            if self.showTarget.isChecked():
                targetColor = (0,255,0)
                targetX, targetY = navMap.evalMapLocation(config.oTarget.x, config.oTarget.y)
                cv2.circle(colImg,(targetX,targetY), 3, targetColor, -1)

            # show scanLocations if requested
            if self.showScanLocations.isChecked():
                for scanLocation in config.scanLocations:
                    scanLocX, scanLocY = navMap.evalMapLocation(scanLocation[0], scanLocation[1])
                    cv2.circle(colImg,(scanLocX,scanLocY), 3, config.scanLocationColor, -1)

            # show markers if requested
            if self.showMarkers.isChecked():
                for marker in config.markerList:
                    navMap.addMarker(colImg, marker.markerX, marker.markerY, marker.markerYaw)

            # limit map to show only drawn objects, no black border
            if config.floorPlan is not None:
                mask = config.floorPlan > 0

                coords = np.argwhere(mask)  # Coordinates of non-black pixels.
                if len(coords) > 0:         # do not try to show empty map
                    y0, x0 = coords.min(axis=0)             # Bounding box of non-black pixels.
                    y1, x1 = coords.max(axis=0) + 1   # slices are exclusive at the top

                    cropped = colImg[y0:y1, x0:x1]    # Get a pointer to the bounding box within colImg

                    height, width, channel = cropped.shape
                    bytesPerLine = 3 * width
                    croppedCopy = np.copy(cropped)  # looks like nobody knows why this is necessary but it works (currently)
                    qImg = QtGui.QImage(croppedCopy.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)

                    pix = QtGui.QPixmap(qImg)

                    self.Map.setPixmap(pix)
                    self.Map.setAlignment(Qt.AlignCenter)
                    self.Map.setScaledContents(True)
                    self.Map.setMinimumSize(1,1)
                    self.Map.show()
                    #config.log(f"end gui map update")


        if updateType == guiUpdate.updType.CART_INFO.value:

            self.cartOrientation.setText(f"{config.oCart.getYaw()}")
            self.cartLocation.setText(f"{config.oCart.getX():4.0f}, {config.oCart.getY():4.0f}")
            mapX, mapY = navMap.evalMapLocation(config.oCart.getX(), config.oCart.getY())
            self.mapLocation.setText(f"{mapX:4.0f}, {mapY:4.0f}")


        if updateType == guiUpdate.updType.BATTERY_UPDATE.value:
            if config.batteryStatus['plugged']:
                msg = f"plugged, {config.batteryStatus['percent']}%"
            else:
                msg = f"battery, {config.batteryStatus['percent']}%"

            self.laptop.setText(msg)
            self.v12.setText(str(config.batteryStatus['v12']))
            self.v6.setText(str(config.batteryStatus['v6']))

