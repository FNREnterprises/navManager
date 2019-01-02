
import time

import numpy as np
import cv2

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt, pyqtSlot

import config
import gui
import guiUpdate
import navMap


class gui(QtWidgets.QMainWindow, gui.Ui_MainWindow):

    def __init__(self, *args, **kwargs):
        QtWidgets.QDialog.__init__(self, *args, **kwargs)

        self.setupUi(self)

        self.threads = []

        # mark simulated subTasks
        if config.navManagerServers['aruco']['simulated']:
            self.aruco.setCheckState(Qt.PartiallyChecked)
        if config.navManagerServers['cartControl']['simulated']:
            self.cartControl.setCheckState(Qt.PartiallyChecked)
        if config.navManagerServers['servoControl']['simulated']:
            self.servoControl.setCheckState(Qt.PartiallyChecked)
        if config.navManagerServers['kinect']['simulated']:
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
        for i, b in enumerate(config.navManagerServers):
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
        config.setTask(b.text())

    def on_restart_clicked(self, b):
        server = b.toolTip()
        print(f"server to restart: {server}")
        config.navManagerServers[server]['conn'] = None
        config.navManagerServers[server]['startRequested'] = time.time()
        config.taskOrchestrator.root.restartServer(server)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})

    def on_showCart_stateChanged(selfself):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showTarget_stateChanged(selfself):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showScanLocation_stateChanged(selfself):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showMarkers_stateChanged(selfself):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def on_showMovePath_stateChanged(selfself):
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})

    def connectionMarkup(self, state):
        checked = True
        style = ""
        if state == "up":
            style = "color: white; background: green"
        elif state == "down":
            style = "color: white; background: red"
            checked = False
        elif state == "try":
            style = "color: black; background: yellow"
        elif state == "wait":
            style = "color: black; background: chartreuse"
        else:
            config.log(f"unknown connection state {state}")
        return checked, style

    # this is called by guiUpdate ...emit...
    @pyqtSlot(int, object)
    def updateGui(self, type, data):

        #print(f"in updateGui, {type}, {data}")
        if type == guiUpdate.updType.CONNECTION_UPDATE.value:

            oData = config.objectview(data)
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


        if type == guiUpdate.updType.MAP.value:

            #config.log(f"start gui map update")
            colImg = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)

            # add cart position and orientation
            if self.showCart.isChecked():
                mapX, mapY = navMap.evalMapLocation(config.oCart.x, config.oCart.y)
                cartColor = (0,255,255)    # BGR!
                #config.log(f"cart pos: {config.oCart.x}/{config.oCart.y}, map: {mapX}/{mapY}")
                config.addArrow(colImg, mapX, mapY, config.oCart.orientation, 10, cartColor)
                cv2.circle(colImg,(mapX, mapY), 3, cartColor, -1)

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
                scanLocColor = (0,0,255)
                for scanLocation in config.scanLocations:
                    scanLocX, scanLocY = navMap.evalMapLocation(scanLocation[0], scanLocation[1])
                    cv2.circle(colImg,(scanLocX,scanLocY), 3, scanLocColor, -1)


            # limit map to show only drawn objects, no black border
            if config.floorPlan is not None:
                mask = config.floorPlan > 0
                coords = np.argwhere(mask)  # Coordinates of non-black pixels.
                y0, x0 = coords.min(axis=0)             # Bounding box of non-black pixels.
                y1, x1 = coords.max(axis=0) + 1   # slices are exclusive at the top

                cropped = colImg[y0:y1, x0:x1]    # Get a pointer to the bounding box within colImg

                height, width, channel = cropped.shape
                bytesPerLine = 3 * width
                croppedCopy = np.copy(cropped)  # looks like nobody knows why this is necessary but it works (currently)
                qImg = QtGui.QImage(croppedCopy.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)

                pix = QtGui.QPixmap(qImg)

                """
                painter = QtGui.QPainter(self)
                pen = QtGui.QPen(Qt.red, 3)
                painter.setPen(pen)
    
                painter.drawEllipse(config.oCart.x, config.oCart.y,5,5)
                """

                self.Map.setPixmap(pix)
                self.Map.setAlignment(Qt.AlignCenter)
                self.Map.setScaledContents(True)
                self.Map.setMinimumSize(1,1)
                self.Map.show()
                #config.log(f"end gui map update")


        if type == guiUpdate.updType.CART_INFO.value:

            self.cartOrientation.setText(f"{config.oCart.orientation}")
            self.cartLocation.setText(f"{config.oCart.x:4.0f}, {config.oCart.y:4.0f}")
            mapX, mapY = navMap.evalMapLocation(config.oCart.x, config.oCart.y)
            self.mapLocation.setText(f"{mapX:4.0f}, {mapY:4.0f}")


        if type == guiUpdate.updType.BATTERY_UPDATE.value:
            if config.batteryStatus['plugged']:
                msg = f"plugged, {config.batteryStatus['percent']}%"
            else:
                msg = f"battery, {config.batteryStatus['percent']}%"

            self.laptop.setText(msg)
            self.v12.setText(str(config.batteryStatus['v12']))
            self.v6.setText(str(config.batteryStatus['v6']))

