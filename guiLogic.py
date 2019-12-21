
import numpy as np
import cv2
import simplejson as json
import os

from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt, pyqtSlot

import config
import gui
import guiUpdate
import navMap
import threadWatchConnections
import navManager


def saveMapSettings():
    a = open("D:/Projekte/InMoov/navManager/mapSettings.json", "w")
    a.write(json.dumps(config.mapSettings))
    a.close()


class gui(QtWidgets.QMainWindow, gui.Ui_MainWindow):

    def __init__(self, *args, **kwargs):
        QtWidgets.QDialog.__init__(self, *args, **kwargs)

        self.setupUi(self)

        self.threads = []

        self.move(100,100)

        if os.path.isfile("D:/Projekte/InMoov/navManager/mapSettings.json"):
            config.mapSettings = json.load(open("D:/Projekte/InMoov/navManager/mapSettings.json"))
        if config.mapSettings['showCart']:
            self.showCart.setChecked(config.mapSettings['showCart'])
        if config.mapSettings['showScanLocations']:
            self.showScanLocations.setChecked(config.mapSettings['showScanLocations'])
        if config.mapSettings['showTarget']:
            self.showTarget.setChecked(config.mapSettings['showTarget'])
        if config.mapSettings['showMarkers']:
            self.showMarkers.setChecked(config.mapSettings['showMarkers'])
        if config.mapSettings['showMovePath']:
            self.showMovePath.setChecked(config.mapSettings['showMovePath'])

        # mark simulated subTasks
        if config.servers['cartControl'].simulated:
            self.cartControl.setCheckState(Qt.PartiallyChecked)
        if config.servers['robotControl'].simulated:
            self.robotControl.setCheckState(Qt.PartiallyChecked)

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
        threadWatchConnections.tryToRestartServer(server)

    def on_showCart_stateChanged(self):
        config.mapSettings['showCart'] = self.showCart.isChecked()
        saveMapSettings()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

    def on_showScanLocations_stateChanged(self):
        config.mapSettings['showScanLocations'] = self.showScanLocations.isChecked()
        saveMapSettings()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

    def on_showTarget_stateChanged(self):
        config.mapSettings['showTarget'] = self.showTarget.isChecked()
        saveMapSettings()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

    def on_showMarkers_stateChanged(self):
        config.mapSettings['showMarkers'] = self.showMarkers.isChecked()
        saveMapSettings()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})

    def on_showMovePath_stateChanged(self):
        config.mapSettings['showMovePath'] = self.showMovePath.isChecked()
        saveMapSettings()
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP})


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
    @pyqtSlot(object)
    def updateGui(self, data):

        #print(f"in updateGui, {data}")
        if data['type'] == guiUpdate.updType.CONNECTION_UPDATE:

            oData = config.objectview(data)
            config.log(f"updateGui, CONNECTION_UPDATE, server: {oData.server}, state: {oData.state}")
            checked, style = self.connectionMarkup(oData.state)

            if oData.server == 'taskOrchestrator':
                self.taskOrchestrator.setChecked(checked)
                self.taskOrchestrator.setStyleSheet(style)
            elif oData.server == 'robotControl':
                self.robotControl.setChecked(checked)
                self.robotControl.setStyleSheet(style)
            elif oData.server == 'cartControl':
                self.cartControl.setChecked(checked)
                self.cartControl.setStyleSheet(style)
            else:
                config.log(f"guiLogic, CONNECTION_UPDATE received for unknown server {oData.server}")


        if data['type'] == guiUpdate.updType.MAP:

            #config.log(f"start gui map update")
            try:
                colImg = cv2.cvtColor(config.floorPlan, cv2.COLOR_GRAY2RGB)
            except Exception as e:
                config.log(f"can not show map: {e}")
                return

            # draw hair cross at 0,0
            cv2.line(colImg, (490,500), (510, 500), config.mapCenterColor , 1)
            cv2.line(colImg, (500, 490), (500, 510), config.mapCenterColor, 1)

            # add cart at position and degrees
            if self.showCart.isChecked():
                degrees = config.oCart.getCartYaw()
                navMap.addCart(colImg, config.oCart.getCartX(), config.oCart.getCartY(), config.oCart.getCartYaw(), config.oCart.mapColor)

            # show target if requested
            if self.showTarget.isChecked():
                navMap.addCart(colImg, config.oTarget.getCartX(), config.oTarget.getCartY(), config.oCart.getCartYaw(), config.oTarget.mapColor)

            # show scanLocations if requested
            if self.showScanLocations.isChecked():
                for scanLocation in config.scanLocations:
                    scanLocX, scanLocY = navMap.evalMapLocation(scanLocation[0], scanLocation[1])
                    cv2.circle(colImg,(scanLocX,scanLocY), 3, config.scanLocationColor, -1)

            # show markers if requested
            if self.showMarkers.isChecked():
                for marker in config.markerList:
                    navMap.addMarker(colImg, marker.markerX, marker.markerY, marker.markerDegrees)

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


        if data['type'] == guiUpdate.updType.CART_INFO:

            self.cartDegrees.setText(f"{config.oCart.getCartYaw()}")
            self.cartLocation.setText(f"{config.oCart.getCartX():4.0f}, {config.oCart.getCartY():4.0f}")
            mapX, mapY = navMap.evalMapLocation(config.oCart.getCartX(), config.oCart.getCartY())
            self.mapLocation.setText(f"{mapX:4.0f}, {mapY:4.0f}")


        if data['type'] == guiUpdate.updType.BATTERY_UPDATE:
            if config.batteryStatus['plugged']:
                msg = f"plugged, {config.batteryStatus['percent']}%"
            else:
                msg = f"battery, {config.batteryStatus['percent']}%"

            self.laptop.setText(msg)
            self.v12.setText(str(config.batteryStatus['v12']))
            self.v6.setText(str(config.batteryStatus['v6']))

