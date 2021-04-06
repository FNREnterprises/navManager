
import os
import simplejson as json
from dataclasses import dataclass

from marvinglobal import marvinglobal as mg
from marvinglobal import cartClasses
from marvinglobal import environmentClasses

import config

class Room(environmentClasses.RoomData):
    def __init__(self):
        super().__init__()

        # load or create the latest room info
        roomFolderPath = f"{mg.PERSISTED_DATA_FOLDER}/{mg.ROOM_FOLDER}"
        if not os.path.exists(roomFolderPath):
            os.mkdir(roomFolderPath)

        filePath = f"{mg.PERSISTED_DATA_FOLDER}/{mg.ROOM_FOLDER}/roomInfo.json"
        if os.path.exists(filePath):
            with open(filePath, "r") as read_file:
                roomInfo = json.load(read_file)

            self.roomName = roomInfo['roomName']
            self.fullScanDone = roomInfo['fullScanDone']

        else:
            self.newRoom = True
            self.roomName = 'unknown'
            self.fullScanDone = False
            self.saveRoomInfo()


    def saveRoomInfo(self):
        # Saving current roomInfo into file:
        roomInfo = { 'newRoom': self.newRoom, 'roomName': self.roomName, 'fullScanDone': self.fullScanDone }
        filePath = f"{mg.PERSISTED_DATA_FOLDER}/{mg.ROOM_FOLDER}/roomInfo.json"
        with open(filePath, "w") as write_file:
            json.dump(roomInfo, write_file, indent=2)

        # update shared copy of roomData
        updStmt = {'cmd': mg.SharedDataItems.ENVIRONMENT_ROOM, 'sender': config.processName,
                   'info': config.roomDataLocal}
        config.marvinShares.updateSharedData(updStmt)



###############################

class ScanLocations:
    """
    a scan location is a position in relation to the room center where the robot
    rotates for 360 degrees and captures wall distances and aruco codes
    """

    def __init__(self):
        self.scanLocations = []
        self.filename = f"{config.PATH_ROOM_DATA}/{config.roomDataLocal.roomName}/scanLocations.json"
        self.scanLocationColor = (128, 255, 128)  # light green


    def reset(self):
        self.scanLocations = []
        self.saveScanLocations()


    def addScanLocation(self, location:mg.Location):
        self.scanLocations.append(location)


    def loadScanLocations(self):
        with open(self.filename, "r") as read_file:
            self.scanLocations = json.load(read_file)


    def saveScanLocations(self):
        with open(self.filename, "w") as write_file:
            json.dump(self.scanLocations, write_file, indent=2)


