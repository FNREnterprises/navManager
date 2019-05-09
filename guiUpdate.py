
import time
from collections import deque

from PyQt5 import QtCore

from enum import Enum

class updType(Enum):
    CONNECTION_UPDATE = 1
    MAP = 2
    CART_INFO = 3
    BATTERY_UPDATE = 4


# queue for gui updates
guiUpdateQueue = deque(maxlen=100)


def getOldestUpdateMessage():
    if len(guiUpdateQueue) > 0:
        return guiUpdateQueue.popleft()
    else:
        return None


class GuiUpdateThread(QtCore.QThread):
    """
    This checks for new data in the guiUpdateQueue
    the queue can have different types of data based on the type attribute
    """

    # signal for gui update
    # raises guiLogic.updateGui
    updateGui = QtCore.pyqtSignal(object)

    def __init__(self):
        QtCore.QThread.__init__(self)


    def run(self):

        time.sleep(2)       # wait for gui to startup
        #config.setUpdateRunning(False)

        while True:
            #if config.getUpdateRunning():
            #    time.sleep(0.01)
            #    continue

            updateData = getOldestUpdateMessage()

            if updateData is not None:

                #print(f"guiUpdate: {updateData}")

                #if updateData['type'] == guiLogic.updType.CONNECTION_UPDATE.value:

                self.updateGui.emit(updateData) # see guiLogic.updateGui

