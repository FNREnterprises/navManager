import os
import rpyc
import simplejson as json

import config
import robotControl

#servoCurrent = {'assigned', 'moving', 'detached', 'position', 'degrees', servoName}
servoCurrent = {}


def initServoCurrent(jsonMsg):
    '''
    called from rpcSend when making first connection with servoControl
    '''

    global servoCurrent

    currServoData = json.loads(jsonMsg)

    # build a dict of servoName with dict of its data
    for d in currServoData:
        servoCurrent.update({d['servoName']: d})


class navManagerListener(rpyc.Service):

    def on_connect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_connect {clientName}")


    def on_disconnect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_disconnect received {clientName}")
        print()


    def exposed_getPid(self):
        return os.getpid(), config.SERVER_WATCH_INTERVAL


    def exposed_log(self, msg):
        config.othersLog(msg)


    def exposed_lowBattery(self, msg):
        config.setTask("dock")

    def exposed_startDockingPhase1(self, startRotation, cartMove, endRotation):
        config.log(f"startDockingPhase1 received from aruco")
        robotControl.cartMovesDockingPhase1(startRotation, cartMove, endRotation)

    def exposed_startDockingPhase2(self, rotation, distance):
        # this may request an initial small rotation and a move to left or right
        config.log("dockingPhase2: verify proper location befor activating phase2")
        #robotControl.cartMovesDockingPhase2(rotation, distance)

    def exposed_dockingFailed(self):
        config.log("docking failed received")
        config.setTask("notask")

    def exposed_cartDocked(self, newStatus):
        config.setCartDocked(newStatus)

    def exposed_updateCartInfo(self, locationX, locationY, orientation):
        config.setCartLocation(locationX, locationY)
        config.setCartOrientation(orientation)

    def exposed_servoUpdate(self, msg):

        global servoCurrent

        #config.log(f"received a servoUpdate: {msg}")
        servoData = json.loads(msg)
        servoCurrent[servoData['servoName']].update(servoData)



class kinectListener(rpyc.Service):

    def on_connect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"kinectListener on_connect seen {clientName}")


    def on_disconnect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"kinectListener on_disconnect received {clientName}")
        print()


    def exposed_log(self, msg):
        config.othersLog(msg)

