import time
import rpyc
import simplejson as json

import config
import robotControl
import guiUpdate



class rpcListener(rpyc.Service):

    def on_connect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_connect {clientName}")


    def on_disconnect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_disconnect received {clientName}")
        print()


    def exposed_lifeSignalUpdate(self, server):
        config.navManagerServers[server]['lifeSignalReceived'] = time.time()
        #config.log(f"life signal received, server: '{server}", publish=False)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'up'})


    def exposed_log(self, msg):
        config.othersLog(msg)


    def exposed_lowBattery(self, msg):
        config.setTask("dock")


    def exposed_startDockingPhase1(self, startRotation, cartMove, endRotation):
        config.log(f"startDockingPhase1 received from aruco")
        robotControl.cartMovesDockingPhase1(startRotation, cartMove, endRotation)


    def exposed_startDockingPhase2(self, rotation, distance):
        # this may request an initial small rotation and a move to left or right
        config.log(f"dockingPhase2: verify proper location befor activating phase2")
        #robotControl.cartMovesDockingPhase2(rotation, distance)


    def exposed_dockingFailed(self):
        config.log(f"docking failed received")
        config.setTask("notask")


    def exposed_cartDocked(self, newStatus):
        config.oCart.docked = newStatus


    def exposed_updateCartInfo(self, cartX, cartY, orientation, cartMoving, cartRotating):
        config.oCart.x = cartX
        config.oCart.y = cartY
        config.oCart.orientation = orientation
        config.oCart.moving = cartMoving
        config.oCart.rotating = cartRotating
        config.oCart.update = time.time()
        #config.log(f"updateCartInfo received: {config.oCart.orientation}, {config.oCart.x}, {config.oCart.y}, {config.oCart.moving}, {config.oCart.rotating}", publish=False)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO.value})


    def exposed_servoUpdate(self, msg):

        #config.log(f"received a servoUpdate: {msg}")
        servoData = json.loads(msg)
        config.servoCurrent[servoData['servoName']].update(servoData)


