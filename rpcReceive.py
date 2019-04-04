import time
import rpyc
import simplejson as json

import config
import robotControl
import rpcSend
import navManager
import guiUpdate


class rpcListener(rpyc.Service):

    def on_connect(self, conn):
        clientConn = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_connect {clientConn}")


    def on_disconnect(self, conn):
        clientName = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_disconnect received {clientName}")
        config.servers[clientName].connectionState = 'down'
        print()


    def exposed_lifeSignalUpdate(self, server):
        config.servers[server].lifeSignalReceived = time.time()
        #config.log(f"life signal received from server: {server}")
        if config.servers[server].connectionState == 'try':
            config.servers[server].connectionState = 'connected'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'connected'})


    def exposed_serverReady(self, server, ready):
        config.log(f"ready={ready} message from server {server} received")
        if ready:  # and not config.servers[server]['serverReady']:
            config.servers[server].connectionState = 'ready'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'ready'})


        if not ready:  # and config.servers[server]['serverReady']:
            config.servers[server].connectionState = 'connected'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'connected'})


    def exposed_log(self, msg):
        config.othersLog(msg)


    def exposed_lowBattery(self, msg):
        navManager.setTask("dock")


    def exposed_startDockingPhase1(self, startRotation, cartMove, endRotation):
        config.log(f"startDockingPhase1 received from aruco")
        robotControl.cartMovesDockingPhase1(startRotation, cartMove, endRotation)


    def exposed_startDockingPhase2(self, rotation, distance):
        # this may request an initial small rotation and a move to left or right
        config.log(f"dockingPhase2: verify proper location befor activating phase2")
        #robotControl.cartMovesDockingPhase2(rotation, distance)


    def exposed_dockingFailed(self):
        config.log(f"docking failed received")
        navManager.setTask("notask")


    def exposed_cartDocked(self, newStatus):
        config.oCart.docked = newStatus


    def exposed_updateCartInfo(self, cartX, cartY, orientation, cartMoving, cartRotating):
        config.oCart.setX(cartX)
        config.oCart.setY(cartY)
        config.oCart.setYaw(orientation)
        config.oCart.moving = cartMoving
        config.oCart.rotating = cartRotating
        config.oCart.update = time.time()
        #config.log(f"updateCartInfo received: {config.oCart.getX()}, {config.oCart.getY()}, {config.oCart.getYaw()},  {config.oCart.moving}, {config.oCart.rotating}", publish=False)
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CART_INFO.value})
        guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.MAP.value})


    def exposed_servoUpdate(self, msg):

        #config.log(f"received a servoUpdate: {msg}")
        servoData = json.loads(msg)
        config.log(f"servo update for servo: {servoData['servoName']}")
        try:
            config.servoCurrent[servoData['servoName']].update(servoData)
        except KeyError:
            config.log(f"key error with servo update {servoData}")


    def exposed_obstacleUpdate(self, data):
        distance = data.split(',')[0]
        angle = data.split(',')[1]

        config.log(f"obstacleUpdate, distance: {distance}, angle: {angle}")



