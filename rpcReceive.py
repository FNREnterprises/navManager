import time
import rpyc
import simplejson as json

import config
import navManager
import guiUpdate
import cartHandling


class rpcListener(rpyc.Service):

    def on_connect(self, conn):
        clientConn = conn._channel.stream.sock.getpeername()
        config.log(f"navManagerListener on_connect {clientConn}")


    def on_disconnect(self, conn):
        config.log(f"on_disconnect got triggered, {conn}")


    def exposed_lifeSignalUpdate(self, server):
        config.servers[server].lifeSignalReceived = time.time()
        #config.log(f"life signal received from server: {server}")
        if config.servers[server].connectionState == 'try':
            config.servers[server].connectionState = 'connected'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'connected'})


    def exposed_serverReady(self, server, ready):
        config.log(f"ready={ready} message from server {server} received")
        if ready:  # and not config.servers[server]['serverReady']:
            config.servers[server].connectionState = 'ready'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'ready'})

        if not ready:  # and config.servers[server]['serverReady']:
            config.servers[server].connectionState = 'connected'
            guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'connected'})


    def exposed_servoControlBasicData(self, jsonMsg):
        # needs to be called by servo control when ready

        config.log(f"getting basic data from servo control")
        allServoData = json.loads(jsonMsg)
        config.servoCurrent = allServoData
        #for d in allServoData:
        #    config.servoCurrent.update({d['servoName']: d})
        config.servers["servoControl"].basicDataReceived = True


    def exposed_cartControlBasicData(self, jsonMsg):
        # needs to be called by cart control when ready

        config.log(f"getting basic data from cartControl")
        #rpcSend.queryCartInfo()
        #rpcSend.queryBatteries()
        config.servers["cartControl"].basicDataReceived = True

        #rpcSend.powerKinect(True)
        # for unknown reason, this ruins the connection with cartControl ?????


    def exposed_log(self, msg):
        config.othersLog(msg)


    def exposed_lowBattery(self):
        navManager.setTask("dock")


    def exposed_dockingFailed(self):
        config.log(f"docking failed received")
        navManager.setTask("notask")


    def exposed_cartDocked(self, newStatus):
        config.oCart.docked = newStatus


    def exposed_cartProgress(self,
            magnitude,
            final,
            cartX,
            cartY,
            degrees,
            cartMoving,
            cartRotating):

        cartHandling.updateCartInfo(cartX, cartY, degrees, cartMoving, cartRotating)

        if config.oMoveSteps is not None:
            config.oMoveSteps.updateMove(magnitude, final)


    def exposed_updateCartInfo(self, cartX, cartY, degrees, cartMoving, cartRotating):
        cartHandling.updateCartInfo(cartX, cartY, degrees, cartMoving, cartRotating)



    def exposed_servoUpdate(self, servoName, msg):

        #config.log(f"received a servoUpdate: {msg}")
        servoData = json.loads(msg)
        #config.log(f"servo update for servo: {servoData['servoName']}")
        try:
            config.servoCurrent[servoName].update(servoData)
        except KeyError:
            config.log(f"key error with servo update {servoName}, {servoData}")


    def exposed_obstacleUpdate(self, data):
        distance = data.split(',')[0]
        angle = data.split(',')[1]

        config.log(f"obstacleUpdate, distance: {distance}, angle: {angle}")



