
import time
import rpyc
import simplejson as json
import winsound

import config
import rpcSend
import guiUpdate



SERVER_WATCH_INTERVAL = 5


def tryToRestartServer(server):

    oServer = config.objectview(config.navManagerServers[server])
    oServer.conn = None
    oServer.startRequested = time.time()
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
    try:
        config.taskOrchestrator.root.restartServer(server)
    except Exception as e:
        config.log(f"failure on requesting restart of server {self.server}, {e}")


# loop here
def watchConnection(server):

    oServer = config.objectview(config.navManagerServers[server])

    while True:

        #config.log(f"watchdog, requesting heart beat from servers")

        # check for simulated
        if oServer.simulated:
            continue

        # check for server start requested
        if oServer.startRequested is None:
            tryToRestartServer(server)
            continue

        # check for server startuptime expired
        if time.time() < oServer.startRequested + oServer.startupTime:
            # server still in startup
            continue

        else:

            # check for life signal expired
            lifeSignalRoundTrip = oServer.lifeSignalReceived - oServer.lifeSignalRequest

            if lifeSignalRoundTrip < 0:
                config.log(f"life signal missing from server '{server}', roundTrip: {lifeSignalRoundTrip} ")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
            if lifeSignalRoundTrip > 3:
                config.log(f"life signal round trip too slow with server '{server}', request: {lifeSignalRoundTrip}")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})

            if oServer.conn is None:

                #########################
                #  try to connect with server
                #########################
                config.log(f"try to connect with '{server}' at {oServer.ip}, {oServer.port}")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'try'})
                try:
                    oServer.conn = rpyc.connect(oServer.ip, oServer.port)
                except Exception as e:
                    config.log(f"could not connect with '{server}', exception: {e}")
                    tryToRestartServer(server)
                    continue

                # try to open a response connection in the server
                try:
                    oServer.conn.root.requestForReplyConnection(config.MY_IP, config.MY_PORT)
                except Exception as e:
                    config.log(f"could not request reply connection with '{server}', exception: {e}")
                    tryToRestartServer(server)
                    continue


                config.log(f"first life signal request from '{server}'")
                good = False
                config.navManagerServers[server]['lifeSignalRequest'] = time.time()
                try:
                    oServer.conn.root.exposed_requestLifeSignal(config.MY_IP, config.MY_PORT)
                except Exception as e:
                    tryToRestartServer(server)
                    config.log(f"requesting life signal from '{server}' failed, exception: {str(e)}")



                ############################################################
                # place to get first time basic information from servers
                ############################################################
                if server == 'servoControl':
                    # get current servo information
                    if oServer.conn is not None:
                        config.servoCurrent = {}
                        jsonMsg = oServer.conn.root.exposed_getServoCurrentList()
                        allServoData = json.loads(jsonMsg)
                        for d in allServoData:
                            config.servoCurrent.update({d['servoName']: d})

                if server == 'cartControl':
                    config.log(f"first connection with cartControl, query cart info")
                    if oServer.conn is not None:
                        rpcSend.queryCartInfo()


            else:
                # if we have a connection with the server ...
                #config.log(f"request Life signal from server '{server}' ")
                config.navManagerServers[server]['lifeSignalRequest'] = time.time()
                try:
                    oServer.conn.root.requestLifeSignal(config.MY_IP, config.MY_PORT)
                except Exception as e:
                    config.log(f"failure requesting life signal from '{server}', try to restart")
                    tryToRestartServer(server)


        # get battery status from cart
        if server == 'cartControl':
            try:
                config.batteryStatus = oServer.conn.root.getBatteryStatus()
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.BATTERY_UPDATE.value})
                if config.batteryStatus < 10:
                    winsound.PlaySound('sound.wav', winsound.SND_FILENAME)
            except Exception as e:
                config.log(f"could not get battery status from cartControl: {e}", publish=False)


        time.sleep(SERVER_WATCH_INTERVAL)
