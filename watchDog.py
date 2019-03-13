
import time
import rpyc
import simplejson as json

import config
import rpcSend
import guiUpdate



SERVER_WATCH_INTERVAL = 5


def tryToRestartServer(server):

    oServer = config.objectview(config.navManagerServers[server])
    oServer.conn = None
    oServer.startRequested = time.time()
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
    config.log(f"trying to restart server {server}")
    try:
        config.taskOrchestrator.root.restartServer(server)
    except Exception as e:
        config.log(f"failure on requesting restart of server {server}, {e}")


# loop here
def watchConnection(server):

    oServer = config.objectview(config.navManagerServers[server])

    # assume server is already running by setting startRequested into the past
    oServer.startRequested = time.time()-30

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
                config.navManagerServers[server]['connectionState'] = 'try'
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'try'})
                try:
                    oServer.conn = rpyc.connect(oServer.ip, oServer.port, config={'sync_request_timeout': 10})
                    config.log(f"connection established with {server}")
                except Exception as e:
                    config.log(f"could not connect with '{server}', exception: {e}")
                    tryToRestartServer(server)
                    continue

                # try to open a response connection in the server
                config.log(f"try to open a response connection in {server}")
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
                    rpcSend.powerKinect(True)
                    rpcSend.queryCartInfo()
                    rpcSend.queryBatteries()

            else:
                # if we have a connection with the server ...
                #config.log(f"request Life signal from server '{server}' ")
                config.navManagerServers[server]['lifeSignalRequest'] = time.time()
                try:
                    oServer.conn.root.requestLifeSignal(config.MY_IP, config.MY_PORT)
                except Exception as e:
                    config.log(f"failure requesting life signal from '{server}', try to restart")
                    tryToRestartServer(server)


        # watch interval actions
        if server == 'cartControl':

            rpcSend.queryBatteries()

        time.sleep(SERVER_WATCH_INTERVAL)
