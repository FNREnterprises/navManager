
import time
import rpyc
import simplejson as json

import config
import rpcSend
import guiUpdate



SERVER_WATCH_INTERVAL = 5


def tryToRestartServer(server):

    oServer = config.servers[server]
    oServer.conn = None
    oServer.startRequested = time.time()
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
    config.log(f"trying to restart server {server}")
    try:
        config.taskOrchestrator.root.restartServer(server)
    except Exception as e:
        config.log(f"failure on requesting restart of server {server}, {e}")


# separate thread for each server
def watchConnection(server):

    oServer = config.servers[server]

    # assume server is already running by setting startRequested into the past
    oServer.startRequested = time.time()-30

    while True:

        #config.log(f"watchdog, requesting heart beat from server {server}")

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
            config.log(f"{server} in startup")
            time.sleep(SERVER_WATCH_INTERVAL)
            continue

        else:

            # check for life signal expired
            lifeSignalRoundTrip = oServer.lifeSignalReceived - oServer.lifeSignalRequest

            if lifeSignalRoundTrip < 0:
                config.log(f"life signal missing from server '{server}', roundTrip: {lifeSignalRoundTrip:.0f} ")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
            elif lifeSignalRoundTrip > SERVER_WATCH_INTERVAL + 2:
                config.log(f"life signal round trip too slow with server '{server}', roundTrip: {lifeSignalRoundTrip}")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE.value, 'server': server, 'state': 'down'})
                oServer.connectionState = 'down'
                oServer.conn = None
            else:
                #config.log(f"life signal from {server} received, round trip duration: {lifeSignalRoundTrip:.1f}")
                pass


            if oServer.conn is None:

                #########################
                #  try to connect with server
                #########################
                config.log(f"try to connect with '{server}' at {oServer.ip}, {oServer.port}")
                config.servers[server].connectionState = 'try'
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
                    oServer.conn.root.requestForReplyConnection(config.MY_IP, config.MY_RPC_PORT)
                except Exception as e:
                    config.log(f"could not request reply connection with '{server}', exception: {e}")
                    tryToRestartServer(server)
                    continue


                config.log(f"first life signal request from '{server}'")
                good = False
                config.servers[server].lifeSignalRequest = time.time()
                try:
                    oServer.conn.root.exposed_requestLifeSignal(config.MY_IP, config.MY_RPC_PORT)
                except Exception as e:
                    tryToRestartServer(server)
                    config.log(f"requesting life signal from '{server}' failed, exception: {str(e)}")

            else:
                # if we have a connection with the server ...
                #config.log(f"watchdog, request Life signal from server '{server}' ")
                config.servers[server].lifeSignalRequest = time.time()
                try:
                    oServer.conn.root.requestLifeSignal(config.MY_IP, config.MY_RPC_PORT)
                except Exception as e:
                    config.log(f"failure requesting life signal from '{server}', trying to restart server")
                    tryToRestartServer(server)


                ############################################################
                # place to get first time basic information from servers
                ############################################################
                if server == 'servoControl':

                    # get current servo information
                    if config.servers[server].connectionState == 'ready' and not config.servers[server].basicDataReceived:
                        config.log(f"first connection with servoControl, get basic data")
                        config.servoCurrent = {}
                        jsonMsg = config.servers[server].conn.root.exposed_getServoCurrentList()
                        allServoData = json.loads(jsonMsg)
                        for d in allServoData:
                            config.servoCurrent.update({d['servoName']: d})
                        config.servers[server].basicDataReceived = True

                if server == 'cartControl':
                    if config.servers[server].connectionState == 'ready' and not config.servers[server].basicDataReceived:

                        config.log(f"first connection with cartControl, query cart info")
                        rpcSend.queryCartInfo()
                        rpcSend.queryBatteries()
                        config.servers[server].basicDataReceived = True

                        #rpcSend.powerKinect(True)
                        # for unknown reason, this ruins the connection with cartControl ?????


        # watch interval actions
        if server == 'cartControl':

            rpcSend.queryBatteries()

        time.sleep(SERVER_WATCH_INTERVAL)
