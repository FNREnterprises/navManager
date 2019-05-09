
import time
import rpyc

import config
import rpcSend
import rpcReceive
import guiUpdate


SERVER_WATCH_INTERVAL = 5


def tryToRestartServer(server):

    if config.taskOrchestrator.closed:
        # we have lost connection with taskOrchestrator, try to reconnect
        config.log(f"connection with taskOrchestrator is closed, try to connect")
        try:
            config.taskOrchestrator = rpyc.connect(config.marvin, 20000, service = rpcReceive.rpcListener)
        except Exception as e:
            config.log(f"could not connect with taskOrchestrator, {e}")
        return

    oServer = config.servers[server]
    oServer.conn = None
    oServer.startRequested = time.time()
    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'down'})
    config.log(f"trying to restart server {server}")
    try:
        config.taskOrchestrator.root.restartServer(server)
    except Exception as e:
        config.taskOrchestrator.close()
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
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'down'})
            elif lifeSignalRoundTrip > SERVER_WATCH_INTERVAL + 2:
                config.log(f"life signal round trip too slow with server '{server}', roundTrip: {lifeSignalRoundTrip}")
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'down'})
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
                guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'try'})
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


        # watch interval actions
        if server == 'cartControl':

            rpcSend.queryBatteries()

        time.sleep(SERVER_WATCH_INTERVAL)
