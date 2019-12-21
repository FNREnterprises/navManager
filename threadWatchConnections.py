
import time
import rpyc

import config
import rpcSend
import rpcReceive
import guiUpdate
import robotHandling
import inmoovGlobal

SERVER_WATCH_INTERVAL = 1


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

    if oServer.simulated:
        return

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

            if oServer.conn is not None and oServer.lifeSignalReceived is not None:

                # check for life signal expired
                timeSinceLastLifeSignal = time.time() - oServer.lifeSignalReceived

                if timeSinceLastLifeSignal < 0:
                    config.log(f"life signal missing from server '{server}', roundTrip: {timeSinceLastLifeSignal:.0f} ")
                    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'down'})
                    oServer.connectionState = 'down'
                    oServer.conn = None
                elif timeSinceLastLifeSignal > inmoovGlobal.RPYC_HEARTBEAT_INTERVAL * 1.5:
                    config.log(f"life signal missing from server '{server}', timeSinceLastLifeSignal: {timeSinceLastLifeSignal} s")
                    guiUpdate.guiUpdateQueue.append({'type': guiUpdate.updType.CONNECTION_UPDATE, 'server': server, 'state': 'down'})
                    oServer.connectionState = 'down'
                    oServer.conn = None
                else:
                    if oServer.connectionState != 'ready':
                        config.log(f"life signal from {server} received, round trip duration: {timeSinceLastLifeSignal:.1f}")
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

                #oServer.conn.root.requestLifeSignal(config.localIp, config.MY_RPC_PORT)

                # try to open a response connection in the server
                config.log(f"try to open a response connection in {server}")
                oServer.lifeSignalReceived = None
                try:
                    oServer.conn.root.requestForReplyConnection(config.localIp, config.MY_RPC_PORT)
                except Exception as e:
                    config.log(f"could not request reply connection with '{server}', exception: {e}")
                    tryToRestartServer(server)
                    continue

                # wait for first life signal from server
                timeout = time.time() + inmoovGlobal.RPYC_HEARTBEAT_INTERVAL * 1.5
                while oServer.lifeSignalReceived is None and time.time() < timeout:
                    time.sleep(1)

                if oServer.lifeSignalReceived is None:
                    continue

                config.log(f"first life signal from '{server}' received")

                # when connecting with cartControl or robotControl repeat head imu calibration
                if server in ['cartControl', 'robotControl']:
                    config.oHead.isHeadImuCalibrated = False


        # watch interval actions
        if server == 'cartControl':

            rpcSend.queryBatteries()

            # get cam properties
            if len(config.cams) == 0:
                for i in range(inmoovGlobal.NUM_CAMS):
                    props = oServer.conn.root.getCamProperties(i)
                    config.cams.update({i: props})
                    config.log(f"cam properties received from cartControl, cam: {i}")

        # check for calibrated head imu
        if not config.oHead.isHeadImuCalibrated:

            # it needs a running cartControl and robotControl
            if config.servers['cartControl'].connectionState == 'ready' and config.servers['robotControl'].connectionState == 'ready':

                # check for calibration initialized
                if not config.oHead.isHeadImuCalibrationInitialized:

                    # set head neck and rothead to 0
                    config.oHead.startHeadImuCalibration()
                    robotHandling.servoMoveToDegreesBlocking('head.rothead', 0, 500)
                    robotHandling.servoMoveToDegreesBlocking('head.neck', 0, 500)

                    # get head imu values from cart
                    yrp = rpcSend.requestHeadOrientation()
                    if yrp is None:
                        config.log(f"could not get head orientation")
                    else:
                        config.log(f"head imu calibration values: {yrp}")
                        config.oHead.applyHeadImuYawCalibration(yrp[0])

        rpcSend.publishLifeSignal()

        time.sleep(inmoovGlobal.RPYC_HEARTBEAT_INTERVAL)
