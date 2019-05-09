
import time

import config
import rpcSend


MRL_REST_API = "http://192.168.0.17:8888/api/service/"

getframe_expr = 'sys._getframe({}).f_code.co_name'


def appendToMoveQueue(d):

    # remove oldest element in queue if queue is full
    if len(config.robotMovesQueue) == config.robotMovesQueue.maxlen:
        config.robotMovesQueue.popleft()
    config.robotMovesQueue.append(d)


def servoMoveToPosition(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)


def servoMoveToPositionBlocking(servoName, position, duration=1000):
    rpcSend.servoRequestPos(servoName, position, duration)
    timeout = time.time() + duration/1000.0 + 3
    while True:
        # check local position as we get servo updates automatically
        time.sleep(0.1)
        _, currPosition, _ = config.servers['servoControl'].conn.root.exposed_getPosition(servoName)
        if time.time() > timeout:
            config.log(f"servoMoveToPositionBlocking timeout, requestedPosition: {position}, current position: {currPosition}")
            return
        if abs(currPosition - position) < 2:
            return


def servoMoveToDegrees(servoName, degrees, duration=1000):
    rpcSend.servoRequestDeg(servoName, degrees, duration)


def servoMoveToDegreesBlocking(servoName, degrees, duration=1000):
    rpcSend.servoRequestDeg(servoName, degrees, duration)
    timeout = time.time() + duration/1000.0 + 3
    while True:
        time.sleep(0.1)
        try:
            currDegrees, _, moving = rpcSend.servoGetPosition(servoName)
        except Exception as e:
            currDegrees = 0
            config.log(f"could not move servo: {servoName} to {degrees}, {e}")
        if time.time() > timeout:
            config.log(f"servoMoveToDegreesBlocking timeout, requestedDegrees: {degrees}, current degrees: {currDegrees}")
            return
        if abs(currDegrees - degrees) < 2:
            return


def servoGetPosition(servoName):
    return rpcSend.servoGetPosition(servoName)


def servoSetAutoDetach(servoName, duration):
    rpcSend.servoSetAutoDetach(servoName, duration)


def servoRest(servoName):
    rpcSend.servoRest(servoName)


def servoRestAll():
    rpcSend.servoRestAll()


def cartMovePose():
    rpcSend.requestMovePose()


def servoStop(servoName):
    rpcSend.servoStop(servoName)


def servoStopAll():
    rpcSend.servoStopAll()


def servoControl(servo, method, value=90, duration=1000):
    """
    use rpyc commands to communicate with inmoovControl
    """
    if config.servers['servoControl'].simulated:
        pass
    else:
        if servo[:4] == 'i01.':
            servo = servo[4:]

        if method == 'moveToPosition':
            config.log(f"--> change to servoMoveToPosition")
            servoMoveToPosition(servo, value, duration)

        elif method == 'moveToPositionBlocking':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToPositionBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'moveToDegrees':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToPositionBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'moveToDegreesBlocking':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoMoveToDegreesBlocking in {caller}")
            servoMoveToPositionBlocking(servo, value, duration=1000)

        elif method == 'getCurrentPos':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoGetPosition in {caller}")
            servoGetPosition(servo)

        elif method == 'setAutoDetach':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoSetAutoDetach in {caller}")
            servoSetAutoDetach(servo, value)

        elif method == 'rest':
            caller = eval(getframe_expr.format(2))
            config.log(f"--> change to servoRest in {caller}")
            servoRest(servo)

        else:
            config.log(f"servoControl, unhandled method '{method}'")

    #navGlobal.log(f"sendMrlCommand, {url}, duration: {time.time()-start:.2f}")


def stopRobot(reason):

    config.log(f"stop Robot received, reason: {reason}")

    if config.servers['cartControl'].simulated:
        config.log(f"stop cart")
    else:
        rpcSend.servoStopAll()
        time.sleep(1)


def approachTarget(target):
    config.log(f"TODO robotControl.approachTarget {target}")
    time.sleep(5)
    return True


