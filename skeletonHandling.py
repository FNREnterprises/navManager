
import time
import config
from marvinglobal import marvinglobal as mg

def requestServoDegrees(servoName, degrees, duration, wait=True) -> bool:

    request = {'msgType': 'requestDegrees', 'sender': config.processName,
               'servoName': servoName, 'degrees': degrees, 'duration': duration, 'sequential': True}
    config.marvinShares.skeletonRequestQueue.put(request)

    if wait:
        if not waitForServoDegreesReached(servoName, degrees, duration):
            config.log(f"could not position {servoName} to {degrees} degrees within {duration} ms")
            return False

    return True


def waitForServoDegreesReached(servoName, requestedDegrees, duration) -> bool:

    config.log(f"wait for {servoName} at {requestedDegrees} degrees, timeout={duration / 1000.0 + 3} s")

    servoCurrentDict = config.marvinShares.servoDict.get(mg.SharedDataItems.SERVO_CURRENT)
    servoCurrent = servoCurrentDict.get(servoName)  # refresh state
    maxWaitSeconds = duration / 1000 + 3
    timeout = time.time() + maxWaitSeconds
    while abs(servoCurrent.currentDegrees - requestedDegrees) > 3 and time.time() < timeout:
        time.sleep(0.1)
        servoCurrentDict = config.marvinShares.servoDict.get(mg.SharedDataItems.SERVO_CURRENT)
        servoCurrent = servoCurrentDict.get(servoName)  # refresh state
        # config.log(f"{servoName}, {servoCurrent.currentDegrees=}")
    if time.time() >= timeout:
        config.log(
            f"waitForServoDegreesReached {requestedDegrees=}, timeout {maxWaitSeconds} s reached, currentDegrees: {servoCurrent.currentDegrees}")
    return time.time() < timeout
