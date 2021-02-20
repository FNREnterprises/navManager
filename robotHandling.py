


import config

def requestServoDegrees(servo, degrees, duration=500):
    q = config.marvinShares.skeletonRequestQueue
    m = mg.skeletonCommandMethods
    m.requestDegrees(q, servo, degrees, duration)

def servoRestAll():
    request = {'sender': config.processName, 'cmd': mg.ServoCommand.STOP_CART, 'reason': reason}

def stopCart(reason):
    request = {'sender': config.processName, 'cmd': mg.CartCommand.STOP_CART, 'reason': reason}
    config.marvinShares.cartRequestQueue.put(request)

