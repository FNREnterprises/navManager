
from marvinglobal import marvinglobal as mg
import config

def publishCartLocation():
    #updStmt:Tuple[mg.SharedDataItems, marvinglobal.mg.Location] = (mg.SharedDataItems.CART_LOCATION, config.locationLocal)
    updStmt = {'cmd': mg.SharedDataItems.CART_LOCATION, 'sender': config.processName,
               'info': config.cartLocationLocal}
    config.marvinShares.updateSharedData(updStmt)

