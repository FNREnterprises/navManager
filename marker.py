
import time
import numpy as np
import cv2
import os
import simplejson as json

from dataclasses import dataclass
from marvinglobal import marvinglobal as mg
from marvinglobal import cartClasses

#import inmoovGlobal
import config
#import rpcSend
#import robotHandling
import navMap
import aruco


#rpyc.core.protocol.DEFAULT_CONFIG['allow_pickle'] = True


foundMarkers = []
rotheadMoveDuration = 800




