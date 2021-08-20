
import time
import numpy as np
import cv2
import os
import simplejson as json

from dataclasses import dataclass
from marvinglobal import marvinglobal as mg
from marvinglobal import cartClasses

import config
import navMap


foundMarkers = []
rotheadMoveDuration = 800




