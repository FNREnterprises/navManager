
import time
import os
import numpy as np
import cv2
import simplejson as json

import inmoovGlobal
import config
import navMap
import rpcSend
import marker
import aruco


def createObstacleMap(kinectDistances, depthCamX, depthCamY, depthCamDegrees, show=False):

    # add the distance information in an image of size floorPlan
    config.obstacleMap = np.zeros((navMap.MAP_HEIGHT, navMap.MAP_WIDTH), dtype=np.uint8)      # numpy image is rows, cols
    config.obstacleMapFat = np.zeros_like(config.obstacleMap)

    mapX = int(depthCamX / navMap.MM_PER_MAP_PIXEL + navMap.MAP_WIDTH / 2)
    mapY = int(navMap.MAP_HEIGHT / 2 - (depthCamY / navMap.MM_PER_MAP_PIXEL))

    for col, dist in enumerate(kinectDistances):
        if not np.isnan(dist) and dist/navMap.MM_PER_MAP_PIXEL < int(config.obstacleMap.shape[1]/2):

            colAngle = (col * config.KINECT_X_RANGE / 640) - (config.KINECT_X_RANGE / 2)
            pointAngle = colAngle + depthCamDegrees
            pX = int(dist/navMap.MM_PER_MAP_PIXEL * np.cos(np.radians(pointAngle)))
            pY = -int(dist/navMap.MM_PER_MAP_PIXEL * np.sin(np.radians(pointAngle)))

            cv2.circle(config.obstacleMap, (pX + mapX, pY + mapY), 1, 255, -1)   # opencv point is (x, y)!!!!

            # the fat version of the obstacle map takes the size of the robot into account in order to keep the
            # center of the robot away from the obstacle line
            cv2.circle(config.obstacleMapFat, (pX + mapX, pY + mapY), navMap.cartRadiusPix, 255, -1)
            #config.log(f"addPartialMap fat circle added")

    if show:
        img = config.obstacleMap
        cv2.line(img, (490,500),(510,500), 255, 1)
        cv2.line(img, (500,490),(500,510), 255, 1)
        cv2.imshow("verify obstacle map", img)
        cv2.waitKey(1000)
        cv2.destroyAllWindows()


def addPartialMap():
    """
    this function is called by the update floor plan thread
    it uses the array of distances for each column (640) as created by the kinect task

    # temporary values for addPartialMap as it runs in separate thread
    depthCamDistances = None
    depthCamDegrees = carts degrees at time of depth image taken
    kinectLocation = carts location at time of depth image taken
    """
    vector2d = navMap.buildImageName(navMap.depthCamX, navMap.depthCamY, navMap.depthCamDegrees)
    config.log(f"addPartialMap, {vector2d}")

    # store the distance array for verification, distances are from right to left!
    filename = f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/planPart{vector2d}.json"
    with open(filename, 'w') as jsonFile:
        json.dump(np.nan_to_num(navMap.depthCamDistances).tolist(), jsonFile, indent=2)

    if not os.path.isfile(filename):
        config.log(f"could not create <obstacle>.json file {filename}")
        return False

    config.log(f"addPartialMap json file saved {filename}")

    createObstacleMap(navMap.depthCamDistances, navMap.depthCamX, navMap.depthCamY, navMap.depthCamDegrees, show=False)

    # only for visual control of obstacles
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/planPart{vector2d}.jpg", config.obstacleMap)

    # add the partial part to the new scan plan
    cv2.addWeighted(config.fullScanPlan, 1, config.obstacleMap, 1, 0.0, config.fullScanPlan)
    cv2.addWeighted(config.fullScanPlanFat, 1, config.obstacleMapFat, 1, 0.0, config.fullScanPlanFat)

    # IF WE ARE IN THE FIRST FULL SCAN add the obstacle Map also to the floor plan
    if not config.fullScanDone:
        cv2.addWeighted(config.floorPlan, 1, config.obstacleMap, 1, 0.0, config.floorPlan)
        cv2.addWeighted(config.floorPlanFat, 1, config.obstacleMapFat, 1, 0.0, config.floorPlanFat)
        config.log(f"addPartialMap, obstacleMap added to floorPlan too")

    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/floorPlan{vector2d}.jpg", config.fullScanPlan)
    cv2.imwrite(f"{config.PATH_ROOM_DATA}/{config.room}/floorPlan/floorPlanParts/floorPlanFat{vector2d}.jpg", config.fullScanPlanFat)

    config.log(f"addPartialMap done")
    return True


###########################################################
###########################################################
# runs as separate thread
###########################################################
###########################################################

def loop():
    """
    this runs in its own thread and is controlled by flags
    :return:
    """
    config.log(f"updateFloorPlan thread started")
    while True:

        if config.flagProcessCartcamImage:

            config.log(f"process cartcam image")

            imageName = navMap.buildImageName(config.oCart.getCartX(), config.oCart.getCartY(), config.oCart.getCartYaw())
            imgPath = f"{config.PATH_ROOM_DATA}/{config.room}/cartcamImages/{imageName}.jpg"
            cv2.imwrite(imgPath, config.cartcamImage)

            # try to find markers in cartcam image
            markersFound = aruco.lookForMarkers("CART_CAM", [], 0)
            if markersFound:
                config.log(f"updateFloorPlan, markers found: {markersFound}")
                marker.updateMarkerFoundResult(markersFound, 'CART_CAM', config.oCart.getCartX(), config.oCart.getCartY(), config.oCart.getCartYaw(), 0)
            config.flagProcessCartcamImage = False


        if config.flagProcessDepthcamImage:

            config.log(f"process depth image, updateFloorPlan")

            depthImgX = config.oCart.getCartX()
            depthImgY = config.oCart.getCartY()
            depthImgDegrees = config.oCart.getCartYaw()

            # find obstacle line

            # add to map
            addPartialMap()

            config.log(f"depth image processed, depthImgX: {depthImgX}, depthImgY: {depthImgY}")
            config.flagProcessDepthImage = False


        time.sleep(0.1)
