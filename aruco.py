
import cv2
from cv2 import aruco       # pip install opencv-contrib-python
import numpy as np

import config

# adapt length to get a correct distance
# include distance of cart front to center (330) into the orthoStopDist
markers = {
    "dockingMarker": {'length':100, 'separation': 0.5, 'orthoStopDist': 600, 'allowedIds': [10] },
    "dockingDetail": {'length':  60, 'separation': 0.5, 'orthoStopDist':   0, 'allowedIds': [11] },
    "objectMarker":  {'length':  60, 'separation': 0.5, 'orthoStopDist':   0, 'allowedIds': [11] }}



# calibrateCamera
# createMarkers()
# calibration.createCharucoBoard()
# calibration.takeCalibrationPictures()
# calibration.calibrateCamera()

# calibration.calibrateCameraCharuco()
# exit()


arucoParams = aruco.DetectorParameters_create()
# this parameter creates a border around each each cell in the cell grid
arucoParams.perspectiveRemoveIgnoredMarginPerCell = 0.25     # default = 0.13
arucoParams.minDistanceToBorder = 2

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)

navManager = None
cap = None
lastImg = None


def lookForMarkers(camera, markerIds, camYaw):
    foundMarkers = []

    if camera == "EYE_CAM":
        img = config.eyecamImage

    elif camera == "CART_CAM":
        img = config.cartcamImage

    else:
        config.log(f"requested camera {camera} not handled in call")
        return []

    if img is None:
        config.log(f"missing image")
        return []

# check for marker
    foundIds, corners = findMarkers(img, show=False)

    if len(markerIds) > 0 and foundIds is None:
        config.log(f"none of the requested markers found in image")
        return []

    if foundIds is not None:
        #config.log(f"markers found: {foundIds}")
        for markerIndex, foundId in enumerate(foundIds):
            if len(markerIds) == 0 or foundId in markerIds:
                try:
                    markerInfo = calculateMarkerFacts(corners[markerIndex], foundId, camera)
                except Exception as e:
                    config.log(f"error in calculateMarkerFacts: {e}")
                    markerInfo = None
                if markerInfo is not None:
                    config.log(f"markerId: {markerInfo['markerId']}, distance: {markerInfo['distanceCamToMarker']}, angleToMarker: {markerInfo['angleToMarker']}, markerDegrees: {markerInfo['markerDegrees']}")
                    foundMarkers.append(markerInfo)

    return foundMarkers



# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.atan2(-R[1, 2], R[1, 1])
        y = np.atan2(-R[2, 0], sy)
        z = 0

    return np.array([y, x, z])


# calculate a cartYaw for cart to be positioned <distance> in front of the marker
def evalDegreesDistToCartTarget(degreesToMarker, distance, markerDegrees):
    config.log(
        f"evalDegreesDistToCartDestination degreesToMarker: {degreesToMarker:.0f}, distance: {distance:.0f}, markerDegrees: {markerDegrees:.0f}")

    # Position x,y of camera
    p1 = point2D(0, 0)

    # Center of Marker
    p2 = point2D(0, 0)
    p2.x = int(distance * np.cos(np.radians(degreesToMarker)))
    p2.y = int(distance * np.sin(np.radians(degreesToMarker)))
    # print(p2)

    # angle between marker and cart destination point (includes markerDegrees)
    beta = degreesToMarker - 90 + markerDegrees

    # cart destination point orthogonal in front of marker with offset
    p3 = point2D(0, 0)
    p3.x = int(p2.x + (distance * np.sin(np.radians(beta))))
    p3.y = int(p2.y - (distance * np.cos(np.radians(beta))))
    # print(p3)

    # angle to cart point in relation to degreesToMarker
    degreesToCartTarget = 180 + np.degrees(np.arctan(p3.y / p3.x))
    distToCartTarget = np.sqrt(np.power(p3.x, 2) + np.power(p3.y, 2))

    config.log(
        f"degreesToCartTarget: {degreesToCartTarget:.0f}, distToCartTarget {distToCartTarget:.0f}, markerPos: {p2}, cartTargetAngle: {beta:.0f}, cartTargetPos: {p3}")

    return degreesToCartTarget, distToCartTarget


def getAvgHue(img):
    # convert to hsv for color comparison
    imgH = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    avgHsvByCol = np.average(imgH, axis=0)
    avgHsv = np.average(avgHsvByCol, axis=0)
    return avgHsv[0]  # hue


# search for marker in frame
def findMarkers(img, show):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # aruco.detectMarkers() requires gray image

    # timestr = datetime.now().strftime("%Y_%m_%d-%H_%M_%S.%f")
    # cv2.imwrite("images/" + timestr + ".jpg", img)

    if show:
        cv2.imshow('ArucoServer', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    try:
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=arucoParams)  # Detect aruco
    except Exception as e:
        config.log(f"exception in detectMarkers: {e}")

    if ids is not None:  # if aruco marker(s) detected

        return ids, corners

    else:
        return None, None


def calculateMarkerFacts(corners, markerId, camera):
    """
    aruco.estimatePoseSingleMarkers looks to be a bit unprecise, use own calcs for distance and direction
    :param corners:
    :param markerId:
    :param camera:
    :return:
    """

    # marker = markerTypes[markerType=="dockingMarker"]
    marker = [m for m in markers if markerId in m.allowedIds]
    #config.log(f"markerId: {markerId}, markerSize: {marker[0].length}")

    if marker is None or len(marker) == 0:
        return None

    if camera == "CART_CAM":
        colAngle = CARTCAM_X_ANGLE / CARTCAM_X_RESOLUTION
        rowAngle = CARTCAM_Y_ANGLE / CARTCAM_Y_RESOLUTION
        imgXCenter = CARTCAM_X_RESOLUTION / 2
        vec = aruco.estimatePoseSingleMarkers(corners, marker[0].length, config.cartcamMatrix,
                                              config.cartcamDistortionCoeffs)  # For a single marker

    else:
        colAngle = EYECAM_X_ANGLE / EYECAM_X_RESOLUTION
        rowAngle = EYECAM_Y_ANGLE / EYECAM_Y_RESOLUTION
        imgXCenter = EYECAM_X_RESOLUTION / 2
        vec = aruco.estimatePoseSingleMarkers(corners, marker[0].length, config.eyecamMatrix,
                                              config.eyecamDistortionCoeffs)  # For a single marker

    # my markers are not rotated, use average of marker height on left and right side for distance calculation
    # corner indices are clockwise starting with topleft (second index)
    config.log(f"corners: {corners[0]}", publish=False)
    tl=0
    tr=1
    br=2
    bl=3
    col=0
    row=1
    centerCol = (corners[0][tr][col] + corners[0][tl][col]) / 2
    # eval left and right marker height in rows
    markerRowsLeft = corners[0][bl][row] - corners[0][tl][row]
    markerRowsRight = corners[0][br][row] - corners[0][tr][row]
    markerRows = (markerRowsLeft + markerRowsRight) / 2

    # eval the angle of the marker in the image using the vertical cam angle and resolution
    heightAngle = markerRows * rowAngle

    # using the known size of the marker the distance is adj=opp/tan
    # use abs value as eye cam delivers a rotated map
    distanceCamToMarker = abs(marker[0].length / np.tan(np.radians(heightAngle)))

    # use the markers center col to calc the angle to the marker
    angleToMarker = (imgXCenter - centerCol) * colAngle
    config.log(f"angleToMarker, centerCol: {centerCol}, offset: {imgXCenter - centerCol}, colAngle: {colAngle}")

    # eval the marker's yaw from the result of aruco.estimatePoseSingleMarkers
    rmat = cv2.Rodrigues(vec[0])[0]
    yrp = rotationMatrixToEulerAngles(rmat)

    # markerDegrees is 0 for an orthogonal view position,
    # negative for a viewpoint right of the marker
    # positive for a viewpoint left of the marker
    markerDegrees = float(-np.degrees(yrp[0]))  # ATTENTION, this is the yaw of the marker evaluated from the image

    # for distances > 1500 markerDegrees are not accurate, reduce value
    if distanceCamToMarker > 1500:
        config.log(f"corrected markerDegrees from {markerDegrees} to {markerDegrees/3} because of distance {distanceCamToMarker}")
        markerDegrees = round(markerDegrees / 3)

    config.log(f"markerId: {markerId}, distanceCamToMarker: {distanceCamToMarker:.0f}, angleToMarker: {angleToMarker:.2f}, markerDegrees: {markerDegrees:.2f}")

    return {'markerId': markerId,
            'distanceCamToMarker': int(distanceCamToMarker),
            'angleToMarker': round(angleToMarker),
            'markerDegrees': round(markerDegrees)}


def loadCalibration():
    data = np.load("C:/Projekte/InMoov/aruco/cartcamCalibration/calibration.npz")
    config.cartcamMatrix = data['cameraMatrix']
    config.cartcamDistortionCoeffs = data['distortionCoeffs'][0]

    data = np.load("C:/Projekte/InMoov/aruco/eyecamCalibration/calibration.npz")
    config.eyecamMatrix = data['cameraMatrix']
    config.eyecamDistortionCoeffs = data['distortionCoeffs'][0]

