
import time
import config

class ImageRequest:

    def __init__(self, request, timeout):
        self.camType = request['camType']
        self.responsePending = True
        self.requestSuccessful = False
        self.timeout = time.time() + timeout
        request['imageId'] = config.imageId
        config.imageId += 1
        config.marvinShares.imageProcessingRequestQueue.put(request)

    def isImageAvailable(self) -> bool:
        while self.responsePending and time.time() < self.timeout:
            #config.log(f"isImageAvailable, wait for result or timeout")
            time.sleep(0.1)
        #config.log(f"{self.responsePending=}, {self.requestSuccessful=}, {time.time() - self.timeout}")
        if not self.requestSuccessful:
            config.log(f"isImageAvailable, imageRequest failed, {self.camType=}, {self.requestSuccessful=} ")
        return self.requestSuccessful

    def setFailed(self):
        config.log(f"ImageRequest {self.camType}, setFailed called")
        self.requestSuccessful = False
        self.responsePending = False

    def setSuccess(self):
        config.log(f"ImageRequest {self.camType}, setSuccess called")
        self.requestSuccessful = True
        self.responsePending = False

    def waitForResult(self):
        if self.responsePending and time.time() < self.timeout:
            time.sleep(0.1)
