import math



class gpsCoordinate:

    def __init__(self, x, y, z) -> None:
        self.x = x
        self.y = y
        self.z = z

class Target:

    def __init__(self) -> None:
        self.middleEdge = 0
        self.bottomEdge = 0
        self.upperEdge = 0
        self.UpperDistance = 0
        self.middleDistance = 0
        self.bottomDistance = 0
        self.referenceBeam = 0
        self.TotalDistance = 0
        self.coordinate = gpsCoordinate(0,0,0)
        
    def setCoordinates(self, x, y, z):
        self.coordinate.x = x
        self.coordinate.y = y
        self.coordinate.z = z

    def MeasureDistance(self, ratio, camera, altitude=60):
        self.middleEdge = altitude*(1/cosd(camera.camAngle))
        self.bottomEdge = altitude*(1/cosd(camera.CamFoV/2 - camera.camAngle))
        self.upperEdge  = altitude*(1/cosd(camera.CamFoV/2 + camera.camAngle))
        self.UpperDistance = math.sqrt(self.upperEdge**2 - altitude**2)
        self.middleDistance = math.sqrt(self.middleEdge**2 - altitude**2)
        self.bottomDistance = math.sqrt(self.bottomEdge**2 - altitude**2)
        self.referenceBeam = (sind(camera.CamFoV/2))*self.bottomEdge 
        self.TotalDistance = tand(camera.CamFoV*(ratio - 0.5) + camera.camAngle)*altitude
        return self

class Camera:

    def __init__(self, CamFoV, CamAngle) -> None:
        self.CamFov = CamFoV
        self.CamAngle = CamAngle
        self.viewingAngle = CamAngle
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
    
    def setAngles(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
    
    def updateViewingAngle(self):
        self.viewingAngle = self.CamAngle + self.pitch

    def getViewingAngle(self):
        return self.viewingAngle


class Plane:

    def __init__(self) -> None:
        self.altitude = 0
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.coordinate = gpsCoordinate(0,0,0)

    def setCoordinates(self, x, y, z):
        self.coordinate.x = x
        self.coordinate.y = y
        self.coordinate.z = z        

    def setAltitude(self, altitude) -> None:
        self.altitude = altitude

    def setPitch(self, pitch) -> None:
        self.pitch = pitch
        
    def setYaw(self, yaw) -> None:
        self.yaw = yaw

    def setRoll(self, roll) -> None:
        self.roll = roll

    def getAltitude(self):
        return self.altitude

    def getPitch(self):
        return self.pitch
        
    def getYaw(self):
        return self.yaw

    def getRoll(self):
        return self.roll



def sind(degree):
    return math.sin(math.radians(degree))
    
def cosd(degree):
    return math.cos(math.radians(degree))

def tand(degree):
    return math.tan(math.radians(degree))
    




