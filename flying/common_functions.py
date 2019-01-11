import os
from os import listdir
from os.path import isfile, join
import tempfile
import pprint
import time
import math
import time
import re

import sys
if "E:\\git\\AirSim\\PythonClient" not in sys.path:
    sys.path.append("E:\\git\\AirSim\\PythonClient")
    print(sys.path)
    
print('--', sys.path)
import airsim
from airsim.types import *

from common_types import *


###########################

def getPichRollYawClient(client):
    return airsim.to_eularian_angles(client.simGetGroundTruthKinematics().orientation)


def convertCoordFromUE4toNED(ue4Point, startPoint):
    
    x = (ue4Point.x-startPoint.x)/100.0
    y = (ue4Point.y-startPoint.y)/100.0
    z = (startPoint.z - ue4Point.z)/100.0
  
    return Location(x,y,z, ue4Point.yaw)


# def getNewPointToCapturePicture(toPoint, yaw, direction, pos, velocity, extra_high = 0):
#     #get direction vector and calculate cos, sin of the angle between the direction vector and Oy
#     angle = yaw
#     cosA = math.cos(angle*math.pi/180.0)
#     sinA = math.sin(angle*math.pi/180.0)
    
#     x = toPoint.x
#     y = toPoint.y
#     z = toPoint.z
    
#     # TODO clean up below if, else
#     x0,y0 = (0,0)
#     if (pos == SOUTH_EAST_POS):
#         x0,y0 = (-INSPECTION_X, INSPECTION_Y)
#     elif (pos == NORTH_EAST_POS):
#         x0,y0 = (INSPECTION_X, INSPECTION_Y)
#     elif (pos == NORTH_WEST_POS):
#         x0,y0 = (INSPECTION_X, -INSPECTION_Y)
#     elif (pos == SOUTH_WEST_POS):
#         x0,y0 = (-INSPECTION_X, -INSPECTION_Y)
#     elif (pos == NORTH_POS):
#         x0 = INSPECTION_X
#     elif (pos == SOUTH_POS):
#         x0 = -INSPECTION_X

#     X = x0*cosA - y0*sinA
#     Y = y0*cosA + x0*sinA
#     x_expected = x + X
#     y_expected = y + Y
#     z_expected = z-INSPECTION_Z - extra_high
#     expectedLoc = Vector3r(x_expected, y_expected, z_expected)
#     print('expectedLoc', expectedLoc)

#     # this is because the velocity affect to the location
#     stop_distance = 0.03
#     if (velocity == NAVIGATION_SPEED):
#         stop_distance = 1
    
#     if (direction == NORTH_SOUTH_DRT):
#         x0 = x0 + stop_distance
#     elif (direction == SOUTH_NORTH_DRT):
#         x0 = x0 - stop_distance
#     elif (direction == EAST_WEST_DRT):
#         y0 = y0 + stop_distance
#     elif (direction == WEST_EAST_DRT):
#         y0 = y0 - stop_distance
    
    
#     X = x0*cosA - y0*sinA
#     Y = y0*cosA + x0*sinA
#     x_stop = x + X
#     y_stop = y + Y
#     z_stop = z-INSPECTION_Z - extra_high
#     stopLoc = Vector3r(x_stop, y_stop, z_stop)
    
#     return expectedLoc, stopLoc

def getNewPointToCapturePicture1(toPoint, currentPoint, pos, velocity, extra_high = 0, extra_width=0):
    #get direction vector and calculate cos, sin of the angle between the direction vector and Oy
    angle = toPoint.yaw
    cosA = math.cos(angle*math.pi/180.0)
    sinA = math.sin(angle*math.pi/180.0)
    
    x = toPoint.x
    y = toPoint.y
    z = toPoint.z
    
    # TODO clean up below if, else
    x0,y0 = (0,0)
    if (pos == NORTH_POS):
        x0 = INSPECTION_X + extra_width
    elif (pos == SOUTH_POS):
        x0 = -(INSPECTION_X + extra_width)
    elif (pos == SOUTH_EAST_POS):
        x0,y0 = (-(INSPECTION_X + extra_width), INSPECTION_Y)
    elif (pos == NORTH_EAST_POS):
        x0,y0 = (INSPECTION_X+ extra_width, INSPECTION_Y)
    elif (pos == NORTH_WEST_POS):
        x0,y0 = (INSPECTION_X+ extra_width, -INSPECTION_Y)
    elif (pos == SOUTH_WEST_POS):
        x0,y0 = (-(INSPECTION_X + extra_width), -INSPECTION_Y)
    

    X = x0*cosA - y0*sinA
    Y = y0*cosA + x0*sinA
    x_expected = x + X
    y_expected = y + Y
    z_expected = z-INSPECTION_Z - extra_high
    expectedLoc = Vector3r(x_expected, y_expected, z_expected)
    #print('expectedLoc', expectedLoc)
    
    

    # this is needed because the momentum make the drone not able to stop right away, so we need to stop in advance a bit
    # https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
    distance = currentPoint.distance_to(expectedLoc)
    print('distance: ', distance)
    stop_distance = STOP_DISTANCE 
    t = stop_distance / distance 
    # calculate stop Location
    x_stop = (1-t)*x_expected + t*currentPoint.x_val
    y_stop = (1-t)*y_expected + t*currentPoint.y_val
    z_stop = (1-t)*z_expected + t*currentPoint.z_val
    
    stopLoc = Vector3r(x_stop, y_stop, z_stop)
    #print ('stopLoc: ', stopLoc)
    
    final_distance = FINAL_DISTANCE 
    t = final_distance / distance 
    # calculate stop Location
    x_stop = (1-t)*x_expected + t*currentPoint.x_val
    y_stop = (1-t)*y_expected + t*currentPoint.y_val
    z_stop = (1-t)*z_expected + t*currentPoint.z_val
    
    finalLoc = Vector3r(x_stop, y_stop, z_stop)
    #print ('finalLoc: ', finalLoc)
    
    midLoc1 = None
    midLoc2 = None
    if (velocity == NAVIGATION_SPEED):
        x_mid1 = 0.35*x_expected + 0.65*currentPoint.x_val
        y_mid1 = 0.35*y_expected + 0.65*currentPoint.y_val
        z_mid1 = 0.35*z_expected + 0.65*currentPoint.z_val + 1.2
        
        x_mid2 = 0.62*x_expected + 0.38*currentPoint.x_val
        y_mid2 = 0.62*y_expected + 0.38*currentPoint.y_val
        z_mid2 = 0.62*z_expected + 0.38*currentPoint.z_val + 1.2
    
        midLoc1 = Vector3r(x_mid1, y_mid1, z_mid1)
        midLoc2 = Vector3r(x_mid2, y_mid2, z_mid2)
    
    return finalLoc, stopLoc, midLoc1, midLoc2


def calAngleBetween2Locs(curLoc, nextLoc):
    xy = (nextLoc.x - curLoc.x, nextLoc.y - curLoc.y)
    rad = math.atan2(xy[1], xy[0])
    degree = rad*180.0/math.pi
    
    return degree


# def calAngleOfMastShouldBeByDroneYaw(drone_yaw):
#     angle = 0
#     if drone_yaw > 0:
#         if drone_yaw < 90:
#             return (drone_yaw - 90)
#         else:
#             return (drone_yaw - 270 )
#     else:
#         if drone_yaw > -90:
#             return (-90 + drone_yaw)
#         else:
#             return (90 + drone_yaw)
        
        
def moveToPosition(client, toPoint, yaw=0, pos=0, velocity=INSPECTION_SPEED, extra_high=0, extra_width=0):   
    
    currentPoint = client.simGetGroundTruthKinematics().position
    
    finalPoint, stopPoint, midLoc1, midLoc2 = getNewPointToCapturePicture1(toPoint, currentPoint, pos, velocity, extra_high, extra_width) 

    if velocity == NAVIGATION_SPEED:
        (x,y,z) = (stopPoint.x_val, stopPoint.y_val, stopPoint.z_val)
        (x_m1,y_m1,z_m1) = (midLoc1.x_val, midLoc1.y_val, midLoc1.z_val)
        (x_m2,y_m2,z_m2) = (midLoc2.x_val, midLoc2.y_val, midLoc2.z_val)
        client.moveToPositionAsync(x_m1,y_m1,z_m1, velocity, drivetrain=DrivetrainType.ForwardOnly, yaw_mode=YawMode(False,0)).join()
        client.moveToPositionAsync(x_m2,y_m2,z_m2, velocity, drivetrain=DrivetrainType.ForwardOnly, yaw_mode=YawMode(False,0)).join()
        client.moveToPositionAsync(x,y,z, velocity, drivetrain=DrivetrainType.ForwardOnly, yaw_mode=YawMode(False,0)).join()
    
    else:
        (x1,y1,z1) = (finalPoint.x_val, finalPoint.y_val, finalPoint.z_val)
        angle = 0
        if yaw != 0:
            #get current Location
            currentPoint = client.simGetGroundTruthKinematics().position
            currentLoc = Location(currentPoint.x_val, currentPoint.y_val, currentPoint.z_val)            
            
            nextLoc = Location(x1,y1,z1)
            yaw2 = calAngleBetween2Locs(currentLoc, nextLoc)        
            
            angle = yaw - yaw2
            print ('big angle, turning left {} ', angle)
        
        client.moveToPositionAsync(x1,y1,z1, INSPECTION_SPEED, drivetrain=DrivetrainType.ForwardOnly, yaw_mode=YawMode(False,angle)).join()
        
    #compare current location and finalPoint location
    currentPoint = client.simGetGroundTruthKinematics().position
    #print('finally, the drone is at ', currentPoint)
    distance = currentPoint.distance_to(finalPoint)
    print('distance to finalPoint destination is {}'.format(distance))
    
    
    
#     if velocity == NAVIGATION_SPEED:
#         for i in range(2):
#             # compare current location and finalPoint location
#             currentPoint = client.simGetGroundTruthKinematics().position
#             distance = currentPoint.distance_to(finalPoint)

#             if distance > 0.1: 
                
#                 #print('distance to final destination is {}'.format(distance))
#                 (x1,y1,z1) = finalPoint.x_val, finalPoint.y_val, finalPoint.z_val
#                 client.moveToPositionAsync(x1,y1,z1, INSPECTION_SPEED, drivetrain=DrivetrainType.ForwardOnly, yaw_mode=YawMode(False,0)).join()
#             else:
#                 break
    
        
#     if velocity == NAVIGATION_SPEED:
#         #compare current location and finalPoint location
#         currentPoint = client.simGetGroundTruthKinematics().position
#         #print('finally, the drone is at ', currentPoint)
#         distance = currentPoint.distance_to(finalPoint)
#         print('distance to finalPoint destination is {}'.format(distance))
        
#         client.hoverAsync().join()
#         time.sleep(3)
            
    
def rotate(client, degree, margin=0.01, back_to_initial_yaw=False, taking_pictures = False, imgDir=None):
    _, _, initial_yaw = getPichRollYawClient(client)   
    # try maximum 10 times to rotate to gain the best accurarcy of rotation
    client.rotateToYawAsync(degree, 5, margin).join()
    _, _, yaw = getPichRollYawClient(client)
    
    new_yaw = yaw*180.0/math.pi
    print('yaw after rotation: ',new_yaw)
                 
    #client.hoverAsync().join()
    # sleep to simulate taking pictures
    if (taking_pictures == True):
        captureImagesByAllCameras(imgDir)
        time.sleep(4)
   
    if (back_to_initial_yaw):
        initial_yaw = initial_yaw*180.0/math.pi
        print("initial_yaw", initial_yaw)
        client.rotateToYawAsync(initial_yaw, 10, margin).join()
    return


#########
def stringSplitByNumbers(x):
    r = re.compile('(\d+)')
    l = r.split(x)
    return [int(y) if y.isdigit() else y for y in l]

def readMastLocations(mastDir, startPoint, mastPrefix=''):
    mastFiles = [f for f in sorted(listdir(mastDir), key=stringSplitByNumbers) if isfile(join(mastDir, f))]
    masts = []
    for f in mastFiles:
        if f.find(mastPrefix) != -1: #file contains the expected prefix
            print(f)
            file = join(mastDir, f)
            #print(file)
            oFile = open(file,"r")
            locString = oFile.readline() # location
            #print(locString)
            pyrString = oFile.readline() #pitch, roll, yaw
            #print(pryString)

            matchObj = re.match(r"X=(.*) Y=(.*) Z=(.*)\n", locString)
            if matchObj:
                x = float(matchObj.group(1))
                y = float(matchObj.group(2))
                z = float(matchObj.group(3)) 
                #print(x,y,z)

            matchObj = re.match(r"P=(.*) Y=(.*) R=(.*)\n", pyrString)
            if matchObj:
                pitch = float(matchObj.group(1))
                yaw = float(matchObj.group(2))
                if yaw > 0 :
                    yaw = yaw - 360
                roll = float(matchObj.group(3))
                #print(p,y,r)

            m = Location(x,y,z,yaw)
            # convert to NED
            nedPoint = convertCoordFromUE4toNED(m, startPoint)
            masts.append(nedPoint)

            oFile.close()
        
    return masts

def readStartLocation(playerStartDir):
    playerStartFilePath = join(playerStartDir, 'PlayerStart')
    oFile = open(playerStartFilePath,"r")
    locString = oFile.readline() # location
    matchObj = re.match(r"X=(.*) Y=(.*) Z=(.*)", locString)
    if matchObj:
        x = float(matchObj.group(1))
        y = float(matchObj.group(2))
        z = float(matchObj.group(3)) 
        startPoint = Location(x,y,z,0.0)
        
        return startPoint
    

class Location:
    def __init__(self, x, y, z, yaw=None, big_angle=0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.big_angle = big_angle
    def __str__(self):
        return 'XYZ({} {} {}) Yaw({})'.format(self.x, self.y, self.z, self.yaw)