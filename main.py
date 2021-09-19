
from djitellopy import Tello
import cv2, math, time
import os
from os import chdir
import threading
from threading import Thread
from TelloProject import  *
from time import sleep



def startORB():
    chdir("/home/george/ORB_SLAM2")
    os.system("./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml")



def FindAxis(angle, x, y):
    if x > 0 > y:
        angle += 90
        return angle
    if x < 0 and y < 0:
        angle += 180
        return angle
    if x < 0 < y:
        angle += 270
        return angle

def CalcAngle(x, y):
    angle = 90 - int(degrees(math.tan(float(abs(y) / abs(x)))))
    return FindAxis(angle, x, y)


def NavigateToExit(x, y, DistanceToExitPoint):
    tello = Tello()
    tello.connect()
    tello.takeoff()
    angle = int(CalcAngle(x, y))
    print(angle)
    forwardistance = int(DistanceToExitPoint * 100)
    if forwardistance < 100:
        forwardistance = forwardistance * 1000
    if forwardistance < 1000:
        forwardistance = forwardistance * 100
    tello.rotate_clockwise(angle)
    print("distance ", forwardistance)
    while forwardistance > 0:
        tello.move_forward(500)
        forwardistance -= 500
    tello.end()


def scan():
    tello = Tello()
    tello.connect()
    tello.speed = 50
    tello.streamoff()
    tello.streamon()
    thr = Thread(target=startORB)
    thr.start()
    tello.takeoff()
    print(tello.get_height())
    # if you are startiing from floor or dick
    #tello.move_up(int(height - tello.get_height()))
    angle = 0
    sleep(3)
    while angle <= (360):
        print (angle)
        # roatat the tello in 15 angles
        tello.rotate_clockwise(15)
        sleep(3)
        # move up and down for better scanning
        tello.move_up(30)
        sleep(3)
        tello.move_down(30)
        angle += 15
        sleep(3)
        # if the tello miss a down command make sure not get the celling
        if(tello.get_height() > 70 ):
            tello.move_down(20)
            sleep(1)
    tello.streamoff()
    tello.end()
    thr.join()





if __name__ == '__main__':
    scan()
    ExitPointX, ExitPointY, DistanceToExitPoint = GetPointCloud()
    time.sleep(3)
    NavigateToExit(ExitPointX, ExitPointY, DistanceToExitPoint)