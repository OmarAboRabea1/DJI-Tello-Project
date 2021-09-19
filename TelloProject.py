import math
from math import sqrt, tan, degrees
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from sklearn.cluster import KMeans
import cv2
from djitellopy import Tello
from sklearn.neighbors import NearestNeighbors
import scipy.cluster.hierarchy as hcluster


# split points that have at least NeighborsNumber at radius with NearestNeighbors Knn algorithm
def findNeighbors(points, NeighborsNumber = 15, radius = 0.1, plt = None, color = 'black', marker="*",  linewidths=0.5):
    InlinerPointsXZ= []
    OutLinerPointsXZ = []
    xi = []
    yi = []
    xij = []
    yij = []
    knn = NearestNeighbors(n_neighbors=10)
    knn.fit(points)
    D, N = knn.radius_neighbors(points, radius=radius, return_distance=True, sort_results=True)
    for input_text, distances, neighbors in zip(points, D, N):
        if len(neighbors) > NeighborsNumber:
            tempInlinerPoint = []
            tempInlinerPoint.append(input_text[0])
            tempInlinerPoint.append(input_text[1])
            xi.append(input_text[0])
            yi.append(input_text[1])
            InlinerPointsXZ.append(tempInlinerPoint)
        else:
            tempOutlimerPoint = []
            tempOutlimerPoint.append(input_text[0])
            tempOutlimerPoint.append(input_text[1])
            xij.append(input_text[0])
            yij.append(input_text[1])
            OutLinerPointsXZ.append(tempOutlimerPoint)
    #plt.scatter(xi, yi, marker=marker, linewidths=linewidths, color= color)
    #plt.show()
    return InlinerPointsXZ, OutLinerPointsXZ, plt

# split points in x,y,z lists and points in 2d (x, z)
def prepareData(pointsIn3):
    length = len(pointsIn3)
    pointsInXZ = []
    x = np.zeros(length)
    y = np.zeros(length)
    z = np.zeros(length)
    for i in range(length):
        pointsInXZ.append([])
        for j in range(2):
            pointsInXZ[i].append(0)
    for i in range(length):
        pointsInXZ[i][0] = pointsIn3[i][0]
        x[i] = pointsIn3[i][0]
        pointsInXZ[i][1] = pointsIn3[i][2]
        y[i] = pointsIn3[i][1]
        z[i] = pointsIn3[i][2]
    return pointsIn3, pointsInXZ, x, y, z

# open csv file and put in list which every cell conatins point in 3d (x,y,z)
def openFile():
    path = '/tmp/pointData1.csv'
    file = open(path, 'r')
    pointsIn3 = []
    for l in file:
        line = l.strip().split(',')
        curentPoint = []
        curentPoint.append(float(line[0]))
        curentPoint.append(float(line[1]))
        curentPoint.append(float(line[2]))
        pointsIn3.append(curentPoint)
    return prepareData(pointsIn3)


# find the angles of the rectangle by calculating the Minimum and Maximum x,y from the points
def FindRectangleCoordinates(InlinerPointsXZ):
    minimumX = InlinerPointsXZ[0][0]
    minimumY = InlinerPointsXZ[0][1]
    maximumX = InlinerPointsXZ[0][0]
    maximumY = InlinerPointsXZ[0][1]
    for i in InlinerPointsXZ:
        minimumX = min(minimumX, i[0])
        minimumY = min(minimumY, i[1])
        maximumX = max(maximumX, i[0])
        maximumY = max(maximumY, i[1])
    width = maximumX - minimumX
    height = maximumY - minimumY
    head = []
    head.append(minimumX)
    head.append(minimumY)
    return head, width, height, minimumX , minimumY , maximumX , maximumY


# find the distance between the point and the nearst point in the rectangle
def DistanceFromRectangle(x, y, minimumX , minimumY , maximumX , maximumY):
    if x < minimumX:
        if y < minimumY:
            return math.hypot(x-minimumX, y- minimumY)
        if y<= maximumY:
            return minimumX - x
        return math.hypot(minimumX - x, maximumY - y)
    elif x <= maximumX:
        if y < minimumY:
            return minimumY - y
        if y <= maximumY:
            return 0
        return y - maximumY
    else:
        if y < minimumY:
            return math.hypot(maximumX - x, minimumY - y)
        if y <= maximumY:
            return x-maximumX
        return  math.hypot(maximumX - x, maximumY - y)



# return the points that outside the rectangle
def RelventPointsToClusters(pointsInXZ, minimumX , minimumY , maximumX , maximumY):
    XoutBox = []
    YoutBox = []
    XYoutBox = []
    for point in pointsInXZ:
        if (point[0] < minimumX or point[0]>maximumX or point[1] > maximumY or point[1]<  minimumY):
            XoutBox.append(point[0])
            YoutBox.append(point[1])
            temp = []
            temp.append(point[0])
            temp.append(point[1])
            XYoutBox.append(temp)
    return XoutBox, YoutBox, XYoutBox


def moveToExit1( x, y, maxDistance):

    angle = 90 - int(degrees(math.tan(float(abs(y)/ abs(x)))))
    print (angle)
    if x > 0 > y:
        angle += 90
    elif x < 0 and y < 0:
        angle += 180
    elif x < 0 < y:
        angle += 270
    print(angle)
    #
    tello = Tello()
    tello.connect()
    tello.speed = 30
    tello.streamoff()
    tello.streamon()
    tello.takeoff()
    #
    tello.rotate_clockwise(angle)
    # 1 unit in ORB_SLAM2 is about 160cm in real life
    distance = int(maxDistance * 160)
    if (distance < 100):
        distance = distance * 100
    else:
        distance = distance * 100
    print("distance ", distance)
    while distance > 500:
        tello.move_forward(500)
        distance -= 500
    tello.move_forward(distance)


# find 5 cluster for the out box points using Kmeans algorithm
def FindKmeans(XYoutBox):
    kmeans = KMeans(n_clusters=5)
    kmeans.fit(XYoutBox)
    clusters = kmeans.cluster_centers_
    return clusters


# check which point are the farest
def FindMostFar(clusters,  minimumX , minimumY , maximumX , maximumY):
    DistanceToExitPoint = 0.0
    for i in range(len(clusters)):
        plt.scatter(clusters[i, 0], clusters[i, 1], marker='*', s=500, color='black')
        CurrentDistance = DistanceFromRectangle(clusters[i, 0], clusters[i, 1], minimumX , minimumY , maximumX , maximumY)
        if CurrentDistance > DistanceToExitPoint:
            DistanceToExitPoint = CurrentDistance
            ExitPointX = clusters[i, 0]
            ExitPointY = clusters[i, 1]
    return ExitPointX, ExitPointY, DistanceToExitPoint


# for gui use
def makeScatter(x, y, pltt = None, marker='*', s=30, color='blue'):
    if(pltt == None):
        plt.scatter(x, y, marker="*", s = s, color = color)
        return plt
    pltt.scatter(x, y, marker=marker, s = s,color = color)
    return pltt


def DrawRectangle(head, width, height):
    rect = patches.Rectangle(head, width, height, fill=False, color="black", linewidth=5)
    return rect

# get the data in  order to make the work easier
def GetPointCloud():
    pointsIn3, pointsInXZ, x, y, z = openFile()
    #  Get the points with X neighbors in Y radius to keep only the boarders
    InlinerPointsXZ, OutLinerPointsXZ , pltt = findNeighbors(pointsInXZ, 15, 0.1, plt, 'black', marker="*",  linewidths=0.5)
    InlinerPointsXZ, OutLinerPointsXZ , pltt = findNeighbors(InlinerPointsXZ, 100, 0.12, pltt, 'red', marker="+",  linewidths=2)
    # find the rectangle for boarder points
    head, width, height,  minimumX , minimumY , maximumX , maximumY = FindRectangleCoordinates(InlinerPointsXZ)
    #rect = DrawRectangle(head, width, height)

    # get points outside the box
    XoutBox, YoutBox, XYoutBox = RelventPointsToClusters(pointsInXZ, minimumX , minimumY , maximumX , maximumY)
    # make average of groups for point outside the box
    clusters = FindKmeans(XYoutBox)
    ExitPointX, ExitPointY, DistanceToExitPoint = FindMostFar(clusters, minimumX, minimumY, maximumX, maximumY)

    return ExitPointX, ExitPointY, DistanceToExitPoint


#ExitPointX, ExitPointY, DistanceToExitPoint = GetPointCloud()
#moveToExit1( ExitPointX, ExitPointY, DistanceToExitPoint)