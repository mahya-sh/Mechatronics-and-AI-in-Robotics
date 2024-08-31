
import time
import serial
import numpy as np
from numpy import sin, cos
from vpython import *


def multiply_matrix(A, B):
    C = np.zeros((A.shape[0], B.shape[1]), dtype=float)
    if A.shape[1] == B.shape[0]:
        rows = A.shape[0]
        cols = B.shape[1]
        for row in range(rows):
            for col in range(cols):
                for elt in range(len(B)):
                    C[row, col] += A[row, elt] * B[elt, col]
        return C
    else:
        return "Sorry, cannot multiply A and B."


def vect(A):
    result = np.zeros((3, 1))
    result[0][0] = 0.5 * (A[2][1] - A[1][2])
    result[1][0] = 0.5 * (A[0][2] - A[2][0])
    result[2][0] = 0.5 * (A[1][0] - A[0][1])
    return result


def Q_1(roll, pitch, yaw):
    Q_x = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    Q_y = np.array([[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]])
    Q_z = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    temp = multiply_matrix(Q_z, Q_y)
    Q = multiply_matrix(temp, Q_x)
    trace = np.matrix.trace(Q)
    Vect = vect(Q)
    phi = np.arccos((-1 + trace) / 2)
    e = Vect / sin(phi)
    return e, phi


arduinoData = serial.Serial('com9', 115200)
time.sleep(1)
toRad = np.pi / 180.0
toDeg = 1 / toRad
# Simulate your object using VPython
while True:
    while arduinoData.inWaiting() == 0:
        pass
    dataPacket = arduinoData.readline()
    try:
        dataPacket = str(dataPacket, 'utf-8')
        splitPacket = dataPacket.split(",")
       # roll = float(splitPacket[0]) * toRad
       # pitch = float(splitPacket[1]) * toRad
       # yaw = float(splitPacket[2]) * toRad
        #print(roll * toDeg, pitch * toDeg, yaw * toDeg)
        #e, phi = Q1(roll, pitch, yaw)
        #print('\n\ne = \n', e, '\nphi = ', phi*toDeg)
        r0 = float(splitPacket[0])
        r1 = float(splitPacket[1])
        r2 = float(splitPacket[2])
        r3 = float(splitPacket[3])
        print(r0, r1, r2, r3)
    except:
        pass

    # Change the attributes of your object to syncronize it with real time motions

