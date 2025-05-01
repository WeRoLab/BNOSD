"""
PyTeapot module for drawing rotating cube using OpenGL as per
quaternion or yaw, pitch, roll angles received over serial port.
"""

import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from postprocess import IMUData
import numpy as np
from scipy.spatial.transform import Rotation as R
import os


import serial

ser = serial.Serial('/dev/tty.usbmodem1301', 115200)

foot_transform = None
shank_transform = None
def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 640), video_flags)
    pygame.display.set_caption("IMU orientation visualization")
    resizewin(640, 640)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        output = read_data()
        if output is None:
            continue
        datapoint = output
        draw(*datapoint.IMU_1.as_quat(), *datapoint.IMU_2.as_quat())
        pygame.display.flip()
        frames += 1
    print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)



def read_data():
    global foot_transform, shank_transform
    ser.reset_input_buffer()
    line = ser.readline().decode('UTF-8').replace('\n', '')
    # print(line)
    os.system('clear')
    # print('\n'.join(line.split(',')))
    if len(line.split(',')) != 16:
        return None
    cycle_count, time_elapsed, qr, qi, qj, qk, pr, pi, pj, pk, qa, qb, qc, pa, pb, pc = line.split(',')
    datapoint = IMUData(cycle_count, [float(qr), float(qi), float(qj), float(qk)], [float(pr), float(pi), float(pj), float(pk)])
    # if foot_transform is None:
    #     sagittal_foot = R.from_euler('xyz', [0, 0, 0], degrees=True)
    #     sagittal_shank = R.from_euler('xyz', [0, 0, 0], degrees=True)
    #     foot_transform = sagittal_foot * datapoint.IMU_1.inv()
    #     shank_transform = sagittal_shank * datapoint.IMU_2.inv()
    # datapoint.calibrate(foot_transform, shank_transform)
    # print(datapoint.IMU_1.as_euler('xyz', degrees=True)[1] + datapoint.IMU_2.as_euler('xyz', degrees=True)[1])
    # print(datapoint.IMU_1.as_euler('xyz', degrees=True))
    # print(datapoint.IMU_2.as_euler('xyz', degrees=True))
    r_ab = np.dot(datapoint.IMU_1.as_matrix().T,datapoint.IMU_2.as_matrix())
    print(r_ab)
    print(np.arccos((np.trace(r_ab)-1)/2)*360/2/np.pi)
    return datapoint


def draw(pr, pi, pj, pk, qr, qi, qj, qk):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(2, 0.0, -10)

    glRotatef(2 * math.acos(pr) * 180.00 / math.pi, -1 * pi, pk, pj)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()

    # paint second rectangle

    glLoadIdentity()
    glTranslatef(-2, 0.0, -10)
    glRotatef(2 * math.acos(qr) * 180.00 / math.pi, -1 * qi, qk, qj)
    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()



def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def quat_to_ypr(q):
    yaw = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
    roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    pitch *= 180.0 / math.pi
    yaw *= 180.0 / math.pi
    yaw -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll *= 180.0 / math.pi
    return [yaw, pitch, roll]


if __name__ == '__main__':
    main()
