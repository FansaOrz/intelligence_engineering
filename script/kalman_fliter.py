#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0]) ** 2
R = np.diag([1.0, math.radians(40.0)]) ** 2
dt = 0.1


def motion_model(x, u):
    B = np.matrix([[dt * math.cos(x[2, 0]), 0.0],
                   [dt * math.sin(x[2, 0]), 0.0],
                   [0.0, dt],
                   [1.0, 0.0]])
    x = x + B * u
    return x


# def observe_model(z):
#    H=
#    pass
# def JacoMo(xEst,u):
#    return jMo
# def JacoOb(xEst):
#    return jOb
def JacoMo(x, u):
    """
    Jacobian of Motion Model
    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.matrix([[1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
                    [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]])

    return jF


def JacoOb(x):
    # Jacobian of Observation Model
    jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, pEst, z, u):
    # yu ce predict
    xPre = motion_model(xEst, u)

    jMo = JacoMo(xEst, u)  # Jacobin of motion model
    jOb = JacoOb(xEst)
    pPre = jMo * pEst * jMo.T + Q
    s = jOb * pPre * jOb.T + R
    k = pPre * jOb.T * np.linalg.inv(s)
    # update estimate
    H = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    zPre = H * xPre
    xEst = xPre + k * (z - zPre)
    pEst = (np.eye(len(xEst)) - k * jOb) * pPre
    return xEst, pEst


def input_data(xTrue, xImu):
    u = np.matrix([1.0, 0.1]).T  # jiaosudu he xiansudu
    xTrue = motion_model(xTrue, u)
    Qsim = np.diag([0.5, 0.5]) ** 2
    Rsim = np.diag([1.0, math.radians(5.0)]) ** 2

    # GPS
    z = np.matrix([xTrue[0, 0] + np.random.randn() * Qsim[0, 0],
                   xTrue[1, 0] + np.random.randn() * Qsim[1, 1]]).T
    ud = np.matrix([u[0, 0] + np.random.randn() * Rsim[0, 0],
                    u[1, 0] + np.random.randn() * Rsim[1, 1]]).T
    xImu = motion_model(xImu, ud)
    return ud, z, xTrue, xImu


if __name__ == "__main__":
    xEst = np.matrix(np.zeros((4, 1)))
    xTrue = xEst
    xImu = xTrue
    pEst = np.eye(4)
    t = 0.1

    hTrue = xTrue
    hEst = xEst
    hz = np.zeros((2, 1))
    hImu = xImu
    for i in range(1000):
        u, z, xTrue, xImu = input_data(xTrue, xImu)

        xEst, pEst = ekf_estimation(xEst, pEst, z, u)
        hTrue = np.hstack((hTrue, xTrue))
        hEst = np.hstack((hEst, xEst))
        hImu = np.hstack((hImu, xImu))
        hz = np.hstack((hz, z))
        # plot
        plt.cla()
        plt.plot(hz[0, :], hz[1, :], ".g")
        plt.plot(np.array(hTrue[0, :]).flatten(),
                 np.array(hTrue[1, :]).flatten(), "-b")
        plt.plot(np.array(hEst[0, :]).flatten(),
                 np.array(hEst[1, :]).flatten(), "-r")
        plt.plot(np.array(hImu[0, :]).flatten(),
                 np.array(hImu[1, :]).flatten(), "-k")
        plt.pause(0.001)
