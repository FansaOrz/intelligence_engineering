#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import cv2
import numpy as np

def Hough(img):

    house = cv2.cvtColor(img, cv2.COLOR_BGR2BGRA)
    edges = cv2.Canny(house, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, -10)
    # cv2.imshow('img', edges)
    # cv2.waitKey()

    lines = lines[:, 0, :]


    for x1, y1, x2, y2 in lines:
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.imshow('img', img)
        cv2.waitKey()

    print "lines", lines
    # for rho, theta in lines:
    #     a = np.cos(theta)
    #     b = np.sin(theta)
    #     # 从图b中可以看出x0 = rho x cos(theta)
    #     #               y0 = rho x sin(theta)
    #     x0 = a * rho
    #     y0 = b * rho
    #     # 由参数空间向实际坐标点转换
    #     x1 = int(x0 + 1000 * (-b))
    #     y1 = int(y0 + 1000 * a)
    #     x2 = int(x0 - 1000 * (-b))
    #     y2 = int(y0 - 1000 * a)
    #     cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 1)
    cv2.imshow('img', img)
    cv2.waitKey()

if __name__ == '__main__':
    img = cv2.imread("/home/fansa/loca.pgm")

    Hough(img)