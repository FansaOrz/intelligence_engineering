#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap
import math

#===============================================================================
def splitandmerge(points, split_thres=0.05, inter_thres=0.3, min_points=6, dist_thres=0.01, ang_thres=np.deg2rad(5)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    print split_thres, dist_thres
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    # return lines
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]
    
    # TODO: CODE HERE!!!

    # Check minimum number of points
    if last_pt - first_pt + 1 < min_points:
        return None

    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    a = y2 - y1
    b = x1 - x2
    c = y1*x2 - y2*x1
    

    # Distances of points to line (prelab question)
    #dist = np.zeros((1, last_pt - first_pt + 1))
    max_dist = 0
    for idx_pt in range(first_pt, last_pt+1): 
        x = points[0, idx_pt]
        y = points[1, idx_pt]
        dist = abs(a*x + b*y + c)/math.sqrt(math.pow(a,2) + math.pow(b,2))
        if dist > max_dist:
            max_dist = dist
            idx = idx_pt


    # Check split threshold
    if max_dist > split_thres:
        # print "111111111111111111"

        # Check sublines
        
        prev = split(points, split_thres, inter_thres, min_points, first_pt, idx)
        post = split(points, split_thres, inter_thres, min_points, idx, last_pt)
       
        # Return results of sublines
        if prev is not None and post is not None:
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # Do not need to split furthermore
    else:
        # print "222222222222222222222222"
        # Optional check interpoint distance
        for i in range(first_pt, last_pt):
            inter_dist = math.sqrt(math.pow(points[1,i+1]-points[1,i],2) + math.pow(points[0,i+1]-points[0,i],2))

            # Check interpoint distance threshold
            if inter_dist > inter_thres:
                # print "33333333333333333333"

                #Split line
                prev = split(points, split_thres, inter_thres, min_points, first_pt, i)
                post = split(points, split_thres, inter_thres, min_points, i+1, last_pt)

                # Return results of sublines
                if prev is not None and post is not None:
                    return np.vstack((prev, post))
                elif prev is not None:
                    return prev
                elif post is not None:
                    return post
                else:
                    return None
        
        # It is a good line
        return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    #return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None:
        return np.array([])
        
    # Check and merge similar consecutive lines
    i = 0
    while i in range(lines.shape[0]-1):
        
        # Line angles
        ang1 = math.atan2((lines[i,3]-lines[i,1]),(lines[i,2]-lines[i,0]))
        ang2 = math.atan2((lines[i+1,3]-lines[i+1,1]),(lines[i+1,2]-lines[i+1,0]))
        
        # Below thresholds?
        angdiff = abs(angle_wrap(ang1 - ang2))
        disdiff = math.sqrt(math.pow(lines[i+1,1]-lines[i,3],2) + math.pow(lines[i+1,0]-lines[i,2],2))
        if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            lines[i,:] = np.array([lines[i,0], lines[i,1], lines[i+1,2], lines[i+1,3]])
            
            # Delete unnecessary line
            lines = np.delete(lines, i+1, 0)
            
        # Nothing to merge
        else:
            i += 1
    print "============================================================"
    print lines.shape
    return lines
