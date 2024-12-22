#!/usr/bin/env python3
#
# @brief Python class BoardVision for detecting board contours and computing waypoints
#    
# Copyright (C) 2024, Frank Traenkle, Hochschule Heilbronn
#  
# This file is part of MAD.
# MAD is free software: you can redistribute it and/or modify it under the terms 
# of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
# MAD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY 
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
# You should have received a copy of the GNU General Public License along with MAD.
# If not, see <https://www.gnu.org/licenses/>.
#

import cv2
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import matplotlib.pyplot as plt

DEBUG=False

class BoardVision:
    def __init__(self, image, numPoints=100):
        self.image = image
        self.numPoints = numPoints
        self.errtxt = ''

    def showImage(self, image):
        pltImage = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # Display the image
        plt.imshow(pltImage)
        plt.axis('off')

    def saveImage(self, filepath):
        cv2.imwrite(filepath, self.image)

    def computeWaypoints(self):
        contourInner, contourOuter, finishLineCenters = self.__detectContours()
        if contourInner is not None and contourOuter is not None and finishLineCenters is not None:
            sc, sl, sr = self.__computeSplines(contourInner, contourOuter, finishLineCenters)
            return sc, sl, sr
        else:
            return None, None, None
        
    def __detectContours(self, thresholdInner=10000, thresholdOuter=50000):
        # Convert image to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        canny = cv2.Canny(blur, 0, 100, apertureSize=3)

        # Dilate the edges
        kernel = np.ones((3, 3), np.uint8)        
        img = cv2.dilate(canny, kernel, iterations=3)
        kernel = np.ones((3, 3), np.uint8)
        img = cv2.erode(img, kernel, iterations=10)
        if DEBUG:
            plt.figure(2)
            self.showImage(img)        
            plt.show(block=False)
        
        # Find contours
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # Detect finish line
        finishLineCenters = None
        for contour in contours:
            # Approximate the contour to a polygon
            rect = cv2.minAreaRect(contour)
            width = rect[1][0]
            height = rect[1][1]
            # center of rect
            if width > 80 and height > 0:
                aspectRatio = width / height
                if aspectRatio > 10:
                    psi = np.deg2rad(rect[2])
                    finishLineCenters = np.array([ [ rect[0][0] - width * 0.5 * np.cos(psi), rect[0][1] - width * 0.5 * np.sin(psi)] ,
                                          [ rect[0][0] + width * 0.5 * np.cos(psi), rect[0][1] + width * 0.5 * np.sin(psi)] ])
                    finishLine = contour
                    finishLineHeight = np.int32(height)
                    # Erase finish line from image
                    canny = cv2.drawContours(canny, [finishLine], -1, 0, 4*finishLineHeight) #, thickness=cv2.FILLED)                    
                    if DEBUG:
                        approx = np.reshape(contour, (contour.shape[0], 2))
                        plt.plot(approx[:,0], approx[:,1], 'b.')                    
                    break
        
        if finishLineCenters is None:
            self.errtxt += 'No finish line found. '
            return None, None, None

        # Apply Canny edge detection once again for curbs    
        kernel = np.ones((3, 3), np.uint8)        
        img = cv2.dilate(canny, kernel, iterations=1)
        kernel = np.ones((3, 3), np.uint8)
        img = cv2.erode(img, kernel, iterations=1)

        if DEBUG:
            plt.figure(3)
            self.showImage(img)
            plt.show(block=False)
        
         # Find contours
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # Sort contours by area
        contours = sorted(contours, key=cv2.contourArea)
        # for c in contours:
        #     print(cv2.contourArea(c))          
        
        # Find inner contour
        contoursInner = [c for c in contours if cv2.contourArea(c) > thresholdInner and cv2.contourArea(c) < thresholdOuter]    
        if len(contoursInner) > 0:
            contourInner = contoursInner[-1] # take the largest contour
        else:
            self.errtxt += 'No inner contour found. '
            contourInner = None    

        # Find outer contour
        contoursOuter = [c for c in contours if cv2.contourArea(c) > thresholdOuter]    
        if len(contoursOuter) > 0:
            contourOuter = contoursOuter[0] # take the smallest contour
        else:
            self.errtxt += 'No outer contour found. '
            contourOuter = None    
        
        return contourInner, contourOuter, finishLineCenters
    
    
    def __smoothContour(self, contour, smoothFactor=5000.0, maxDist=2):
        # Approximate the contour to a polygon
        # epsilon = 0.002 * cv2.arcLength(contour, True)
        # approx = cv2.approxPolyDP(contour, epsilon, True)

        points = np.reshape(contour, (contour.shape[0], 2))
        tck, u = splprep(points.T, u=None, s=smoothFactor, per=1)
        #u_new = np.linspace(u.min(), u.max(), 1000)
        s0, s1 = splev(u, tck, der=0)

        # remove waypoints from spline that are too far from the original contour (remove outliers)
        s0_new = []
        s1_new = []
        x = []
        j = 0
        for i in range(len(u)-1):
            if abs(s0[i] - points[i,0]) <= maxDist and abs(s1[i] - points[i,1]) <= maxDist:
                s0_new.append(s0[i])
                s1_new.append(s1[i])
                if j > 0:
                    x.append(x[j-1] + np.sqrt((s0_new[j] - s0_new[j-1])**2 + (s1_new[j] - s1_new[j-1])**2))
                else:
                    x.append(0)
                j += 1

        # recompute spline without outliers
        tck, u = splprep([s0_new, s1_new], u=None, s=0, per=1)
        x = np.linspace(u[0], u[-1], num=self.numPoints)
        s0, s1 = splev(x, tck)
        s = np.array([s0, s1]).T

        return x, s, tck
    
    def __computeSplines(self, contourInner, contourOuter, finishLineCenters):
        _, _, tckl = self.__smoothContour(contourOuter)
        _, _, tckr = self.__smoothContour(contourInner)
        # start points
        xl0, _ = self.__getNearest(tckl, finishLineCenters[0])
        xr0, _ = self.__getNearest(tckr, finishLineCenters[1])
        # interpolate center line and curbs
        x = np.linspace(0, 1, self.numPoints)
        xl = (xl0 + x) % 1.0
        sl0, sl1 = splev(xl, tckl)
        sl = np.array([sl0, sl1]).T
        xr = (xr0 - x) % 1.0
        sr0, sr1 = splev(xr, tckr)
        sr = np.array([sr0, sr1]).T
        sc = 0.5 * np.add(sl, sr)

        return sc, sl, sr
    
    def __getNearest(self, tck, s):
        # search for nearest waypoint as initial guess
        x = np.linspace(0, 1, self.numPoints)
        s0, s1 = splev(x, tck)
        dist = np.sqrt((s0 - s[0])**2 + (s1 - s[1])**2)
        imin = np.argmin(dist) # nearest
        
        # numerically minimize distance
        def f(x):
            s0, s1 = splev(x, tck)
            return np.sqrt((s0 - s[0])**2 + (s1 - s[1])**2)
        
        res = minimize(f, x0=x[imin], tol=1e-6)
        s0min, s1min = splev(res.x, tck)
        
        return res.x, np.array([s0min, s1min])
    
    
if __name__ == "__main__":
    image = cv2.imread('src/mbmadvisionaruco/data/image.png')
    vision = BoardVision(image)
    plt.figure(1)
    vision.showImage(image)
    sc, sl, sr = vision.computeWaypoints()
    if sc is not None:
        plt.plot(sc[:,0], sc[:,1], 'y.')
        plt.plot(sl[:,0], sl[:,1], 'g.')
        plt.plot(sr[:,0], sr[:,1], 'r.')
        plt.plot(sc[0,0], sc[0,1], 'yo')
        plt.plot(sl[0,0], sl[0,1], 'go')
        plt.plot(sr[0,0], sr[0,1], 'ro')
    else:
        print(vision.errtxt)    
    plt.show() 
        