# -*- coding: utf-8 -*-

# import the necessary packages
from __future__ import division

import imutils
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray  # fuer die Kamera
from picamera import PiCamera
import cv2
import VermessungTest4 as Vermessung  # mit Bindestrich im Name funktioniert es nicht
import time
import numpy as np
import Ultraschall1 as Ultraschall
#import datenbank as db

# definieren der Mindestgroesse der zu erkennenden Objekte
min_area = 13000 #bei Auflösung (1296, 736)
max_area = 700000 #framerand
# sonst rundet es auf 2560 = 32*80 --> sonst wird aufgerundet dann funktioniert die kamera nicht
#resolution = (1296, 736)
#resolution = (1920, 1088)
resolution = (1296, 730) # sonst wird das Bild verkleinert
#resolution = (640,480)

# # initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 40
rawCapture = PiRGBArray(camera, size=resolution)

# allow the camera to warmup
time.sleep(0.2)

# initialise the first frame
firstFrame = None

ultraschall = True

def showImage(name, image, delay):
    image2 = image.copy()
    cv2.imshow(name, imutils.resize(image2, width=800))
    cv2.moveWindow(name, 20, 20)
    cv2.waitKey(delay)
    cv2.destroyAllWindows()


def drawKontur(image, kontur, delay):
    image2 = image.copy()
    cv2.drawContours(image2, [kontur], 0, (255, 255, 0), 10)
    cv2.imshow("Image mit Kontur", imutils.resize(image2, width=800))
    cv2.moveWindow("Image mit Kontur", 20, 20)
    cv2.waitKey(delay)
    cv2.destroyAllWindows()


def drawMesspunkte(image, messpunkte, delay):
    image2 = image.copy()
    for punkt in messpunkte:
        p = messpunkte[:][:2]
        print "punkt: ", p
        cv2.drawContours(image2, np.array(punkt), 0, (255, 255, 0), 10)
    cv2.imshow("Image mit Kontur", imutils.resize(image2, width=800))
    cv2.moveWindow("Image mit Kontur", 20, 20)
    cv2.waitKey(delay)
    cv2.destroyAllWindows()


"""
passt der erste frame oder muss noch gewartet werden bis das ganze objekt im bild ist
"""
try:
    for index, frame in enumerate(camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)):
        # grab the raw NumPy array representing the frame, then initialize the timestamp
        # and occupied/unoccupied text
        picture = frame
        frame = frame.array

        # print index

        # speichert den originalen frame ab
        original = frame

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        if index <= 10:  # den ersten Frame überspringen da er sich frablich von den anderen Unterscheidet # vl aufwärmen
            continue

        #Vermessung kalibrieren
        #showImage("new", frame, 0)
        #break



        # resize the frame, convert it to grayscale, and blur it
        # den frame auf eine Standardgroesse konvertieren
        # frame = imutils.resize(frame, width=1952)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)


        # wenn der erste frame nicht None ist, wird dieser neu initialisiert
        if firstFrame is None:
            firstFrame = gray
            showImage("gray first", gray, 0)
            messpunkteUmgewandelt = [[100,100, 100], [100,100, 100], [100,100, 100], [100,100, 100]]
            l, b = Vermessung.KubaturErmitteln(original, messpunkteUmgewandelt)
            print "erster frame erkannt"
            continue


        """ am anfang machen Variante 1
        if index == 3:
            # Ultraschall testen
            # breite_bild_gesamt_mm, bildlaengePX, bildbreitePX
            messpunkteUmgewandelt, hoehe_max = Ultraschall.Messung(Vermessung.breiteBildGesamt, resolution[0],
                                                                   resolution[1])
            print "messpunkte umgewandelt\n", messpunkteUmgewandelt
            print "maximale höhe: ", hoehe_max
            continue # gleich anschließend ein neues foto machen"""

        # Vergleicht den aktuellen frame mit dem ersten frame
        frameDelta = cv2.absdiff(firstFrame, gray)
        # thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

        #showImage("Frame Delta", frameDelta, 0)

        # (15 und 255 stehen für weiß/schwarz werte) 255 schwarz 0 weiss
        #thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1]
        #thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY_INV)[1]
        #thresh = cv2.adaptiveThreshold(frameDelta, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        thresh = cv2.adaptiveThreshold(frameDelta, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)

        #kernel = np.ones((7, 7), np.uint8)
        # dilate the thresholded frame to fill in holes, then find contours
        # on thresholded frame
        #thresh = cv2.erode(thresh, None, iterations=1)
        #thresh = cv2.dilate(thresh, None, iterations=15)
        #edged = cv2.Canny(thresh, 100, 400)

        #showImage("Thresh", thresh, 0)

        konturen = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]


        if len(konturen) == 0:  # wenn keine Kontur erkannt wird, diesen Frame überspringen
            continue

        frame2 = frame.copy()

        cv2.drawContours(frame2, konturen, -1, (0, 0, 255), 5)
        #showImage("Konturen", frame2, 0)

        # die größte Kontur ermitteln und überprüfen ob sie groß genug ist.
        kontur_maximal = max(konturen, key=cv2.contourArea)

        #showImage("image: ", frame, 0)
        #showImage("thresh", thresh, 0)

        print "kontour area: ", cv2.contourArea(kontur_maximal)

        # kontur_maximal.shape = (?,1,2)
        if (cv2.contourArea(kontur_maximal) <= min_area) | (cv2.contourArea(kontur_maximal) >= max_area):
            continue
        print "groß genug"

        print "Area der maximale Kontur: ", cv2.contourArea(kontur_maximal)
        # showImage("original",original, 1000)
        #showImage("thresh",thresh, 0)
        #drawKontur(frame, kontur_maximal, 0)
        print cv2.contourArea(kontur_maximal)
        print "ARC LENGTH: ",cv2.arcLength(kontur_maximal, True)



        max_hoehe = max(kontur_maximal[:, :, 1])
        #max_hoehe = np.reshape(sorted(kontur_maximal, key=lambda point: point[0][1], reverse=True)[:10], (-1, 1, 2))
        print "max Höhe: \n",max_hoehe

        #drawKontur(frame,max_hoehe,0)
        #drawKontur(thresh, max_hoehe, 0)


        ########## Proplem kapt mit Fadne###########

        #abstand_frame_objekt = frame.shape[0] - max_hoehe[1][0][1]
        abstand_frame_objekt = frame.shape[0] - max_hoehe
        #print "frameshape: ", frame.shape[1], "max höhe: ", max(kontur_maximal[:, :, 1]), "diff= ",abstand_frame_objekt
        #if abstand_frame_objekt <= 5:
        #    continue

        drawKontur(frame, kontur_maximal, 0)
        

        # test
        #showImage("original", original, 0)
        #showImage("thresh", thresh, 0)
        #continue


        if ultraschall:
            # Ultraschall testen
            # breite_bild_gesamt_mm, bildlaengePX, bildbreitePX
            messpunkteUmgewandelt, hoehe_max = Ultraschall.Messung(Vermessung.breiteBildGesamt, resolution[0],
                                                                   resolution[1])
            print "messpunkte umgewandelt\n", messpunkteUmgewandelt
            print "maximale höhe: ", hoehe_max
            ultraschall = False
        
        l, b = Vermessung.KubaturErmitteln(original, messpunkteUmgewandelt)
        
        #db.messung_eintragen(l, b, hoehe_max) # Die gemessenen Daten in die Datenbank eintragen
        
        break

    print "finito"

except (cv2.error, KeyboardInterrupt):
    # cleanup the camera and close any open windows
    cv2.destroyAllWindows()
    print "Done"
