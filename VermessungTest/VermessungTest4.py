# -*- coding: utf-8 -*-
# fuer die verwendung von umlauten etc.
# packete importieren
from __future__ import division
import math
import cv2
import itertools
import numpy as np
import imutils

import Ultraschall1 as Ultraschall

# Kalibrierdaten
laengeBildGesamt = 220  # die Laenge des Gesamten Bildes in mm (am Boden)
breiteBildGesamt = 380  # breite des gesamten Bildes in mm
sensorHoehe = 400  # Abstand vom Sensor zum Boden in mm
laenge_referenzobjekt = 20  # Höhe des Referenzobjektes in mm
hoehe_referenzobjekt = 30
maximum = 1000

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


def drawKonturen(image, konturen, delay):
    image2 = image.copy()
    cv2.drawContours(image2, konturen, 0, (255, 255, 0), 10)
    cv2.imshow("Image mit Kontur", imutils.resize(image2, width=800))
    cv2.moveWindow("Image mit Kontur", 20, 20)
    cv2.waitKey(delay)
    cv2.destroyAllWindows()


# finden der konturen
# und anschliessendes sortieren nach groesse
def Konturen(image):  # anzahl ... wie viele Konturen sollen behalten werden
    # Bild zu schwarzweiss convertieren und rauschen entfernen (GaussianBlur)
    # anschliessend werden die Konturen erkannt (Canny)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    # threshold the image, then perform a series of erosions +
    # dilations to remove any small
    #  regions of noise
    thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)[1]
    #thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
    thresh = cv2.dilate(thresh, None, iterations=10)
    thresh = cv2.erode(thresh, None, iterations=10)


    edged = cv2.Canny(thresh, 50, 400)

    # Test1
    # Vorschau des Originals und des bearbeiteten Bildes
    # print "Vorschau 1"

    showImage("original", image, 0)

    showImage("thresh", thresh, 0)

    showImage("edged", edged, 0) 

    _, konturen, _ = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    image2 = image.copy()
    cv2.drawContours(image2, konturen, -1, (255, 255, 0), 5)
    showImage("alle konturen", image2, 0)

    # kontur_maximal = max(konturen, key=cv2.contourArea)
    kontur_maximal = max(konturen, key=lambda k: cv2.arcLength(k, False))
    print "arc length= ", cv2.arcLength(kontur_maximal, False), " Area: ", cv2.contourArea(kontur_maximal)

    drawKontur(image, kontur_maximal, 0)

    drehwinkel_grad = np.rad2deg(Drehwinkel(image, kontur_maximal))

    print "Drehwinkel: ", drehwinkel_grad

    # oberste Kontur finden
    # dann:
    image_gedreht = imutils.rotate_bound(edged, (drehwinkel_grad * -1))
    image_gedreht_orig = imutils.rotate_bound(image, (drehwinkel_grad * -1))
    showImage("gedreht ",image_gedreht, 0)

    _, konturen, _ = cv2.findContours(image_gedreht.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    kontur_maximal = np.array(max(konturen, key=lambda k: cv2.arcLength(k, False))) # nochmal die Kontur des gedrehten Bildes ermitteln

    kontur_maximal = np.resize(kontur_maximal, (-1,1,2))

    return kontur_maximal, image_gedreht_orig  # gibt die größte Kontur zurück


# jeder kontur auf ihre merkmale untersuchen um herauszufinden ob es die richtige ist
def Drehwinkel(image, kontur):
    """
    Die beiden höchsten Konturpunkte finden und anschließend den Winkel daraus berechnen
    :param image: Das Bild auf dem sich das Objekt befindet
    :param Kontur: Die größte erkannte Kontur
    :return: Gibt den Drehwinkel in Grad zurück
    """

    # Umfang der Kontur ermitteln und ein Polygon an die Kontur annähern
    umfang = cv2.arcLength(kontur, True)
    poly = cv2.approxPolyDP(kontur, 0.01 * umfang, False)

    # Den die obersten beiden Punkte der Kontur bestimmen
    oberstenPunkte = np.reshape(sorted(poly, key=lambda point: point[0][1], reverse=True)[:2], (2, 1, 2))

    x_diff = oberstenPunkte[0][0][0] - oberstenPunkte[1][0][0]
    y_diff = oberstenPunkte[0][0][1] - oberstenPunkte[1][0][1]

    print x_diff, y_diff

    seitenverhaeltnis = y_diff / x_diff

    drehwinkel = np.arctan(seitenverhaeltnis)

    return drehwinkel


def Eckpunkte(kontur):
    """
    Aus der Objektkontur werden die vier Eckpunkte des Objektes ermittelt
    :param kontur: die Kontur des Objektes, gespeichtert in einem numpy-Array
    :return: Gibt die Punkte links, rechts, oben und unten in einem Array zurück
    """

    links = kontur[kontur[:, :, 0].argmin()][0]
    rechts = kontur[kontur[:, :, 0].argmax()][0]
    oben = kontur[kontur[:, :, 1].argmin()][0]
    unten = kontur[kontur[:, :, 1].argmax()][0]

    kontur_objekt = np.array([links, rechts, oben, unten])

    return kontur_objekt


def AbstandZwischenPunkten(eckpunkt, messpunkt, hoehe_min, pixelpoints):
    if messpunkt[0][2] <= hoehe_min:
        return maximum

    if np.argwhere(pixelpoints == [messpunkt[0]]) == []:
        return maximum

    return math.sqrt((eckpunkt[0] - messpunkt[0][0]) ** 2 + (eckpunkt[1] - messpunkt[0][1]) ** 2)


def UltraschallMesspuntZuEckpunkt(eckpunkte, messpunkte, hoehe_min, image, kontur_max):
    """
    Jedem Eckpunkt wird ein Messpunkt zugeordnet
    :param eckpunkte: Die ermittelten Eckpunkte des Objekts
    :param messpunkte: Ein Array aller Messpunkte
    :param hoehe_min: Minimale Höhe des Objektes
    :param image: Das aufgenommene Bild
    :param kontur_max: Die Randkontur des Objekts
    :return: Gibt ein Array zurück in dem die vier Eckpunkte und deren dazu gefundenen Höhen abgespeichert sind
    """

    messpunkte_passend = np.zeros((4, 1, 3))  # links, rechts, oben, unten

    # Maske erstellen, die alle Pixel der Kontur beinhaltet
    mask = np.zeros(image.shape[:2], np.uint8)
    cv2.drawContours(mask, [kontur_max], 0, 255, -1)
    pixelpoints = np.transpose(np.nonzero(mask))

    for i, e in enumerate(eckpunkte):
        if i == 2:
            # dem obersten Punkt den Höhenwert des Referenzobjektes zuweisen
            messpunkte_passend[i] = [[0,0,hoehe_referenzobjekt]]

        messpunkte_passend[i] = min(messpunkte, key=lambda m: AbstandZwischenPunkten(e, m, hoehe_min, pixelpoints))

    return np.array(messpunkte_passend)


def BreiteInMM(breite_bild_PX, breite_bild_MM, breite_objekt_PX, hoehe_sensoren, hoehe_objekt_durchschnitt):
    """
    breite_bild_PX ... horizontale Auflösung des Bildes in Pixel
    breite_bild_MM ... breite des Bildes im mm am Boden
    breite_objekt_PX ... Breite des Objektes in PX
    hoehe_sensoren ... Abstand vom Boden zu den Sensoren in mm
    hoehe_objekt_durchschnitt ... beschreibt den Mittelwert der Höhen der beiden Messpunkte
    """

    # die Länge die einem Pixel am Boden entspricht
    pixel_breite_boden = float(breite_bild_MM) / float(breite_bild_PX)
    print "pixel_breite_boden: ", pixel_breite_boden
    # passt die Pixelgröße der Höhe an
    relative_pixelgroesse = float(hoehe_sensoren) / (float(hoehe_sensoren) - float(hoehe_objekt_durchschnitt))
    print "relative_pixelgroesse", relative_pixelgroesse
    breite_objekt_mm = breite_objekt_PX * pixel_breite_boden * relative_pixelgroesse
    print "breite_objekt_mm", breite_objekt_mm

    return laenge_objekt_mm


def LaengeInMM(laenge_bild_PX, laenge_bild_MM, laenge_objekt_PX, hoehe_sensoren, hoehe_objekt_durchschnitt):
    """
    laenge_bild_PX ... vertikale Auflösung des Bildes in Pixel
    laenge_bild_MM ... länge des Bildes im mm am Boden
    laenge_objekt_PX ... länge des Objektes in PX
    hoehe_sensoren ... Abstand vom Boden zu den Sensoren in mm
    hoehe_objekt_durchschnitt ... beschreibt den Mittelwert der Höhen der beiden Messpunkte
    """

    # die Länge die einem Pixel am Boden entspricht
    pixel_laenge_boden = float(laenge_bild_MM) / float(laenge_bild_PX)
    print "pixel_laenge_boden: ", pixel_laenge_boden
    # passt die Pixelgröße der Höhe an
    relative_pixelgroesse = float(hoehe_sensoren) / (float(hoehe_sensoren) - float(hoehe_objekt_durchschnitt))
    print "relative_pixelgroesse", relative_pixelgroesse
    laenge_objekt_mm = laenge_objekt_PX * pixel_laenge_boden * relative_pixelgroesse
    print "laenge_objekt_mm", laenge_objekt_mm

    laenge_objekt_mm -= laenge_referenzobjekt

    return laenge_objekt_mm


def KubaturErmitteln(image, messpunkte):
    drehwinkel = 0  # --< für Testzwäcke
    # image = Bild_Drehen(image, np.rad2deg(drehwinkel))
    kontur_max, image_gedreht = Konturen(image)  # (?,1,2)
    print "max Kontur: ",kontur_max[0]
    eckpunkte = Eckpunkte(kontur_max)

    print "Eckpunkte: \n", eckpunkte

    image_gedreht2 = image_gedreht.copy()
    for e in eckpunkte:
        cv2.drawContours(image_gedreht2, np.reshape(e, (1,1,2)), 0, (0, 0, 255), 50)

    showImage("gedrehtes Bild", image_gedreht2, 0)

    # Rechteck anlegen zur Veranschaulichung
    x, y, w, h = cv2.boundingRect(eckpunkte)
    cv2.rectangle(image_gedreht, (x, y), (x + w, y + h), (0, 255, 0), 2)

    showImage("Außenkonturen", image_gedreht,0)

    laengeBildGesamtPX, breiteBildGesamtPX = image.shape[:2]  # rows, colums



    width = eckpunkte[1][0] - eckpunkte[0][0]
    length = eckpunkte[3][1] - eckpunkte[2][1]

    messpunkte_eckpunkte_passend = UltraschallMesspuntZuEckpunkt(eckpunkte, messpunkte, Ultraschall.hoehe_minimal,
                                                                 image, kontur_max)

    print "passende messpunkte: \n", messpunkte_eckpunkte_passend

    # eckpunktUndHoehe.append([e, hoehe])
    hoehe_objekt_durchschnitt_laenge = (messpunkte_eckpunkte_passend[2][0][2] + messpunkte_eckpunkte_passend[3][0][
        2]) / 2
    hl = hoehe_objekt_durchschnitt_laenge
    laenge_MM = LaengeInMM(laengeBildGesamtPX, laengeBildGesamt, length, sensorHoehe, hl)
    hoehe_objekt_durchschnitt_breite = (messpunkte_eckpunkte_passend[1][0][2] + messpunkte_eckpunkte_passend[0][0][
        2]) / 2
    hb = hoehe_objekt_durchschnitt_breite
    breite_MM = BreiteInMM(breiteBildGesamtPX, breiteBildGesamt, width, sensorHoehe, hb)
    print "Teil 6 fertig"

    print "durchschnittliche höhen: breite: ", hb, " länge: ", hl


    print "width: ", breite_MM, " mm"
    print "laenge: ", laenge_MM, " mm"
    # print "hoehe: " + str(hoehe_maximal) + " mm"

    return laenge_MM, breite_MM  # gibt die drei Kubatur Daten zurück (länge, breite, höhe)


"""def UmfangErmitteln(image):
    drehwinkel = 0  # --< für Testzwäcke
    # image = Bild_Drehen(image, np.rad2deg(drehwinkel))
    kontur_max = Konturen(image)  # (?,1,2)

    eckpunkte = Eckpunkte(kontur_max)

    print "Eckpunkte: \n", eckpunkte

    # Rechteck anlegen zur Veranschaulichung
    x, y, w, h = cv2.boundingRect(eckpunkte)
    cv2.rectangle(image_gedreht, (x, y), (x + w, y + h), (0, 255, 0), 2)




    laengeBildGesamtPX, breiteBildGesamtPX = image.shape[:2]  # rows, colums

    width = eckpunkte[1][0] - eckpunkte[0][0]
    length = eckpunkte[3][1] - eckpunkte[2][1]

    print "kontur area: ", cv2.contourArea(kontur_max)
    print "width: ", width, " PX"
    print "laenge: ", length, " PX"
    # print "hoehe: " + str(hoehe_maximal) + " mm"

    return width, length  # gibt die zwei Flächen Daten zurück (länge, breite, höhe)
    """
