h# -*- coding: utf-8 -*-

# Bibliotheken einbinden
from __future__ import division

import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setwarnings(False)  # damit nit immer die Meldung kommt dass die Channels schon in use sind

# wichtige Gößen definieren
# Ladelaenge_Maximal = 300  # mm
# Zeit_pro_Messung = 1.015  # Millisekunden <-- Aus Messungen
# Messreihen_pro_Ladung = Ladelaenge_Maximal / (Zeit_pro_Messung * 4)
Schallgeschwindigkeit = 344000  # mm/sekunde

# Sensoren Abstand zum Mittelpunkt
Sensor_Abstand_Mitte = [65, 30, -30, -65]  # - Bedeutet nach links (Fahrtrichtung) in mm

# Sensoren Abstand zum Boden
Sensoren_Abstand_Boden = 322  # millimeter

# Hoehe angeben ab der gemessen werden soll in Millimeter
hoehe_minimal = 40  # mm

GPIO_trigger_echo = np.zeros((4, 2), dtype=int)  # dtype int, sonst gibts probleme mit dem initialisieren
print GPIO_trigger_echo.shape

# GPIO Modus (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
# GPIO Pins zuweisen
# Sensor 1
# [0] ... trigger | [1] ... echo
GPIO_trigger_echo[0][0] = 6
GPIO_trigger_echo[0][1] = 5
# Sensor 2
GPIO_trigger_echo[1][0] = 13
GPIO_trigger_echo[1][1] = 12
# Sensor 3
GPIO_trigger_echo[2][0] = 20
GPIO_trigger_echo[2][1] = 19
# Sensor 3
GPIO_trigger_echo[3][0] = 21
GPIO_trigger_echo[3][1] = 26

# Richtung der GPIO-Pins festlegen (IN / OUT)
for gpio in GPIO_trigger_echo:
    GPIO.setup(gpio[0], GPIO.OUT)
    GPIO.setup(gpio[1], GPIO.IN)


def Hoehe(GPIO_TRIGGER, GPIO_ECHO):
    # setze Trigger auf HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # setze Trigger nach 0.01ms auf LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartZeit = time.time()
    StopZeit = time.time()

    # speichere Startzeit
    while GPIO.input(GPIO_ECHO) == 0:
        StartZeit = time.time()

    # speichere Ankunftszeit
    while GPIO.input(GPIO_ECHO) == 1:
        StopZeit = time.time()

    # Zeit Differenz zwischen Start und Ankunft
    TimeElapsed = StopZeit - StartZeit
    # mit der Schallgeschwindigkeit (344000 mm/s) multiplizieren
    # und durch 2 teilen, da hin und zurueck
    distanz = (TimeElapsed * Schallgeschwindigkeit) / 2

    hoehe = Sensoren_Abstand_Boden - distanz

    return hoehe



def MesspunkteDrehen(messpunkte, mittelpunkt, drehwinkelGrad):
    """
    Die einzelnen Messpunkte werden zu Vektoren (Schaft beim Mittelpunkt) umgewandelt
    und anschließend um den gewünschten Winkel gedreht
    :param messpunkte: Array der Ultraschallmesspunkte
    :param mittelpunkt: Mittelpunkt des aufgenommenen Bildes
    :param drehwinkelGrad: Drehwinkel des Objektes in Grad
    :return: Gibt das Array an gedrehten Messpunkten
    """

    x = mittelpunkt[0]
    y = mittelpunkt[1]

    for m in messpunkte:
        m[0], m[1] = (m[0] - x), (m[1] - y)
        laenge = np.sqrt(m[0] ** 2 + m[1] ** 2, dtype=float)
        winkelAlt = np.arctan(m[0], m[1], dtype=float)
        winkelAlt = np.rad2deg(winkelAlt)
        winkelNeu = winkelAlt + drehwinkelGrad
        m[0] = np.arcsin(winkelNeu) * laenge
        m[1] = np.arccos(winkelNeu) * laenge
        m[0] += x
        m[1] += y

    return messpunkte

def MesspunktUmwandeln(messpunkt, messzeitraum_gesamt, bildlaengePX, bildbreiteMM, bildbreitePX):
    """
    Die Ultraschallmesspunkte werden je einem Pixel auf dem Bild zugeordnet
    :param messpunkt: Der Messpunkt der umgewandelt wird
    :param messzeitraum_gesamt: Zeitdifferenz zwischen dem ersten und dem letzten Messpunkt
    :param bildlaengePX: maximale darstellbare Länge des Bildes am Boden
    :param bildbreiteMM: maximale darstellbare Breite des Bildes aum Boden
    :param bildbreitePX: Vertikale Auflösung der Kamera
    :return: Gibt die x- und y- Koordinate auf dem Bild und die gemessene Höhe zurück
    """

    x = (((bildbreiteMM / 2) + Sensor_Abstand_Mitte[messpunkt[0]]) / bildbreiteMM) * bildbreitePX
    y = ((bildlaengePX * messpunkt[1]) / messzeitraum_gesamt)

    # messpunkt[2] ... gemessene Höhe des Messpunktes
    return x, y, messpunkt[2]


def Messung(breite_bild_gesamt_mm, bildlaengePX, bildbreitePX):
    messpunkte = []
    messpunkte_umgewandelt = []
    Anz_Sensoren_Fertig = 0
    startTime = None
    messzeitraum_gesamt = None
    Anz_Sensoren_Bereit = 0
    counter = 0

    hoehe = None

    # warten bis die Palette den Messraum betritt, spricht wenn die Mindesthöhe erreicht wird
    while True:
        Anz_Sensoren_Bereit = 0
        for gpio in GPIO_trigger_echo:
            hoehe = Hoehe(gpio[0], gpio[1])
            #print gpio, " misst: ", hoehe
            if hoehe >= hoehe_minimal:
                Anz_Sensoren_Bereit += 1

        if Anz_Sensoren_Bereit >= 2:
            startTime = time.time()
            break
    print "minimale Höhe überschritten"

    while True:
        # vor jedem Auswerten wird die Anzahl der fertigen Sensoren auf 0 gesetzt
        Anz_Sensoren_Fertig = 0

        for index, gpio in enumerate(GPIO_trigger_echo):  # enumerate braucht man für den index

            hoehe = Hoehe(gpio[0], gpio[1])
            time.sleep(0.1) # bissl warten zwischen den Messungen um bessere Ergebnisse zu erzielen

            # print index, " ", gpio, ": Höhe: ",hoehe

            if (hoehe < hoehe_minimal):
                Anz_Sensoren_Fertig += 1

            stopTime = time.time()
            messzeitpunkt = stopTime - startTime;
            messpunkt = [index, messzeitpunkt, hoehe]
            messpunkte.append(messpunkt)  # Messpunkte in eine Liste einfügen

        if (Anz_Sensoren_Fertig >= 4):
            print "fertig"
            messzeitraum_gesamt = stopTime - startTime
            break


    print "Messzeitraum gesamt: ",messzeitraum_gesamt

    for m in messpunkte:
        # die Klammern sind dafür da, dass es die gleiche formatierung wie a kontur hat
        messpunkte_umgewandelt.append([
            MesspunktUmwandeln(m, messzeitraum_gesamt, bildlaengePX, breite_bild_gesamt_mm, bildbreitePX)])

    # eventuell list zu Array machen, nach tests
    messpunkte_umgewandelt = np.array(messpunkte_umgewandelt, dtype=int)
    print messpunkte_umgewandelt.shape

    # messpunkte_umgewandelt shape: [[x,y,h][x,y,h]]
    hoehe_maximal = max(messpunkte_umgewandelt[:,0, 2])
    print "hoehe maximal: ", hoehe_maximal

    np.set_printoptions(precision=3)

    hoechsten = np.array(sorted(messpunkte_umgewandelt, key=lambda m: m[0][2], reverse=True)[:10])

    print "die 10 höchsten punkte: \n", hoechsten

    # gibt die Messpunkte und die Gesamtdauer der Messung zurück
    return messpunkte_umgewandelt, hoehe_maximal

