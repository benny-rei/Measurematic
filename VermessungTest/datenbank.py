import MySQLdb
import random

Servername = '192.168.137.11'
Benutzer = 'raspberry'
Passwort = 'raspberry'
Datenbank = 'ec_logisticsdb'


def messung_eintragen(laenge, breite, hoehe):
    verbindung = verbindung_herstellen()
    cursor = verbindung.cursor()
    
    print (laenge, breite, hoehe)

    sql_command = "INSERT INTO artikel (artikel_id,bezeichnung, " \
                  "laenge, breite, hoehe)VALUES (%s, %s, %s, %s, %s)"

    cursor.execute(sql_command, (None, 'obj', laenge, breite, hoehe))

    verbindung.commit()



def verbindung_herstellen():
    try:
        verbindung = MySQLdb.connect(host=Servername, user=Benutzer, passwd=Passwort, db=Datenbank)
    except:
        print('Keine Verbindung zur Datenbank moeglich')
        exit(0)
    return  verbindung



