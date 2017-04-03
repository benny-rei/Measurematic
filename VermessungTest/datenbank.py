import MySQLdb
import random

Servername = '192.168.137.11'
Benutzer = 'raspberry'
Passwort = 'raspberry'
Datenbank = 'ec_logisticsdb'



def messung_eintragen(z1, z2, z3):
    verbindung=verbindung_herstellen()
    cursor = verbindung.cursor()

    print (z1, z2, z3)

    sql_command = "INSERT INTO artikel (artikel_id,bezeichnung, " \
                  "laenge, breite, hoehe)VALUES (%s, %s, %s, %s, %s)"

    cursor.execute(sql_command, (None, 'obj', z1, z2, z3))

    verbindung.commit()



def verbindung_herstellen():
    try:
        verbindung = MySQLdb.connect(host=Servername, user=Benutzer, passwd=Passwort, db=Datenbank)
    except:
        print('Keine Verbindung zur Datenbank moeglich')
        exit(0)
    return  verbindung



