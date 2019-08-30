# MultiGeiger
Das Projekt "Multigeiger" wurde von Jürgen Böhringer entwickelt und ist auf seiner Webseite http://www.boehri.de beschrieben.

Die Dokumentation dazu ist im Verzeichnis 'doc' zu finden (incl. Bauanleitung) und der Sourcecode liegt im Verzeichnis 'src/Multigeiger' als .ino-Datei und kann mit der *Arduino-IDE* oder mit *Platformio  und Visual Code* übersetzt werden.

## Installation
Das Verzeichnis clonen oder als .zip runterladen und entpacken. Mit der Arduino-IDE in dem neuen Verzeichnis die Datei *multigeiger.ino* öffnen. Gleiches gilt für Platformio.

Als externe Libraries werden benötigt:
 * U8g2 von Oliver, aktuelle Version 2.16.4
 * IoTWebConf von Balazs Kelemen, akt. Version 2.3.0
 
In dem Sourcecode können vor dem Übersetzen noch folgende Grundeinstellungen (ab Zeile 58) gemacht werden:
 * #define SERIAL_DEBUG  
hier kann eine der weiter oben definierten Einstellungen gemacht werden. Die Bedeutng ist im Source erklärt.
 * #define SPEAKER_TICKS 0/1  
Einschalten (1) oder Aussschalten (0) der Knackgeräusche.
 * #define LED_TICK 0/1  
Ein- oder Ausschalten des Blitzens der LED bei einem Zählpuls.
 * #define PLAY_SOUND 0/1  
 Wenn eingeschaltet, wird beim Restart ein Sound abgespielt 
 * #define DEBUG_SERVER_SEND 0/1  
 wenn auf 1, dann wird jedesmal beim Senden der Daten zum Server auf der seriele Schnittstelle (USB) Debug-Info mit ausgegeben.
 
## Ablauf nach dem Start
Das Gerät baut einen eigene WLAN-Accesspoint (AP) auf. Die SSID des AP lautet **ESP-xxxxxxxx**, wobei die xxx 
die Chip-ID (bzw. die MAC-Adresse) des WLAN-Chips sind. Beispiel: **ESP-51564452**.  
Dieser access-Point bleibt für 30sec aktiv. Danach versucht das Gerät, sich mit dem (früher) eingestellten WLAN
zu verbinden. Dieser Verbindungsversuch dauer ebenfalls 30sec. Kommt keine Verbindung zu Stande, wird wieder der
eigene AP für 30sec erzeugt. Wenn das WLAN nicht erreicht werden kann, läuft dieses Spiel endlos.  
Solange keine Verbindung zum WLAN besteht, wird auf dem Display in der untersten Zeile ganz klein *connecting ...*
angezeigt. Diese Anzeige verschwindet, sobald eine WLAN-Verbindung hergstellt ist.

## Einstellung des WLAN
Wenn das Gerät den eigene AP aufgebaut hat, verbindet man sich mit diesem. Entweder mit einem Handy oder einem PC o.ä. 
Die Verbindung fragt nach einem Passwort, es lautete **ESP32Geiger**.  
Ist die Verbindung mit dem Accesspointe hergestellt, so bleibt das Timeout von 30sec stehen, d.h. man hat beliebig Zeitm die  Daten einzugeben. Es öffnet sich **automatisch** die Startseite des Gerätes. Es braucht also - in der Regel - nicht extra der Browser aufgerufen werden. Falls die Startseite ausnahmsweise doch nicht erscheint, 
so muss mit dem Browser die Adresse **192.168.4.1** aufgerufen werden und nun erscheint die Startseite. Diese besteht nur aus einer Zeile *Go to __configure page__ to change settings*. Hier auf den blauen Teil klicken und man kommt zur Einstellungsseite:  
![config](/images/config.png)  
Diese hat die folgenden 4 Zeilen:  
 * Thing Name  
 Die ist die SSID des Gerätes und kann zwar geändert werden, sollte aber nicht !!
 * AP password  
 Die ist das Passwort für den AP. Dieses **MUSS** beim ersten mal geändert werden. Es kann natürlich auch das gleiche Passwort wieder verwendet werden - wichtig ist nur, dass da was reingeschrieben wird und dass man das **nicht vergessen** darf.
 * WiFi SSID  
 Hier muss nun die SSID des eigene WLAN eingegeben werden.
 * WiFi passwort  
 Und hier das zugehörige Passwort.
 
Ist Alles eingegeben, kann man auf **Apply** drücken. Nun werden die eingestellten Daten übernommen und in das interne EEPROM gespeichert. Nun bitte **unbedingt** über **Abbrechen** diese Seite verlassen! Nur dann verlässt das Programm den Config-Mode und verbindet sich mit dem heimischen WLAN. 

## Server
Der Messzyklus beträgt 10min, d.h. es werden 10min lang die Impulse gezählt und dann der Count pro Minute (cpm) berechnet. 
Jeweils nach diesen 10min werden die Daten zu den Servern bei *luftdaten.info* und bei *madavi.de* gesendet.  
Bei *luftdaten* werden die Daten gespeichert und stehen am nächsten Tag zum Abruf als CSV-Datei bereit:  
http://archive.luftdaten.info/date/date_radiation_sbm-20_sensor_SID.csv  
wobei date = Datum im Format YYYY-MM-DD ist (beides mal gleich) und SID die Sensornummer des Sensors (**nicht** die ChipID).   
Bei *madavi* werden die Daten in einer RRD-Datenbank abgelegt und können direkt aktuell als Grafik über diesen Link betrachtet werden:  
https://www.madavi.de/sensor/graph.php?sensor=esp32-CHIPID-sbm20  
Hier ist dann CHIPID die ChipId (also die Ziffern der SSID des internen Accesspoints).  
Während der Übertragung der Daten zu den Servern wird in der Statuszeile (unterste Zeile) des Displays kurz der Name des servers eingeblendet.


 
