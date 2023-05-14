Dieses Programm dient dazu das EVN Smartmeter T210-D auszulesen und mittels ESP 32 an Mqtt Broker zu senden.
Für Fehleranalyse wurde noch ein LCD mit 16x2 Zeichen und I2C ansteuerung realisiert

Ich verwende IOBroker für die weiterverarbeitung.

Software: Visual Studio Code mit Platform IO

Zusätlich wurde die LED mittels Fotowiderstand überwacht
(damit wurde der analoge Zähler ausgewerter und war auch der Beginn bei Smartmeter)
Hier ist zu beachten, dass die Einstellung am Poti eine Herausforderung ist und dadurch auch beide Signale gesendet wordeen sind

Die Grundstruktur stammt von:
https://github.com/all4electronics/SmartMeter-Kundenschnittstelle-auslesen
Vielen dank an Dominik

Danke auch an Michael
https://www.michaelreitbauer.at/sagemcom-t210-d-auslesen-smart-meter-evn/
Die Eintraege in deinem Forum waren auch sehr hilfreich

und
https://www.mikrocontroller.net/topic/510661

und auch noch an Georg, für die Mqtt handling
https://www.mikrocontroller.net/user/show/g3gg0



Als Hardware wurde folendes verwendet:
ESP32 NodeMCU von AZDeliver
https://www.amazon.de/gp/product/B09PLBPBCC/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

Fotowiderstand:
https://www.amazon.de/gp/product/B07P5Z2XD4/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
bzw Linienmoul für Analog Zähler:
https://www.amazon.de/gp/product/B07DRCKV3X/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

Modbus Modul:

https://at.rs-online.com/web/p/entwicklungstools-kommunikation-und-drahtlos/2167484

Div Kabel:
https://www.amazon.de/gp/product/B000VN1BXQ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
https://www.amazon.de/gp/product/B07KFGS3BF/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1


I2C Display:
https://www.amazon.de/AZDelivery-HD44780-Display-Schnittstelle-Hintergrund/dp/B07JH6GHPR/ref=sr_1_1?crid=1MAEZ7LEDZT8O&keywords=lcd+16x2+i2c&qid=1684080528&s=industrial&sprefix=lcd+16%2Cindustrial%2C309&sr=1-1


