26/7/2019 Alex Nijmeijer
   De MeetJeStad nodes bevatten al een geprogrammeerde EERPOM een bootloader.

   De bron-code van de "Eeprom-writer" moet eenmalig, PER NODE, worden aangepast en uitgevoerd.
   De "Eeprom-writer" software overschrijft alleen de TTN-OTAA keys. 
   Note: De OSCCAL parameter in de EEPROM wordt niet aangepast.

   Het "Node" programma bevat de uiteindelijke functionaliteit van de HitteStress Node.
   Dit programma moet worden geprogrammeerd nadat de "Eeprom-writer" is uitgevoerd.


De volgende libraries worden verondersteld te zijn geinstalleerd:
  - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
  - NeoGPS (mjs-specific fork): https://github.com/meetjestad/NeoGPS
  - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
  - lmic (mjs-specific fork): https://github.com/meetjestad/arduino-lmic
  - SDS011 by Rajko Zschiegner 
  - AltSoftSerial

