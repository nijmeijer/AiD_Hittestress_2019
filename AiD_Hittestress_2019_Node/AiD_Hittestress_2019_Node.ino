  /*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp
   22/7/2019: Adapted for Apeldoorn-In-Data Hittestress by Alex Nijmeijer

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   MeetJeStad board setup : https://github.com/meetjestad/mjs_boards/raw/master/package_meetjestad.net_index.json

   In order to compile the following libraries need to be installed:
   - "DHT.h" (local copy, improved timing)
   - NeoGPS (mjs-specific fork): https://github.com/meetjestad/NeoGPS see also project-specific config-file in libraries zip
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic https://github.com/matthijskooijman/arduino-lmic 1.5.0+arduino-2
   - SDS011 by Rajko Zschiegner 
   - AltSoftSerial

   - Arduino-IDE version 1.8.9 (1.8.10 failed on linker-issues related to LMIC)
 *******************************************************************************/

// include external libraries
#include <AltSoftSerial.h>

#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "SDS011.h"                  
#include "DHT.h"

#define DEBUG true
//#define DEBUG false

#include "mjs_lmic.h"

// define various pins
#define SDSrxPin       A3
#define SDStxPin       A2
#define GPS_PIN        8
#define VBAT_PIN       A0
#define SW_GND_PIN     20
#define LED_PIN        21
#define SW_GND_SDS_PIN (8  /*D8*/)
#define DHTPIN         7       
#define DHTTYPE        DHT22    // (MM) DHT 22  (AM2305), AM2321


// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
#define BATTERY_DIVIDER_RATIO ((120.0 + 330.0) / 120.0)

// Most recently read values (stored in global variables)
float temperature;  // outside-temperature
float CpuTemp;      // cpu-temperature
float humidity;     // outside-humidity
int32_t lat32;      // GNSS lat in deg
int32_t lon32;      // GNSS lon in deg
int16_t alt16;      // GNSS altitude in meters
#ifdef GPS_FIX_HDOP
  int16_t hdop16;     //  Horizontal Dilution of Precision
#endif

float Vbat;         // battery voltage
float pm2_5, pm10;  // particle sensor

// Sensor objects
SDS011        sds;                         // Serial Dust sensor
NMEAGPS       gps;                         // Serial GNSS sensor (global navigation satellite system)
DHT           ht(DHTPIN, DHTTYPE, 2);      // AM2305 sensor for temp/hum, 2:override 0/1 threshold in lib. Lib has been altered to avoid timing-issues
AltSoftSerial gpsSerial(GPS_PIN, GPS_PIN); // Serial port for GNSS
gps_fix       gps_data;
 
// Lora Payload Buffer
uint8_t mydata[15];
uint8_t mydata_size;
uint8_t uplink_data[2];
uint8_t uplink_port=0;

// setup timing variables
uint16_t update_interval_secs=120; 
uint8_t  dust_delay_secs = 5;
uint16_t const GPS_TIMEOUT = 120000;                  // 120 secs

uint16_t update_iterator_cnt = 0; // 0: , 3: generates GPS
uint8_t PacketType, PacketTypeNext;

// Function Prototypes
unsigned char AiD_add_uint32 (unsigned char idx_in, int value) ;
unsigned char AiD_add_uint16 (unsigned char idx_in, uint16_t value) ;

void setup() {
  // when in debugging mode start serial connection
  if(DEBUG) {
    //Serial.begin(9600);
    Serial.begin(115200);
  }

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, LOW); // initially, GPS is disabled

  // blink 'hello'
#if 0
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
#endif

  // start communication to sensors
  gpsSerial.begin(9600);
  sds.begin(SDSrxPin, SDStxPin);                             // Software serial port for particle sensor

  // Make sure we have the initial power-up Vbat and CpuTemp (for debugging / calibration)
  getVBat();
  GetCpuTemp();
  
}

void loop() {
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();
  Serial.print(F("Time-loop-entry "));
  Serial.print(millis() / 1000);
  Serial.println(F(" sec "));
  

  // Activate and read our sensors (do it in a loop for sensor debugging)
  #if 0
    while (1) {
      Update_Iterator();
      AcquireSensorData();
      dumpSensorData();
      //delay(3000);
    }
  #else 
      Update_Iterator();
      AcquireSensorData();
      dumpSensorData();
  #endif
  
  // Work around a race condition in LMIC, that is greatly amplified
  // if we sleep without calling runloop and then queue data
  // See https://github.com/lmic-lib/lmic/issues/3
  os_runloop_once();

  // We can now send the data
  queueData();

  mjs_lmic_wait_for_txcomplete();

  // check for received UPLINK data
  if (uplink_port != 0) {
              Serial.print(F("Data Received @port "));
              Serial.print(uplink_port);
              Serial.print(F("="));
              Serial.print(uplink_data[0]);
              Serial.print(F(" "));
              Serial.println(uplink_data[1]);
                
              switch (uplink_port) { // port
                case 2 : update_interval_secs = uplink_data[0] | (uplink_data[1]<<8);
                         Serial.println(update_interval_secs);
                         update_iterator_cnt = 0;
                         break;
                case 3 : if ( uplink_data[0]>1 and  uplink_data[0]<50) // in range?
                         {
                           dust_delay_secs = uplink_data[0] ;
                           Serial.println(dust_delay_secs);
                           update_iterator_cnt = 0;
                         }
                         break;
                case 4 : update_iterator_cnt = 0;
                        break;
            }
            uplink_port = 0;
          }


  // Schedule sleep
  unsigned long sleepDuration = (unsigned long)update_interval_secs*1000; //UPDATE_INTERVAL;
  if (PacketTypeNext!=3 || PacketType!=3) sleepDuration=(unsigned long)update_interval_secs*500; //UPDATE_INTERVAL/2; // we are about to transmit a mid/low pri packet, or we will do so the next packet => the mid/low packet will be correctly spaced in time

  Serial.print(F("Uncompensated interval: ")); 
  Serial.println((unsigned long)sleepDuration);
  sleepDuration = sleepDuration *181/120;
  Serial.print(F("Scaled interval: ")); 
  Serial.println((unsigned long)sleepDuration);
 
  unsigned long msPast = millis() - startMillis;
  if (msPast < sleepDuration)
    sleepDuration -= msPast;
  else
    sleepDuration = 0;



  if (DEBUG) {
    Serial.print(F("Time "));
    Serial.print(millis() / 1000);
    Serial.println(F(" sec "));
  
    
    Serial.print(F("Sleeping for "));
    Serial.print(sleepDuration);
    Serial.print(F("ms... "));
    Serial.flush();
  }

  
  doSleep(sleepDuration);
  if (DEBUG) {
    Serial.println(F("Woke up."));
  }
}

void doSleep(uint32_t time) {
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  while (time > 0) {
    uint16_t slept;
    if (time < 8000)
      slept = Watchdog.sleep(time);
    else
      slept = Watchdog.sleep(8000);
#if 1
    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000) / (64 * 256);
    }
#endif
    if (slept >= time)
      break;
    time -= slept;
  }

  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}

void queueData() 
{
  mydata_size = 0;              // init
  switch (PacketType) {
    case 1:  // Compose AiD message with Location
              mydata[mydata_size++] = 0xB; 
              mydata_size = AiD_add_uint32(mydata_size,  (uint32_t)(lat32 & 0xFFFFFFFF));
              mydata_size = AiD_add_uint32(mydata_size,  (uint32_t)(lon32 & 0xFFFFFFFF));
              mydata_size = AiD_add_uint16(mydata_size,  (uint16_t)(alt16 & 0xFFFF));
              #ifdef GPS_FIX_HDOP
                mydata_size = AiD_add_uint16(mydata_size,  (uint16_t)(hdop16 & 0xFFFF));
              #else
                mydata_size = AiD_add_uint16(mydata_size,  (uint16_t)0);
              #endif
              break;

     case 2:  // Apeldoorn In data Rev2 (send battery status, CPU-Temp)
              mydata[mydata_size++] = 0xD; 
              mydata_size = AiD_add_uint16(mydata_size, round(CpuTemp*10));
              mydata_size = AiD_add_uint16(mydata_size, round(Vbat*100));
              break;
               
     case 3:  // Compose AiD message with Particle Density, humidity, temperature 
              mydata[mydata_size++] = 0xE; 
              mydata_size = AiD_add_uint16(mydata_size,  round(pm2_5*10.0)); // pm: 0 - 999.9  x10 = 0-10000 
              mydata_size = AiD_add_uint16(mydata_size,  round(pm10*10.0));
              mydata_size = AiD_add_uint16(mydata_size,  round(humidity*10));
              mydata_size = AiD_add_uint16(mydata_size,  round(temperature*10));
              break;
     default: ;// should never happen
          
  } // case

  //LMIC_setDrTxpow(DR_SF7, 14); // sf7: hoge BPS
    LMIC_setDrTxpow(DR_SF9, 14);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else  {
       // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(mydata[0], &mydata[1], mydata_size-1, 0); // packet-type as port
    }
    
    Serial.println(("Packet-type ") + String(PacketType) + (" queued. Size=")+ String(mydata_size) + (". Next type ")+ String(PacketTypeNext));
     
}

void Update_Iterator () {

  update_iterator_cnt++; // 16 bits => wraps around in 0xFFFF increments => divs below must be powers of 2, for even distribution over time
  PacketType=3;          // default HI PRI, unless changed below
  
  if (update_iterator_cnt % 128 == 4) { // lowest PRI, shifted in time W.R.T. mid PRI
   // Serial.println(F("Low Pri"));
    PacketType=1;
  }

  if (update_iterator_cnt % 8 == 0) { // mid PRI // 8
    //Serial.println(F("Mid Pri"));
    PacketType=2;
  }

  if (PacketType==3) {                // Hi PRI
    //Serial.println(F("Hi Pri"));
  }

  PacketTypeNext = 3;
  if ((update_iterator_cnt+1) % 32 == 4) PacketTypeNext = 1;
  if ((update_iterator_cnt+1) % 8 == 0) PacketTypeNext = 2;
  
}



// ------------------------------
// Read Sensors
// ------------------------------
// Temperature
float getTemperature(float oldTemp)
{
  float newTemp;
  float tempTemp; 
    
  tempTemp = ht.readTemperature();
  // Check if any reads failed and exit early (to try again).
  if (isnan(tempTemp)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      newTemp = oldTemp;
     } else
     newTemp = tempTemp;

  return newTemp;
}

void GetCpuTemp () 
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication. 
  // Calibration, assuming 1.22 scaling is correct:
  // Assumes device is at room-temperature at first power-up
  #define wADC_ROOMTEMP (333)
  #define ROOMTEMP (22.0)
  #define wADC_OFFSET (ROOMTEMP*1.22-wADC_ROOMTEMP)
  t = (wADC + wADC_OFFSET ) / 1.22;
  
  Serial.print(F("wADC#CpuTemp="));
  Serial.println(String(wADC));
  CpuTemp = t;
}

// Humidity
float getHumidity(float oldHumid)
{
  float newHumid ;
  float tempHumid;

  tempHumid = ht.readHumidity(); // from manual: The minimum interval for reading sensor 2S; reading interval time is less than 2S, may lead to temperature and humidity are not allowed or communication is unsuccessful and so on.
  if (isnan(tempHumid)) {
       Serial.println(F("Failed to read from DHT sensor!"));
       newHumid = oldHumid;
    } else
      newHumid = tempHumid;
    
  return newHumid;
}

// Particle Density
void getParticleDensity(void)
{
  int error;
  uint16_t dust_delay_ms;
  
  dust_delay_ms = dust_delay_secs * 1000;
  
  sds.newwakeup();
  delay(dust_delay_ms);
  error = sds.read(&pm2_5,&pm10);
  sds.sleep();

  if (error) {
    Serial.println(F("Error reading from Dust sensor"));
    pm2_5 = -1.0;
    pm10 = -1;
   }
}

// Position
// GPS and related SoftSerial is normally disabled, unless a position is acquired.
void getPosition()
{
  // clear memory
  memset(&gps_data, 0, sizeof(gps_data));
  gps.reset();
  gps.statistics.init();
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, HIGH); // HIGH: enable GPS (LOW causes error message later on)
  if (DEBUG)
    Serial.println(F("Waiting for GPS..."));

  unsigned long startTime = millis();
  int valid = 0;
  int valid_hdop = 1;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10) {
    if (gps.available(gpsSerial)) {
      gps_data = gps.read();
      
      #ifdef GPS_FIX_HDOP
        valid_hdop = gps_data.valid.hdop;
      #endif
      
      if (gps_data.valid.location && gps_data.valid.altitude &&  gps_data.valid.status && valid_hdop && gps_data.status >= gps_fix::STATUS_STD) { //STATUS_STD (STATUS_DGPS wordt nooit bereikt)
        valid++;
        lat32 =(int32_t)gps_data.latitudeL() ; // * 32768 / 10000000);
        lon32 =(int32_t)gps_data.longitudeL() ;//* 32768 / 10000000);
        alt16 = (int16_t)gps_data.altitude();
        #ifdef GPS_FIX_HDOP
          hdop16 = (int16_t)gps_data.hdop;
        #endif  
      } else {
        //lat32 = 0;
        //lon32 = 0;
        //alt16=0;
        //#ifdef GPS_FIX_HDOP
        //  hdop16 = 0;
        //#endif  

      }
      #if 1
      if (gps_data.valid.satellites) {
        Serial.print(F("#Satellites: "));
        Serial.println(gps_data.satellites);
      }
      #endif
    }
  }
  digitalWrite(SW_GND_PIN, LOW);

  if (gps.statistics.ok == 0)
    Serial.println(F("No GPS data received, check wiring"));
}

// Battery voltage
void getVBat()
{
  analogReference(INTERNAL);
  uint16_t reading = analogRead(VBAT_PIN);
  Vbat = (float)1.1*reading/1023 * BATTERY_DIVIDER_RATIO;
} 



// ------------------------------
// Acquire Sensor Data
// ------------------------------
void AcquireSensorData ()
{
  switch (PacketType)  {
    case 1:  getPosition();
    case 2:  getVBat();
             GetCpuTemp();
    case 3:  getParticleDensity();                      // via softserial
             temperature = getTemperature(temperature); // store in global variable, via I2C
             humidity = getHumidity(humidity);          // store in global variable, via I2C
  }
}

// ------------------------------
// Dump all Sensors to console
// ------------------------------

void dumpSensorData() {
#if 0
  if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
    Serial.print(F("lat/lon: "));
    Serial.print(lat32/10000000, 6);
    Serial.print(F(","));
    Serial.println(lon32/10000000, 6);
    Serial.println(alt16);
    #ifdef GPS_FIX_HDOP
      Serial.println(hdop16);
    #endif
  } else {
    Serial.println(F("No GPS fix"));
  }
#endif

#if 0
  Serial.print(F("temp=") + String(temperature) + F("degC  "));
  Serial.println(F("hum=") + String(humidity)+ F("%  "));
  Serial.println(F("particles 2.5um/10um: ") + String(pm2_5) + F("/") + String(pm10) + F(" ug/m3") );
#endif

#if 0
  Serial.println(F("Vbat=") + String(Vbat)+ " V");
  Serial.println(F("CpuTemp=") + String(CpuTemp) + " degC  ");
#endif  
  Serial.flush();
}

// ------------------------------
// Add data-formats to packet
// ------------------------------

// UINT32
unsigned char AiD_add_uint32 (unsigned char idx_in, uint32_t value) { 
   for (uint8_t i=0; i<4; i++) {
     mydata[idx_in++] = (value>>24) & 0xFF; // msb 
     value= value <<8;                      // shift-left
   }
   return (idx_in);
 }

 // UINT16
unsigned char AiD_add_uint16 (unsigned char idx_in, uint16_t value) { 
   mydata[idx_in++] = (value>>8) & 0xFF; // msb 
   mydata[idx_in++] = (value) & 0xFF;    // lsb
   return (idx_in);
 }
