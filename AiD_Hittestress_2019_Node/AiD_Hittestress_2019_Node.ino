/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp
   22/7/2019: Adapted for Apeldoorn-In-Data Hittestress by Alex Nijmeijer

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - NeoGPS (mjs-specific fork): https://github.com/meetjestad/NeoGPS
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic (mjs-specific fork): https://github.com/meetjestad/arduino-lmic
   - SDS011 by Rajko Zschiegner 
   - AltSoftSerial
 *******************************************************************************/

// include external libraries
#include <SPI.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <AltSoftSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "SDS011.h"                  

#define DEBUG true
//#define DEBUG false

#include "mjs_lmic.h"

#define MEASUREMENTS        5
#define MAXIMUM_TEMPERATURE 70
#define MINIMUM_TEMPERATURE -20
#define MAXIMUM_HUMIDITY    100.1

// define various pins
#define SDSrxPin       A3
#define SDStxPin       A2
#define GPS_PIN        8
#define VBAT_PIN       A0
#define SW_GND_PIN     20
#define LED_PIN        21
#define SW_GND_SDS_PIN (8  /*D8*/)


// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
#define BATTERY_DIVIDER_RATIO ((120.0 + 330.0) / 120.0)

// Value in mV (nominal @ 25ºC, Vcc=3.3V)
// The temperature coefficient of the reference_voltage is neglected
float const reference_voltage_internal = 1137.0;


// Most recently read values (stored in global variables)
float temperature;  // temperature
float humidity;     // humidity
int32_t lat32;      // GNSS lat
int32_t lon32;      // GNSS lon
float Vcc;          // CPU-VCC
float Vbat;         // battery voltage
float pm2_5, pm10;  // particle sensor

// Sensor objects
SDS011 sds;          // Serial Dust sensor
NMEAGPS gps;         // Serial GNSS sensor (global navigation satellite system)
HTU21D htu;          // I2C temp/hum sensor 
AltSoftSerial gpsSerial(GPS_PIN, GPS_PIN);  // Serial port for GNSS
gps_fix gps_data;
 
// Lora Payload Buffer
unsigned char mydata[20];
unsigned char mydata_size;


// setup timing variables
//uint32_t const UPDATE_INTERVAL = 900000;
uint32_t const UPDATE_INTERVAL = 60000*2;
uint32_t const GPS_TIMEOUT = 120000;

#define UPDATE_ITERATOR_MAX 6
//uint8_t update_iterator = 0; // 0 means GPS, others hum, temp, batt-levels
uint8_t update_iterator = 2; // 0 means GPS, others hum, temp, batt-levels

 
// Function Prototypes
unsigned char AiD_add_float (unsigned char idx_in,  float value);
unsigned char AiD_add_uint32 (unsigned char idx_in, int value) ;
unsigned char AiD_add_uint16 (unsigned char idx_in, uint16_t value) ;


void setup() {
  // when in debugging mode start serial connection
  if(DEBUG) {
    Serial.begin(9600);
//    Serial.println(F("Start"));
  }

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, LOW);
  digitalWrite(SW_GND_PIN, HIGH); // gnd closed

  pinMode(SW_GND_SDS_PIN, OUTPUT);
  digitalWrite(SW_GND_SDS_PIN, LOW);
  digitalWrite(SW_GND_SDS_PIN, HIGH); // gnd closed

  // blink 'hello'
#if 0
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
#endif

  // start communication to sensors
  htu.begin();
  gpsSerial.begin(9600);
  sds.begin(SDSrxPin, SDStxPin);                             // Software serial port for particle sensor
}

void loop() {
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();


  // Activate and read our sensors (do it in a loop for debugging)
  #if 0
    while (1) {
      AcquireSensorData();
      dumpSensorData();
      delay(1000);
    }
  #else 
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

  // Schedule sleep
  unsigned long msPast = millis() - startMillis;
  unsigned long sleepDuration = UPDATE_INTERVAL;
  if (msPast < sleepDuration)
    sleepDuration -= msPast;
  else
    sleepDuration = 0;

  if (DEBUG) {
    Serial.print(F("Sleeping for "));
    Serial.print(sleepDuration);
    Serial.println(F("ms..."));
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

    if (slept >= time)
      break;
    time -= slept;
  }

  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}




void queueData() 
{
 // this function is "riding along" on the update_iterator. 
 // it is called after the sensor update, which increments the update_iterator. so when the update_iterator=1, the GPS was updated.
  mydata_size = 0;              // init
 if (update_iterator==1)
  { // Compose AiD message with Location
    mydata[mydata_size++] = 0xB; // Apeldoorn In data Rev2 (send location)
    mydata_size = AiD_add_uint32(mydata_size,  (uint32_t)(lat32 & 0xFFFFFFFF));
    mydata_size = AiD_add_uint32(mydata_size,  (uint32_t)(lon32 & 0xFFFFFFFF));
  }
  else if (update_iterator==2)
  { // Compose AiD message with Battery / Supply voltage status
    mydata[mydata_size++] = 0xC; // Apeldoorn In data Rev2 (send location)
    mydata_size = AiD_add_float(mydata_size, Vcc);
    mydata_size = AiD_add_float(mydata_size, Vbat);
  } else {
  // Compose AiD message with Particle Density, humidity, temperature 
  mydata[mydata_size++] = 0xA; // Apeldoorn In data Rev2 (added GPS, battery level)
  mydata_size = AiD_add_uint16(mydata_size,  round(pm2_5*10.0)); // pm: 0 - 999.9  x10 = 0-10000 
  mydata_size = AiD_add_uint16(mydata_size,  round(pm10*10.0));
  mydata_size = AiD_add_uint16(mydata_size,  round(humidity*10));
  mydata_size = AiD_add_uint16(mydata_size,  round(temperature*10));
  }

   LMIC_setDrTxpow(DR_SF11, 14); // sf7: hoge BPS
  
  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(mydata[0], &mydata[1], mydata_size-1, 0); // packet-type as port
  Serial.println(F("Packet queued"));

}

// ------------------------------
// Read Sensors
// ------------------------------
// Temperature
/// This function prevents bogous readings from sensors.
/// \param oldTemp Last read temperature value.
/// \return new temperature from sensor.
float getTemperature(float oldTemp)
{
  float newTemp = oldTemp;
  float tempTemp = 0.0;
  int i = 0;

  while( i < MEASUREMENTS)
  {
    tempTemp = htu.readTemperature();
    
    if((tempTemp < MAXIMUM_TEMPERATURE) && (tempTemp > MINIMUM_TEMPERATURE))
    {
      i = MEASUREMENTS;
      newTemp = tempTemp;
    }
    else
    {
      i++;
    }
  }
  return newTemp;
}

// Humidity
/// This function prevents bogous readings from sensors.
/// \param oldHumid Last read humidity value.
/// \return new humidity from sensor.
float getHumidity(float oldHumid)
{
  float newHumid = oldHumid;
  float tempHumid = 0.0;
  int i = 0;

  while( i < MEASUREMENTS)
  {
    tempHumid = htu.readHumidity();
    
    if((tempHumid < MAXIMUM_HUMIDITY) && (tempHumid > 0.0))
    {
      i = MEASUREMENTS;
      newHumid = tempHumid;
    }
    else
    {
      i++;
    }
  }
  return newHumid;
}

// Particle Density
void getParticleDensity(void)
{
  int error;
  sds.wakeup();
  delay(5000);
  error = sds.read(&pm2_5,&pm10);
  sds.sleep();

  if (error) {
    Serial.println("Error reading from Dust sensor");
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

  digitalWrite(SW_GND_PIN, HIGH);
  if (DEBUG)
    Serial.println(F("Waiting for GPS..."));

  unsigned long startTime = millis();
  int valid = 0;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10) {
    if (gps.available(gpsSerial)) {
      gps_data = gps.read();
      if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
        valid++;
        lat32 =(int32_t)gps_data.latitudeL() ; // * 32768 / 10000000);
        lon32 =(int64_t)gps_data.longitudeL() ;//* 32768 / 10000000);
      } else {
        lat32 = 0;
        lon32 = 0;
      }
//      if (gps_data.valid.satellites) {
//        Serial.print(F("#Satellites: "));
//        Serial.println(gps_data.satellites);
//      }
    }
  }
  digitalWrite(SW_GND_PIN, LOW);

  if (gps.statistics.ok == 0)
    Serial.println(F("No GPS data received, check wiring"));
}

// Supply Voltage
void getVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  // Wait a bit before measuring to stabilize the reference (or
  // something).  The datasheet suggests that the first reading after
  // changing the reference is inaccurate, but just doing a dummy read
  // still gives unstable values, but this delay helps. For some reason
  // analogRead (which can also change the reference) does not need
  // this.
  delay(2);

  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH714  
  uint8_t high = ADCH; // unlocks both
  uint16_t result = (high<<8) | low;
  Vcc = (float)  1125300L / result / 1000;
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

  if (update_iterator==0)
  {
    getPosition();
    getVcc();
    getVBat();
  } else {
    getParticleDensity(); // via softserial
  
    temperature = getTemperature(temperature); // store in global variable, via I2C
    humidity = getHumidity(humidity);          // store in global variable, via I2C

  }


  if (update_iterator<UPDATE_ITERATOR_MAX)
    update_iterator++;
  else {
   //   sds.end();
      update_iterator=0;
  }
}

// ------------------------------
// Dump all Sensors to console
// ------------------------------

void dumpSensorData() {
#if 0
  if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
    Serial.print(F("lat/lon: "));
    Serial.print(gps_data.latitudeL()/10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL()/10000000.0, 6);
  } else {
    Serial.println(F("No GPS fix"));
  }
#endif

#if 0
  Serial.print("temp=" + String(temperature) + " degC ");
  Serial.print("hum=" + String(humidity)+ " % ");
#endif

#if 0
  Serial.println("Vbat=" + String(Vbat)+ " V");
  Serial.println("Vcc=" + String(Vcc)+ " V");
  Serial.println("particles 2.5um/10um: " + String(pm2_5) + "/" + String(pm10) + " ug/m3" );
#endif  
  Serial.flush();
}

// ------------------------------
// Add data-formats to packet
// ------------------------------
// FLOAT
unsigned char AiD_add_float (unsigned char idx_in,  float value) { 
  union {
     uint32_t a_uint;
     float a_float;
  } convert;
   convert.a_float = value;
   return ( AiD_add_uint32 (idx_in, convert.a_uint));
 }

// UINT32
unsigned char AiD_add_uint32 (unsigned char idx_in, uint32_t value) { 
   for (uint8_t i=0; i<4; i++) {
     mydata[idx_in++] = (value>>24) & 0xFF; // msb 
     value= value <<8;
   }
   return (idx_in);
 }

 // UINT16
unsigned char AiD_add_uint16 (unsigned char idx_in, uint16_t value) { 
   mydata[idx_in++] = (value>>8) & 0xFF; // msb 
   mydata[idx_in++] = (value) & 0xFF; // lsb
   return (idx_in);
 }
