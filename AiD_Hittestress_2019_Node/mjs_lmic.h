/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman & Bas Peschier

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/

#define EEPROM_LAYOUT_MAGIC_OLD 0x2a60af86 // Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC 0x2a60af87 // Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC_START 0x00 // 4 bytes
#define EEPROM_OSCCAL_START (EEPROM_LAYOUT_MAGIC_START + 4) // 1 byte
#define EEPROM_APP_EUI_START (EEPROM_OSCCAL_START + 1)
#define EEPROM_APP_EUI_LEN 8
#define EEPROM_DEV_EUI_START (EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN)
#define EEPROM_DEV_EUI_LEN 8
#define EEPROM_APP_KEY_START (EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN)
#define EEPROM_APP_KEY_LEN 16
#define EEPROM_AID_HITTE_START (EEPROM_APP_KEY_START + EEPROM_APP_KEY_LEN)
#define EEPROM_AID_HITTE_LEN 5

// Try transmission for up to 60 seconds (this includes joining)
const uint32_t TX_TIMEOUT = 60000;
extern uint8_t uplink_data[2];
extern uint8_t uplink_port;
extern uint8_t JoinedStatus;

#define LED_PIN        21


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <avr/eeprom.h>


#if 1
// OTAA
void os_getArtEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN - 1 - i);
  }
}

void os_getDevEui (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_DEV_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN - 1 - i);
  }
}

void os_getDevKey (uint8_t* buf) {
  for (byte i = 0; i < EEPROM_APP_KEY_START; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_KEY_START + i);
  }
}
#else
// ABP
// also disable_join in config.h of the lmic library
// LoRaWAN NwkSKey, network session key. Copy from TTN Console MSB ... LSB
static const PROGMEM u1_t NWKSKEY[16] = { 0x54, 0xC4, 0x6F, 0xC6, 0x20, 0x7D, 0xEA, 0x76, 0x59, 0x73, 0x9B, 0xAC, 0x47, 0x49, 0xC3, 0xB4 };
// LoRaWAN AppSKey, application session key. Copy from TTN Console MSB ... LSB
static const u1_t PROGMEM APPSKEY[16] = { 0xF5, 0x6E, 0x20, 0x4A, 0x4C, 0x87, 0xB0, 0x44, 0x9A, 0x84, 0xC4, 0xFE, 0x6C, 0x35, 0x0F, 0xC3 };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011BBD;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#endif

 

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};

ev_t waitingForEvent = (ev_t)0;

void onEvent (ev_t ev) {
  if (waitingForEvent == ev)
    waitingForEvent = (ev_t)0;

  if (DEBUG) {
    Serial.print((uint32_t)os_getTime());
    Serial.print(": ");
    switch (ev) {
      case EV_SCAN_TIMEOUT:
        //Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
      case EV_BEACON_FOUND:
        //Serial.println(F("EV_BEACON_FOUND"));
        break;
      case EV_BEACON_MISSED:
        //Serial.println(F("EV_BEACON_MISSED"));
        break;
      case EV_BEACON_TRACKED:
        //Serial.println(F("EV_BEACON_TRACKED"));
        break;
      case EV_JOINING:
        // Let join start at SF9, since that's what we'll be using for
        // transmission anyway
        LMIC_setDrTxpow(DR_SF9, 14);
        
        Serial.println(F("EV_JOINING"));
        JoinedStatus = 0;
        break;
      case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        // Disable link check validation
        LMIC_setLinkCheckMode(0);

        pinMode(LED_PIN, OUTPUT);
        for (uint8_t i=0; i<10; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(200);
          digitalWrite(LED_PIN, LOW);
          delay(100);
        }
        JoinedStatus = 1;
        break;
      case EV_RFU1:
        //Serial.println(F("EV_RFU1"));
        break;
      case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
      case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
      case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if(LMIC.dataLen != 0) {
          uplink_port = LMIC.frame[LMIC.dataBeg-1];
          uplink_data[0] = LMIC.frame[LMIC.dataBeg];
          uplink_data[1] = LMIC.frame[LMIC.dataBeg+1];
        }
        break;
      case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
      case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
      case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
      case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
      case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
      default:
        Serial.println(F("Unknown event"));
        break;
    }
  }
}

void printHex(const __FlashStringHelper *prefix, uint8_t *buf, size_t len) {
  Serial.print(prefix);
  for (size_t i = 0; i < len; ++i) {
    if (buf[i] < 0x10)
      Serial.write('0');
    Serial.print(buf[i], HEX);
  }
  Serial.println();
}

void mjs_lmic_setup() {
  //Serial.println("OSSCAL CPU="+String(OSCCAL));
  // Check whether the layout of the EEPROM is correct
  uint32_t hash = eeprom_read_dword(0x00);
  if (hash != EEPROM_LAYOUT_MAGIC && hash != EEPROM_LAYOUT_MAGIC_OLD) {
    Serial.println(F("EEPROM is not correctly configured"));

    while (true) /* nothing */;
  }
  // Old magic indicates the bootloader did not handle OSCCAL yet, so we
  // need to load it from EEPROM
  if (hash == EEPROM_LAYOUT_MAGIC_OLD) {
    // Write OSCCAL from EEPROM
    uint8_t osccal_byte = eeprom_read_byte((uint8_t*)EEPROM_OSCCAL_START);
    if (osccal_byte != 0xff) {
       OSCCAL = osccal_byte;
    //Serial.println("OSSCAL EEPROM="+String(osccal_byte));
    }
  }
  //Serial.println("OSSCAL CPU="+String(OSCCAL));

  uint8_t buf[EEPROM_APP_KEY_LEN];
  os_getArtEui(buf);
  printHex(F("App EUI: "), buf, EEPROM_APP_EUI_LEN);
  os_getDevEui(buf);
  printHex(F("Dev EUI: "), buf, EEPROM_DEV_EUI_LEN);
  os_getDevKey(buf);
  printHex(F("App Key: "), buf, EEPROM_APP_KEY_LEN);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#if 0
 // ABP
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#endif
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  #if 1
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  #endif
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  LMIC_setAdrMode(0);
 //  LMIC_setLinkCheckMode(1);


  // Let LMIC compensate for +/- 2% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);   // changed 2->5
}

void mjs_lmic_wait_for_txcomplete() {
  waitingForEvent = EV_TXCOMPLETE;
  uint32_t start = millis();
  while(waitingForEvent && millis() - start < TX_TIMEOUT)
    os_runloop_once();
    if (DEBUG) {
    if (waitingForEvent)
      Serial.println(F("Transmit timeout"));
    else
      Serial.println(F("Transmit complete"));
   }
}
