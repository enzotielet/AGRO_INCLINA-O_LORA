//Descomentar se usar heltec
#include "heltec.h"



//Por padrão, o código usa dragino. Se usar Heltec, descomentar linha abaixo.
#define heltec

// Descomentar se utilizar o payload Cayenne LPP
#define cayenne 

//Configurações DHT
#ifdef heltec
  #define DHTPIN 13 // Pino 13 do Heltec
#else
  #define DHTPIN A1 //Pino A1 Dragino/Arduíno
#endif
#define DHTTYPE DHT11 // DHT 11

//Configuracoes luminosidade
//#ifdef heltec
  #define PINO_LUZ 12 
//#endif
 
// Instancia DHT
DHT dht(DHTPIN, DHTTYPE);

//Define qual dos dispositivos será compilado. Somente um por vez pode ser compilado
#define ttn_dragino 
//#define ttn_heltec_forte 
//#define ttn_heltec_fraco 

//Define canais utilizados. Somente um por vez pode ser utilizado
//#define canal_unico //Canal único usado no Heltec Single Gateway de sferrigo
//#define ttn_caxias_915 //Canais usados na TTN Caxias e canal 915.1 configurado no SGW
#define ttn_caxias //Canais usados na TTN Caxias

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const PROGMEM u1_t NWKSKEY[16] = { 0x62, 0x2A, 0x51, 0x71, 0xE1, 0xD8, 0x20, 0x1B, 0x45, 0x60, 0x11, 0x37, 0xFE, 0x9F, 0x1B, 0x63 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//static const u1_t PROGMEM APPSKEY[16] = { 0x11, 0xC7, 0xF4, 0xF7, 0x43, 0xB2, 0xE1, 0x31, 0x69, 0x84, 0x6E, 0xE9, 0xEE, 0x9F, 0x87, 0xD0 };

//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
//static const u4_t DEVADDR = 0x2603149D;

#ifdef ttn_dragino
  static const PROGMEM u1_t NWKSKEY[16] = { 0xEB, 0xC4, 0xCD, 0xAA, 0xDC, 0x2E, 0x7A, 0x35, 0x80, 0x0B, 0x52, 0xF4, 0xD3, 0xA8, 0x8E, 0x20 };
  static const u1_t PROGMEM APPSKEY[16] = { 0xD5, 0xB0, 0xB6, 0x0D, 0xE2, 0x9A, 0xBA, 0x08, 0xD1, 0xB4, 0x2E, 0xB2, 0xCD, 0x37, 0xAE, 0x9B };
  static const u4_t DEVADDR = 0x260317AD;
#endif
#ifdef ttn_heltec_forte
  static const PROGMEM u1_t NWKSKEY[16] = { 0xDE, 0x19, 0x05, 0x7B, 0xB6, 0x9C, 0x76, 0xE7, 0x5F, 0xDB, 0x1C, 0x08, 0x23, 0xE3, 0x31, 0x9F };
  static const u1_t PROGMEM APPSKEY[16] = { 0xD8, 0x7B, 0xCD, 0x10, 0xD4, 0x67, 0x39, 0x07, 0x45, 0xC8, 0x44, 0x16, 0x4D, 0x93, 0x9E, 0x48 };
  static const u4_t DEVADDR = 0x2603178B;
#endif
#ifdef ttn_heltec_fraco
  static const PROGMEM u1_t NWKSKEY[16] = { 0xF7, 0x0F, 0x22, 0x84, 0x5C, 0x07, 0x50, 0x84, 0x31, 0x5C, 0x0D, 0x5B, 0x54, 0x96, 0xBC, 0x3F };
  static const u1_t PROGMEM APPSKEY[16] = { 0x1C, 0xF6, 0xAC, 0x56, 0x01, 0x38, 0xA7, 0xCC, 0xD7, 0xC1, 0x2E, 0xF9, 0x0B, 0x1A, 0x54, 0x52 };
  static const u4_t DEVADDR = 0x26031DAA;
#endif
