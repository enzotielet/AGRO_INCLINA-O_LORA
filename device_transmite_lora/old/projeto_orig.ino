#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "TemperaturaUmidadeAr.h"
#include "UmidadeSolo.h"
#include "Anemometro.h"
#include "DirecaoVento.h"
#include "PostWifi.h"
#include <CayenneLPP.h>

//Apenas o pino 13 é usado, pois é o RX
#define RXD2 13
//#define TXD2 14

// Parâmetros do dispositivo
static const PROGMEM u1_t NWKSKEY[16] = { 0x58, 0x13, 0xE5, 0x8E, 0xBC, 0xE6, 0x32, 0xB2, 0x51, 0x09, 0xFE, 0x39, 0xCF, 0x29, 0x4D, 0xB8};
static const u1_t PROGMEM APPSKEY[16] = { 0xF2, 0x23, 0xFB, 0x4B, 0xEA, 0xA1, 0xF6, 0xC2, 0x6D, 0xD0, 0x91, 0xA0, 0x62, 0xF9, 0xD1, 0x37 };
static const u4_t DEVADDR = 0x2609A304;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// Limite 51 bytes
CayenneLPP lpp(51);
bool novoPayloadPronto = false;
uint8_t payloadRecebido[51];
uint8_t tamanhoPayload = 0;
//  static uint8_t  mydata[51];

// Talvez deixar só o sendjob
// static osjob_t initjob, sendjob, blinkjob;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60; // Padrão 60

// Pin mapping TTGO T-Beam v1.2 (AXP2101 + SX1276)
const lmic_pinmap lmic_pins = {
    .nss = 18, // CS / NSS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,          // Reset do SX1276
    .dio = {26, 33, 32} // DIO0, DIO1, DIO2
};

// Junção que decodifica JSON para envio ao Cayenne
void processJson(const String &jsonStr)
{
  Serial.println("JSON recebido:");
  Serial.println(jsonStr);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error)
  {
    Serial.print("Erro no parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
  // lpp.reset();

  // Adiciona os campos (ajuste canais conforme quiser)
  int temperatura_ar;
  temperatura_ar = doc["temperatura_ar"];
  int umidade_ar = doc["umidade_ar"];
  lpp.addTemperature(1, doc["temperatura_ar"]);
  // lpp.addTemperature(1, 23.5); //Teste fixo
  lpp.addRelativeHumidity(2, doc["umidade_ar"]);
  lpp.addAnalogInput(3, doc["umidade_solo"]);
  lpp.addAnalogInput(4, doc["precipitacao"]);
  lpp.addDigitalInput(5, doc["chovendo"]);
  lpp.addAnalogInput(6, doc["velocidade_do_vento"]);
  lpp.addAnalogInput(7, doc["pressao_do_ar"]);
  lpp.addAnalogInput(8, doc["pressao_altitude"]);
  lpp.addAnalogInput(9, doc["co"]);
  lpp.addAnalogInput(10, doc["nh3"]);
  lpp.addAnalogInput(11, doc["no2"]);
  lpp.addAnalogInput(12, doc["co2"]);
  lpp.addAnalogInput(13, doc["tensao_alimentacao"]);
  float tempoSegundos = doc["tempo_ligado"] | 0; // valor padrão
  lpp.addAnalogInput(14, tempoSegundos / 1000.0);

  // Mostra payload LPP
  Serial.print("Payload LPP (");
  Serial.print(lpp.getSize());
  Serial.println(" bytes):");
  // int tamanhoPayload;
  // uint8_t payloadRecebido[51];
  novoPayloadPronto = true;
  tamanhoPayload = lpp.getSize();
  memcpy(payloadRecebido, lpp.getBuffer(), tamanhoPayload);

  Serial.println("temperatura_ar:");
  Serial.println(temperatura_ar);
  Serial.println(umidade_ar);

  Serial.println("Payload LPP armazenado para transmissão LoRa.");
}

void do_send(osjob_t *j)
{
  Serial.println("teste do_send");
  // Delay para leitura dos dados
  // delay(TX_INTERVAL * 1000);
  
  // Lê dados da UART2 (JSON)
  static String inputString = "";
  while (Serial1.available())
  {
    char c = Serial1.read();
    Serial.print(c);
    if (c != '\n')
      inputString += c;
    else
    {
      processJson(inputString);
      // Serial.println(inputString);
      inputString = "";
    }
    if (!Serial1.available())
      delay(10);
  }

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }

  novoPayloadPronto = true;
  if (novoPayloadPronto)
  {
    Serial.println("Enviando payload recebido via JSON...");
    LMIC_setTxData2(1, payloadRecebido, tamanhoPayload, 0);
    novoPayloadPronto = false; // limpa flag após envio

    Serial.println();
    Serial.println("Packet queued");
    Serial.print("TX nº: ");
    Serial.println(LMIC.freq);
  }
}

void onEvent(ev_t ev)
{
  // sleep(5);
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.print("EV: ");
  Serial.println(ev);
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println("EV_SCAN_TIMEOUT");
    break;
  case EV_BEACON_FOUND:
    Serial.println("EV_BEACON_FOUND");
    // LMIC_sendAlive();
    break;
  case EV_BEACON_MISSED:
    Serial.println("EV_BEACON_MISSED");
    break;
  case EV_BEACON_TRACKED:
    Serial.println("EV_BEACON_TRACKED");
    break;
  case EV_JOINING:
    Serial.println("EV_JOINING");
    break;
  case EV_JOINED:
    Serial.println("EV_JOINED");
    // LMIC_setPingable(1);
    // Serial.println("SCANNING...");
    break;
  case EV_RFU1:
    Serial.println("EV_RFU1");
    break;
  case EV_JOIN_FAILED:
    Serial.println("EV_JOIN_FAILED");
    break;
  case EV_REJOIN_FAILED:
    Serial.println("EV_REJOIN_FAILED");
    break;
  case EV_TXCOMPLETE:
    Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
    if (LMIC.dataLen)
    {
      // data received in rx slot after tx
      Serial.println("==========================================");
      Serial.print("Data Received: ");
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println("==========================================");
      Serial.println();
    }
    else
    {
      Serial.println("==========================================");
      Serial.println("Nada recebido!");
      Serial.println("==========================================");
      Serial.println();
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    Serial.println("Agendando nova transmissão...");
    break;
  case EV_LOST_TSYNC:
    Serial.println("EV_LOST_TSYNC");
    break;
  case EV_RESET:
    Serial.println("EV_RESET");
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println("EV_RXCOMPLETE");
    break;
  case EV_LINK_DEAD:
    Serial.println("EV_LINK_DEAD");
    break;
  case EV_LINK_ALIVE:
    Serial.println("EV_LINK_ALIVE");
    break;
  default:
    Serial.println("Unknown event");
    break;
  }
}

void setup()
{

  Serial.begin(115200);                     // Monitor
  Serial1.begin(115200, SERIAL_8N1, 13, 2); // UART2 (para receber JSON)

  os_init();

  LMIC_reset();
  // On AVR, these values are stored in flash and only copied to RAM
#ifdef PROGMEM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF7;
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 14); // Ver se GW está no 10; 14 é 14dBM
                               // Desabilita os canais desnecessários dos Gateways de Caxias.
  for (int i = 0; i < 7; i++)
  {
    LMIC_disableChannel(i);
    Serial.println("Desabilitando canal ");
  }

  for (int i = 16; i < 63; i++) // alterado i de 15 para 16 para fechar envios a cada 10 min
  {
    LMIC_disableChannel(i);
    Serial.println("Desabilitando canal ");
  }

  // Start job
  do_send(&sendjob);
}

void loop()
{
  os_runloop_once();
}
