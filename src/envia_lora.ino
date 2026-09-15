#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ===================== CONFIG =====================


// Intervalo (segundos) - vale para SIMULAÇÃO e para o COMPASSO DE ENVIO
uint32_t simulateInterval = 10; // em segundos

// Se true: no tick, se não houver JSON novo, ainda envia o último payload válido
static bool SEND_OLD_WHEN_NO_NEW_JSON = true;

// ===================== ABP KEYS =====================
static const PROGMEM u1_t NWKSKEY[16] = { 0xD9, 0xDA, 0xCC, 0x9B, 0x80, 0xF0, 0xAE, 0x7D, 0x75, 0x64, 0x78, 0x4A, 0xC2, 0xF4, 0x87, 0x7F};
static const u1_t PROGMEM APPSKEY[16] = { 0x6D, 0xBE, 0xE9, 0x03, 0x80, 0x2D, 0xB6, 0xC2, 0x7D, 0x83, 0x67, 0x8A, 0x46, 0xFE, 0x67, 0x16};
static const u4_t DEVADDR = 0x260CF798;

// Callbacks OTAA vazios (ABP)
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// Limite 51 bytes
CayenneLPP lpp(12);

// Payload pronto para TX
volatile bool novoPayloadPronto = false;
uint8_t payloadRecebido[12];
uint8_t tamanhoPayload = 0;

// Job LMIC (mantido por compatibilidade)
static osjob_t sendjob;

// ===================== PINMAP (TTGO T-Beam SX1276 típico) =====================
// Ajuste se o seu hardware for diferente.
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32}, // DIO0, DIO1, DIO2
};

// ===================== SIMULAÇÃO UART (FAKE STREAM) =====================
// ===================== JSON -> CayenneLPP =====================
// Doc LPP: https://docs.mydevices.com/docs/lorawan/cayenne-lpp
void accelsend()
{

  sensors_event_t event; 
  accel.getEvent(&event);

  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  // ângulo de inclinação em cada eixo usando atan2 (resultado em radianos → graus)
  // pitch: inclinação em torno do eixo Y (frente/trás)
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  // roll: inclinação em torno do eixo X (esquerda/direita)
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

  // tilt Z: ângulo do eixo Z em relação à gravidade (0° = plano, 90° = vertical)
  float tiltZ = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;

  
  // IMPORTANTE: sempre resetar, senão acumula payload antigo
  lpp.reset();

  lpp.addAnalogInput(1,  pitch);  // Canal 1: Pitch
  lpp.addAnalogInput(2,  roll);   // Canal 2: Roll
  lpp.addAnalogInput(3,  tiltZ);  // Canal 3: Tilt Z


  tamanhoPayload = lpp.getSize();
  if (tamanhoPayload > sizeof(payloadRecebido))
    tamanhoPayload = sizeof(payloadRecebido);

  memcpy(payloadRecebido, lpp.getBuffer(), tamanhoPayload);
  novoPayloadPronto = true;

  Serial.print("Payload LPP preparado (");
  Serial.print(tamanhoPayload);
  Serial.println(" bytes) -> pronto para LoRa.");
    // --- valores brutos ---
  Serial.print("X: "); Serial.print(ax, 2);
  Serial.print("  Y: "); Serial.print(ay, 2);
  Serial.print("  Z: "); Serial.print(az, 2);
  Serial.println(" m/s^2");

  // --- ângulos calculados ---
  Serial.print("Pitch: "); Serial.print(pitch, 1); Serial.print(" graus  ");
  Serial.print("Roll:  "); Serial.print(roll,  1); Serial.print(" graus  ");
  Serial.print("Tilt Z:"); Serial.print(tiltZ, 1); Serial.println(" graus");
  Serial.println("---");

}

// ===================== Ajuste do intervalo via Serial (USB) =====================
// Comandos:
//  - T=60   -> seta intervalo 60 seg
//  - T?     -> mostra intervalo atual
//  - OLD=1  -> envia payload antigo se não houver JSON novo
//  - OLD=0  -> não envia se não houver JSON novo
void handleSerialCommands()
{
  static String cmd = "";

  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      cmd.trim();
      if (cmd.length() == 0) { cmd = ""; return; }

      if (cmd == "T?")
      {
        Serial.print("Intervalo atual (s): ");
        Serial.println(simulateInterval);
      }
      else if (cmd.startsWith("T="))
      {
        long v = cmd.substring(2).toInt();
        if (v < 10) v = 10;
        if (v > 600) v = 600;
        simulateInterval = (uint32_t)v;

        Serial.print("Novo intervalo (s): ");
        Serial.println(simulateInterval);
      }
      else 
      {
        Serial.println("Comandos: T=60 | T? ");
      }

      cmd = "";
      return;
    }

    cmd += c;
    if (cmd.length() > 64) cmd = "";
  }
}

// ===================== Eventos LMIC =====================
void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": EV: ");
  Serial.println(ev);

  if (ev == EV_TXCOMPLETE)
  {
    Serial.println("EV_TXCOMPLETE");

    if (LMIC.dataLen)
    {
      Serial.println("==========================================");
      Serial.print("Data Received: ");
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();
      Serial.println("==========================================");
    }
  }
}


static uint32_t g_lastTickMs = 0;

void tickSendIfDue()
{
  
  uint32_t now = millis();
  uint32_t periodMs = simulateInterval * 1000UL;

  if (g_lastTickMs == 0) g_lastTickMs = now;

  if (now - g_lastTickMs < periodMs) return;
  g_lastTickMs = now;


  accelsend();
  // tenta enviar (mesmo payload antigo se necessário)
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println("Tick: OP_TXRXPEND, não enviando agora.");
    return;
  }

  if (tamanhoPayload == 0)
  {
    Serial.println("Tick: ainda não existe payload válido para enviar.");
    return;
  }
  
  Serial.println("Tick: enfileirando TX LoRa...");
  //LMIC_setTxData2(1, payloadRecebido, tamanhoPayload, 0);
  Serial.printf("Canal: %u\n", LMIC.txChnl);
  Serial.printf("Freq : %lu\n", LMIC.freq);
  Serial.printf("DR   : %u\n", LMIC.datarate);
  LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
  novoPayloadPronto = false;
  Serial.println("Tick: Packet queued.");
}

// ===================== Setup =====================



void setup()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBOOT: iniciou setup()");


  // SPI do rádio (T-Beam SX1276 típico)
  SPI.begin(5, 19, 27, 18);
  Serial.println("BOOT: SPI ok (LoRa)");
  
  Wire.begin(21, 22);
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
 // displaySensorDetails();
 // displayDataRate();
 // displayRange();
  Serial.println("");

  os_init();
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  LMIC_setLinkCheckMode(0);

  // Ajustes RX2 / DR / TxPower
  LMIC.dn2Dr = DR_SF7;
  LMIC_setDrTxpow(DR_SF10, 14);

  // Desabilita canais (mantendo tua lógica)
  for (int i = 0; i < 7; i++) LMIC_disableChannel(i);
  for (int i = 16; i < 63; i++) LMIC_disableChannel(i);

  g_lastTickMs = millis();

  Serial.println("Sistema pronto.");
  Serial.print("Intervalo (s) LoRa: ");
  Serial.println(simulateInterval);
 

}

// ===================== Loop =====================
void loop()
{
  os_runloop_once();
  handleSerialCommands();


  tickSendIfDue();
}
