/*
  ============================================================
  ADXL345 + ESP32 LoRa (LMIC, ABP)
  - Amostragem rápida (200 ms) com detecção de evento
  - Envio periódico: fase inicial rápida -> ritmo normal
  - Envio por evento (tranco / tombo) com cooldown
  Monitor Serial: 115200 baud
  ============================================================
*/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

// ===================== I2C / ADXL345 =====================
#define SDA_PIN     21
#define SCL_PIN     22
#define ADXL_ADDR   0x53
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ===================== TEMPOS DE ENVIO =====================
const uint32_t ENVIOS_INICIAIS     = 10;      // quantos envios no ritmo inicial
const uint32_t INTERVALO_INICIAL_S = 10;      // segundos entre os primeiros envios
const uint32_t INTERVALO_NORMAL_S  = 60 * 5;  // segundos entre os demais envios

// ===================== AMOSTRAGEM + DETECCAO =====================
const uint32_t AMOSTRAGEM_MS      = 200;   // periodo de amostragem
const float    THR_ACEL_LINEAR    = 15.0;  // m/s^2 alem da gravidade -> tranco
const float    THR_INCLINACAO     = 45.0;  // graus (pitch/roll) -> tombo
const uint32_t COOLDOWN_ALARME_S  = 30;    // minimo entre envios por evento

// Se true: no tick, se nao houver leitura nova, ainda reenvia o ultimo payload
static bool SEND_OLD_WHEN_NO_NEW = true;

// ===================== ABP KEYS =====================
static const PROGMEM u1_t NWKSKEY[16] = { 0xD9, 0xDA, 0xCC, 0x9B, 0x80, 0xF0, 0xAE, 0x7D, 0x75, 0x64, 0x78, 0x4A, 0xC2, 0xF4, 0x87, 0x7F };
static const u1_t PROGMEM APPSKEY[16] = { 0x6D, 0xBE, 0xE9, 0x03, 0x80, 0x2D, 0xB6, 0xC2, 0x7D, 0x83, 0x67, 0x8A, 0x46, 0xFE, 0x67, 0x16 };
static const u4_t DEVADDR = 0x260CF798;

// Callbacks OTAA vazios (ABP)
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// ===================== PAYLOAD =====================
CayenneLPP lpp(12);
uint8_t  payloadRecebido[12];
uint8_t  tamanhoPayload   = 0;
bool     novoPayloadPronto = false;

// ===================== PINMAP LORA =====================
const lmic_pinmap lmic_pins = {
  .nss  = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst  = 14,
  .dio  = {26, 33, 32}, // DIO0, DIO1, DIO2
};

// ===================== ESTADO =====================
// Ultimas leituras
static float g_pitch = 0, g_roll = 0, g_tiltZ = 0;
static float g_ax = 0, g_ay = 0, g_az = 0;
static bool  g_temLeitura = false;

// Controle de ticks e eventos
static uint32_t g_lastTickMs   = 0;
static uint32_t g_lastSampleMs = 0;
static uint32_t g_lastAlarmeMs = 0;
static uint32_t g_enviosFeitos = 0;
static bool     g_pedidoEnvio  = false; // flag levantada por evento

uint32_t intervaloAtualS()
{
  return (g_enviosFeitos < ENVIOS_INICIAIS) ? INTERVALO_INICIAL_S : INTERVALO_NORMAL_S;
}

// ===================== AMOSTRAGEM DO SENSOR =====================
void amostrarSensor()
{
  sensors_event_t event;
  accel.getEvent(&event);
  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  g_ax = ax; g_ay = ay; g_az = az;
  g_pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  g_roll  = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
  g_tiltZ = atan2(sqrt(ax*ax + ay*ay), az)  * 180.0 / PI;
  g_temLeitura = true;

  // --- Deteccao de evento extremo ---
  float accMag    = sqrt(ax*ax + ay*ay + az*az);
  float accLinear = fabs(accMag - 9.81);  // aceleracao alem da gravidade

  const char* motivo = nullptr;
  if (accLinear > THR_ACEL_LINEAR)                                motivo = "TRANCO";
  else if (fabs(g_pitch) > THR_INCLINACAO ||
           fabs(g_roll)  > THR_INCLINACAO)                        motivo = "TOMBO";

  if (motivo) {
    uint32_t now = millis();
    bool foraCooldown = (g_lastAlarmeMs == 0) ||
                        (now - g_lastAlarmeMs >= COOLDOWN_ALARME_S * 1000UL);
    if (foraCooldown) {
      Serial.printf("!!! EVENTO: %s | accLin=%.1f pitch=%.1f roll=%.1f !!!\n",
                    motivo, accLinear, g_pitch, g_roll);
      g_pedidoEnvio  = true;
      g_lastAlarmeMs = now;
    }
  }
}

// ===================== MONTAGEM DO PAYLOAD =====================
void prepararPayload()
{
  if (!g_temLeitura) return;

  lpp.reset();
  lpp.addAnalogInput(1, g_pitch);
  lpp.addAnalogInput(2, g_roll);
  lpp.addAnalogInput(3, g_tiltZ);

  tamanhoPayload = lpp.getSize();
  if (tamanhoPayload > sizeof(payloadRecebido))
    tamanhoPayload = sizeof(payloadRecebido);
  memcpy(payloadRecebido, lpp.getBuffer(), tamanhoPayload);
  novoPayloadPronto = true;
}

// ===================== EVENTOS LMIC =====================
void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": EV: ");
  Serial.println(ev);

  if (ev == EV_TXCOMPLETE) {
    Serial.println("EV_TXCOMPLETE");
    if (LMIC.dataLen) {
      Serial.println("==========================================");
      Serial.print("Data Received: ");
      Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
      Serial.println();
      Serial.println("==========================================");
    }
  }
}

// ===================== TICKS =====================
void tickAmostrar()
{
  uint32_t now = millis();
  if (g_lastSampleMs == 0) g_lastSampleMs = now;
  if (now - g_lastSampleMs < AMOSTRAGEM_MS) return;
  g_lastSampleMs = now;
  amostrarSensor();
}

void tickSendIfDue()
{
  uint32_t now      = millis();
  uint32_t periodMs = intervaloAtualS() * 1000UL;

  if (g_lastTickMs == 0) g_lastTickMs = now;
  bool porTempo  = (now - g_lastTickMs >= periodMs);
  bool porEvento = g_pedidoEnvio;

  if (!porTempo && !porEvento) return;

  if (LMIC.opmode & OP_TXRXPEND) {
    // mantem a flag; tenta no proximo loop
    return;
  }

  prepararPayload();

  if (tamanhoPayload == 0) {
    Serial.println("Tick: sem payload valido.");
    return;
  }

  if (!g_temLeitura && !SEND_OLD_WHEN_NO_NEW) {
    Serial.println("Tick: sem leitura nova, envio pulado.");
    return;
  }

  const char* motivo = porEvento ? "EVENTO" : "TEMPO";
  Serial.printf("Tick (%s): TX LoRa | Pitch=%.1f Roll=%.1f TiltZ=%.1f\n",
                motivo, g_pitch, g_roll, g_tiltZ);
  Serial.printf("Canal: %u | Freq: %lu | DR: %u\n",
                LMIC.txChnl, LMIC.freq, LMIC.datarate);

  LMIC_setTxData2(1, payloadRecebido, tamanhoPayload, 0);
  novoPayloadPronto = false;
  g_pedidoEnvio     = false;
  g_lastTickMs      = now;      // reinicia o relogio do envio periodico
  g_enviosFeitos++;

  Serial.printf("Envio #%lu | proximo (tempo) em %lu s",
                (unsigned long)g_enviosFeitos, (unsigned long)intervaloAtualS());
  if (g_enviosFeitos == ENVIOS_INICIAIS) {
    Serial.print("  (fim da fase inicial)");
  }
  Serial.println();
}

// ===================== SETUP =====================
void setup()
{
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBOOT: iniciou setup()");

  // --- ADXL345 ---
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!accel.begin(ADXL_ADDR)) {
    Serial.println("Falha no ADXL345! Travando.");
    while (1) { delay(1000); }
  }
  accel.setRange(ADXL345_RANGE_4_G); // 4G cobre trancos moderados; use _8_G se precisar mais

  // --- LoRa ---
  delay(100);
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

  LMIC.dn2Dr = DR_SF7;
  LMIC_setDrTxpow(DR_SF10, 14);

  for (int i = 0;  i < 7;  i++) LMIC_disableChannel(i);
  for (int i = 16; i < 63; i++) LMIC_disableChannel(i);

  g_lastTickMs   = millis();
  g_lastSampleMs = millis();

  Serial.println("Sistema pronto.");
  Serial.printf("Primeiros %lu envios: a cada %lu s\n",
                (unsigned long)ENVIOS_INICIAIS, (unsigned long)INTERVALO_INICIAL_S);
  Serial.printf("Demais envios: a cada %lu s\n", (unsigned long)INTERVALO_NORMAL_S);
  Serial.printf("Amostragem: %lu ms | Thr tranco: %.1f m/s^2 | Thr inclinacao: %.1f deg | Cooldown: %lu s\n",
                (unsigned long)AMOSTRAGEM_MS, THR_ACEL_LINEAR, THR_INCLINACAO,
                (unsigned long)COOLDOWN_ALARME_S);
}

// ===================== LOOP =====================
void loop()
{
  os_runloop_once();
  tickAmostrar();
  tickSendIfDue();
}