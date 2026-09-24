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
#define SDA_PIN 21
#define SCL_PIN 22
#define MODO_TESTE 1   // 1 = random | 0 = ADXL345 real
#define ADXL_ADDR_1 0x53
uint8_t adxlAddress = 0;
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


// Doc LPP: https://docs.mydevices.com/docs/lorawan/cayenne-lpp
void accelsend()
{
  float pitch;
  float roll;
  float tiltZ;



  // =========================
  // MONTA CAYENNE LPP
  // =========================

  #if MODO_TESTE 
   float t = millis() / 1000.0;
  float pitchAlvo = 15.0 * sin(t * 0.10);
  float rollAlvo  = 10.0 * sin(t * 0.07);

  // ---- PERTURBAÇÃO PERIÓDICA ----
  // A cada ~30s, dispara um "evento" que dura alguns segundos.
  // Alterna entre TRANCO (impacto curto e forte) e TOMBO (inclinação grande sustentada).
  static uint32_t proximoEventoS = 30;   // primeiro evento aos 30s
  static uint32_t fimEventoS     = 0;    // quando o evento atual termina
  static uint8_t  tipoEvento     = 0;    // 0 = tranco, 1 = tombo
  static float    tombokPitch    = 0;    // inclinação extra do tombo
  static float    tombokRoll     = 0;

  uint32_t tS = (uint32_t)t;

  // Dispara novo evento
  if (tS >= proximoEventoS && tS >= fimEventoS) {
    tipoEvento = random(0, 2);           // sorteia tipo
    if (tipoEvento == 0) {
      // TRANCO: dura 1 segundo
      fimEventoS = tS + 1;
      Serial.println(">>> EVENTO: TRANCO <<<");
    } else {
      // TOMBO: dura 5 segundos, inclinação forte
      fimEventoS   = tS + 5;
      tombokPitch  = random(-70, 71);    // ±70°
      tombokRoll   = random(-70, 71);
      Serial.print(">>> EVENTO: TOMBO pitch=");
      Serial.print(tombokPitch);
      Serial.print(" roll=");
      Serial.println(tombokRoll);
    }
    proximoEventoS = tS + 30 + random(0, 15);  // próximo em 30-45s
  }

  // Aplica efeito do evento se estiver ativo
  bool eventoAtivo = (tS < fimEventoS);
  float trancoX = 0, trancoY = 0, trancoZ = 0;

  if (eventoAtivo) {
    if (tipoEvento == 1) {
      // Tombo: sobrescreve inclinação alvo
      pitchAlvo = tombokPitch;
      rollAlvo  = tombokRoll;
    }
  }

  // Converte inclinação em componentes de gravidade
  float g = 9.81;
  float pr = pitchAlvo * PI / 180.0;
  float rr = rollAlvo  * PI / 180.0;

  float ax = g * sin(pr);
  float ay = g * sin(rr) * cos(pr);
  float az = g * cos(rr) * cos(pr);

  // Aplica TRANCO (pico de aceleração linear, some da gravidade)
  if (eventoAtivo && tipoEvento == 0) {
    // Impulso forte e aleatório em algum eixo (±20 m/s², ~2g extra)
    trancoX = (random(-2000, 2001) / 100.0);
    trancoY = (random(-2000, 2001) / 100.0);
    trancoZ = (random(-2000, 2001) / 100.0);
    ax += trancoX;
    ay += trancoY;
    az += trancoZ;
  }

  // Ruído normal
  auto ruido = []() {
    return ((random(0, 1000) / 1000.0) - 0.5) * 0.1;
  };
  ax += ruido();
  ay += ruido();
  az += ruido();

  Serial.println("=== MODO SIMULACAO ===");
  #else
    sensors_event_t event;
    accel.getEvent(&event);
    float ax = event.acceleration.x;
    float ay = event.acceleration.y;
    float az = event.acceleration.z;


    Serial.println("=== MODO SENSOR REAL ===");
  #endif
    pitch = atan2(ax,sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll =atan2(ay,sqrt(ax * ax + az * az)) * 180.0 / PI;

    tiltZ =atan2(sqrt(ax * ax + ay * ay),az) * 180.0 / PI;


  lpp.reset();

  lpp.addAnalogInput(1, pitch);
  lpp.addAnalogInput(2, roll);
  lpp.addAnalogInput(3, tiltZ);

  tamanhoPayload = lpp.getSize();

  if (tamanhoPayload > sizeof(payloadRecebido))
  {
    tamanhoPayload = sizeof(payloadRecebido);
  }

  memcpy( payloadRecebido,lpp.getBuffer(),tamanhoPayload);

  novoPayloadPronto = true;

  Serial.print("Payload LPP preparado (");
  Serial.print(tamanhoPayload);
  Serial.println(" bytes)");

  Serial.print("Pitch: ");
  Serial.print(pitch, 1);

  Serial.print(" | Roll: ");
  Serial.print(roll, 1);

  Serial.print(" | TiltZ: ");
  Serial.println(tiltZ, 1);

  Serial.println("---");
}


// Comandos:
//  - T=60   -> seta intervalo 60 seg
//  - T?     -> mostra intervalo atual
//  - OLD=1  -> envia payload antigo se não houver JSON novo
//  - OLD=0  -> não envia se não houver JSON novo


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
  Serial.println(simulateInterval);

  accelsend(); // lê sensor e monta payload LPP
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
  #if MODO_TESTE 
    randomSeed(analogRead(0));
  #else
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!accel.begin(ADXL_ADDR_1)) {
      Serial.println("Falha no ADXL345!");
      while (1);
    }
    accel.setRange(ADXL345_RANGE_2_G);
  #endif

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

  tickSendIfDue();
}
