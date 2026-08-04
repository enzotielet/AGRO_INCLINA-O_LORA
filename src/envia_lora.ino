#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <ArduinoJson.h>

// ===================== CONFIG =====================
#define USE_SIMULATED_UART 1      // 1 = usa fake UART; 0 = usa Serial1 real
#define RXD2 13                   // RX da UART que recebe JSON (quando USE_SIMULATED_UART=0), usar GND também


// Intervalo (segundos) - vale para SIMULAÇÃO e para o COMPASSO DE ENVIO
uint32_t simulateInterval = 60; // em segundos

// Se true: no tick, se não houver JSON novo, ainda envia o último payload válido
static bool SEND_OLD_WHEN_NO_NEW_JSON = true;

// ===================== ABP KEYS =====================
static const PROGMEM u1_t NWKSKEY[16] = { 0x58, 0x13, 0xE5, 0x8E, 0xBC, 0xE6, 0x32, 0xB2, 0x51, 0x09, 0xFE, 0x39, 0xCF, 0x29, 0x4D, 0xB8};
static const u1_t PROGMEM APPSKEY[16] = { 0xF2, 0x23, 0xFB, 0x4B, 0xEA, 0xA1, 0xF6, 0xC2, 0x6D, 0xD0, 0x91, 0xA0, 0x62, 0xF9, 0xD1, 0x37 };
static const u4_t DEVADDR = 0x2609A304;

// Callbacks OTAA vazios (ABP)
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// Limite 51 bytes
CayenneLPP lpp(51);

// Payload pronto para TX
volatile bool novoPayloadPronto = false;
uint8_t payloadRecebido[51];
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
#if USE_SIMULATED_UART
class FakeSerialStream : public Stream {
public:
  FakeSerialStream() : head(0), tail(0) {}

  void inject(const char* s) {
    while (*s) push((uint8_t)*s++);
  }

  int available() override {
    return (tail - head + BUFSZ) % BUFSZ;
  }

  int read() override {
    if (!available()) return -1;
    uint8_t c = buf[head];
    head = (head + 1) % BUFSZ;
    return c;
  }

  int peek() override {
    if (!available()) return -1;
    return buf[head];
  }

  void flush() override {}
  size_t write(uint8_t) override { return 0; }

private:
  static const int BUFSZ = 2048;
  uint8_t buf[BUFSZ];
  volatile int head, tail;

  void push(uint8_t c) {
    int next = (tail + 1) % BUFSZ;
    if (next == head) return; // cheio -> descarta
    buf[tail] = c;
    tail = next;
  }
};

FakeSerialStream simUart;
#endif

// ===================== JSON -> CayenneLPP =====================
// Doc LPP: https://docs.mydevices.com/docs/lorawan/cayenne-lpp
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

  // IMPORTANTE: sempre resetar, senão acumula payload antigo
  lpp.reset();

  // Canais LPP
  float temperatura_ar = doc["temperatura_ar"] | 0.0f;
  lpp.addTemperature(1, temperatura_ar);
  float umidade_ar = doc["umidade_ar"] | 0.0f;
  lpp.addRelativeHumidity(2, umidade_ar);
  lpp.addAnalogInput(3, doc["umidade_solo"] | 0);
  float precipitacao = doc["precipitacao"] | 0.0f;
  lpp.addAnalogInput(4, precipitacao);
  lpp.addDigitalInput(5, doc["chovendo"] | 0);
  float velocidade_do_vento = doc["velocidade_do_vento"] | 0.0f;
  lpp.addAnalogInput(6, velocidade_do_vento);
  lpp.addBarometricPressure(7, doc["pressao_do_ar"] | 0);
  lpp.addAnalogInput(8, doc["pressao_altitude"] | 0);
  lpp.addAnalogInput(9, doc["co"] | 0);
  lpp.addAnalogInput(10, doc["nh3"] | 0);
  lpp.addAnalogInput(11, doc["no2"] | 0);
  lpp.addLuminosity(12, doc["co2"] | 0);
  // float tensao_alimentacao = doc["tensao_alimentacao"] | 0.0f;
  // lpp.addAnalogInput(13, tensao_alimentacao);
  float tempoMs = doc["tempo_ligado"] | 0;
  lpp.addDigitalInput(13, doc["direcao_do_vento_num"] | 0);
  lpp.addAnalogInput(14, tempoMs / 1000.0f);
  

  tamanhoPayload = lpp.getSize();
  if (tamanhoPayload > sizeof(payloadRecebido))
    tamanhoPayload = sizeof(payloadRecebido);

  memcpy(payloadRecebido, lpp.getBuffer(), tamanhoPayload);
  novoPayloadPronto = true;

  Serial.print("Payload LPP preparado (");
  Serial.print(tamanhoPayload);
  Serial.println(" bytes) -> pronto para LoRa.");
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
      else if (cmd == "OLD=1")
      {
        SEND_OLD_WHEN_NO_NEW_JSON = true;
        Serial.println("Modo: envia payload antigo quando não houver JSON novo.");
      }
      else if (cmd == "OLD=0")
      {
        SEND_OLD_WHEN_NO_NEW_JSON = false;
        Serial.println("Modo: NÃO envia quando não houver JSON novo.");
      }
      else
      {
        Serial.println("Comandos: T=60 | T? | OLD=1 | OLD=0");
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

// ===================== Simulação ajustável =====================
#if USE_SIMULATED_UART
void simulateUartTick()
{
  static unsigned long last = 0;
  if (millis() - last < simulateInterval * 1000UL) return;
  last = millis();

  // Agora com '\n' no final (igual você confirmou no JSON real)
  const char* jsonLine =
    "{\"temperatura_ar\":23.7,\"umidade_ar\":61,\"umidade_solo\":90,"
    "\"precipitacao\":10,\"chovendo\":7,\"velocidade_do_vento\":2.4,"
    "\"pressao_do_ar\":1013.2,\"pressao_altitude\":980.1,\"co\":1,\"nh3\":0,"
    "\"no2\":0,\"co2\":3000,\"tensao_alimentacao\":3.98,\"tempo_ligado\":123456}\n";

  Serial.print("=== SIMULACAO: injetando JSON no fake UART a cada ");
  Serial.print(simulateInterval);
  Serial.println(" s ===");

  simUart.inject(jsonLine);
}
#endif

// ===================== Captura UART por linha (terminada em '\n') =====================
static String g_lastJson = "";
static volatile bool g_lastJsonReady = false;

void pollStreamJson(Stream &s)
{
  static String line = "";
  static uint32_t lastByteMs = 0;

  const size_t MAX_LINE = 1600;      // segurança
  const uint32_t STALE_MS = 20000;    // descarta meia linha muito antiga

  while (s.available())
  {
    char c = (char)s.read();
    lastByteMs = millis();

    if (c == '\r') continue;

    if (c == '\n')
    {
      line.trim();
      if (line.length() > 0)
      {
        // guarda SEMPRE o último JSON completo
        g_lastJson = line;
        g_lastJsonReady = true;
      }
      line = "";
      continue;
    }

    line += c;

    if (line.length() > MAX_LINE)
    {
      Serial.println("Aviso: linha UART grande demais; descartando.");
      line = "";
    }
  }

  // Se parou no meio da linha por muito tempo, descarta
  if (line.length() > 0 && (millis() - lastByteMs > STALE_MS))
  {
    Serial.println("Aviso: linha UART incompleta ficou velha; descartando.");
    line = "";
  }
}

// ===================== Tick de envio no compasso do simulateInterval =====================
static uint32_t g_lastTickMs = 0;

void tickSendIfDue()
{
  uint32_t now = millis();
  uint32_t periodMs = simulateInterval * 1000UL;

  if (g_lastTickMs == 0) g_lastTickMs = now;

  if (now - g_lastTickMs < periodMs) return;
  g_lastTickMs = now;

  // se tiver JSON novo completo, atualiza payload
  if (g_lastJsonReady)
  {
    g_lastJsonReady = false;
    processJson(g_lastJson);
  }
  else
  {
    Serial.println("Tick: nenhum JSON novo completo desde o último envio.");
    if (!SEND_OLD_WHEN_NO_NEW_JSON) return;
  }

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

#if !USE_SIMULATED_UART
  // UART RX-only real (TX desabilitado)
  Serial1.begin(115200, SERIAL_8N1, RXD2, -1);
  Serial.println("BOOT: Serial1 ok (real)");
#else
  Serial.println("BOOT: usando UART SIMULADA (fake stream)");
#endif

  // SPI do rádio (T-Beam SX1276 típico)
  SPI.begin(5, 19, 27, 18);
  Serial.println("BOOT: SPI ok (LoRa)");

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
  Serial.print("SEND_OLD_WHEN_NO_NEW_JSON: ");
  Serial.println(SEND_OLD_WHEN_NO_NEW_JSON ? "true" : "false");

#if USE_SIMULATED_UART
  Serial.println("SIMULACAO ativa. Comandos: T=60 | T? | OLD=1 | OLD=0");
#else
  Serial.println("Aguardando JSON via Serial1 (RX=13)...");
#endif


}

// ===================== Loop =====================
void loop()
{
  os_runloop_once();
  handleSerialCommands();

#if USE_SIMULATED_UART
  simulateUartTick();
  pollStreamJson(simUart);   // SEMPRE drena
#else
  pollStreamJson(Serial1);   // SEMPRE drena
#endif

  tickSendIfDue();
}
