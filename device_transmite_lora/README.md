# ADXL345 + ESP32 LoRa — Monitor de Inclinação e Impacto

Firmware para ESP32 com acelerômetro ADXL345 que monitora **inclinação** e **impactos** de um objeto e envia os dados por **LoRaWAN** (ABP) usando a biblioteca LMIC. O payload é empacotado em formato **Cayenne LPP**.

O dispositivo opera em dois modos de envio simultâneos:

- **Periódico**: envia leituras em intervalos regulares (fase inicial rápida para teste de link, depois ritmo normal).
- **Por evento**: envia imediatamente quando detecta um **tranco** (aceleração acima do limite) ou **tombo** (inclinação acima do limite), respeitando um cooldown para evitar spam.

---

## Hardware

| Componente | Descrição |
|---|---|
| ESP32 LoRa (Heltec / TTGO ou similar) | MCU + rádio SX127x |
| ADXL345 | Acelerômetro 3 eixos via I²C |
| Antena LoRa adequada à banda | 915 MHz (AU915 / US915) |

### Ligações

**I²C (ADXL345):**

| ADXL345 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Endereço I²C: `0x53` (SDO em GND). Para `0x1D`, puxar SDO em VCC e trocar `ADXL_ADDR` no código.

**LoRa (pinmap LMIC):**

| Função | GPIO |
|---|---|
| NSS | 18 |
| RST | 14 |
| DIO0 | 26 |
| DIO1 | 33 |
| DIO2 | 32 |

SPI usa os pinos padrão da plataforma (SCK/MISO/MOSI).

---

## Dependências

Instalar as bibliotecas pelo Library Manager da Arduino IDE ou pelo `platformio.ini`:

- `MCCI LoRaWAN LMIC library` (ou `arduino-lmic`)
- `CayenneLPP`
- `Adafruit ADXL345`
- `Adafruit Unified Sensor`
- `Wire` e `SPI` (padrão do core ESP32)

### Configuração do LMIC

No arquivo `lmic_project_config.h` (ou `project_config/lmic_project_config.h`), definir a região correta — **esse código assume AU915**:

```c
#define CFG_au915 1
#define CFG_sx1276_radio 1
```

Se usar US915, trocar `CFG_au915` por `CFG_us915` e ajustar os canais no `setup()` conforme a sub-banda do gateway.

---

## Configuração da aplicação

### Chaves ABP

Definidas no topo do `.ino`. **Trocar pelos valores do seu dispositivo na rede LoRaWAN** antes de gravar:

```cpp
static const PROGMEM u1_t NWKSKEY[16] = { ... };
static const u1_t PROGMEM APPSKEY[16] = { ... };
static const u4_t DEVADDR = 0x260CF798;
```

### Parâmetros ajustáveis

Todos no topo do arquivo:

| Parâmetro | Default | O que faz |
|---|---|---|
| `ENVIOS_INICIAIS` | 10 | Quantos envios usam o ritmo inicial antes de ir ao ritmo normal |
| `INTERVALO_INICIAL_S` | 10 s | Período entre os primeiros envios (fase de validação) |
| `INTERVALO_NORMAL_S` | 300 s (5 min) | Período entre os demais envios |
| `AMOSTRAGEM_MS` | 200 ms | Frequência de leitura do acelerômetro |
| `THR_ACEL_LINEAR` | 15.0 m/s² | Limite de aceleração extra (além da gravidade) para disparar **TRANCO** |
| `THR_INCLINACAO` | 45.0 ° | Limite de pitch/roll para disparar **TOMBO** |
| `COOLDOWN_ALARME_S` | 30 s | Tempo mínimo entre dois envios por evento |
| `SEND_OLD_WHEN_NO_NEW` | `true` | Se `true`, reenvia o último payload quando não há leitura nova |

A faixa do ADXL345 é definida em `setup()` com `accel.setRange(ADXL345_RANGE_4_G)`. 

---

## Payload Cayenne LPP

Cada envio contém 3 canais do tipo *Analog Input*:

| Canal | Grandeza | Unidade |
|---|---|---|
| 1 | Pitch (inclinação frente-trás) | graus |
| 2 | Roll (inclinação lateral) | graus |
| 3 | TiltZ (desvio total do eixo Z em relação à vertical) | graus |



---

## Como funciona por dentro

### Amostragem (`tickAmostrar` → `amostrarSensor`)

A cada `AMOSTRAGEM_MS`, lê os três eixos do ADXL345 e calcula:

- **Pitch** e **roll** a partir da decomposição da gravidade entre os eixos (`atan2`).
- **TiltZ**: ângulo entre o eixo Z atual e a vertical real.
- **accMag**: magnitude do vetor aceleração.
- **accLinear**: `|accMag − 9.81|` — a aceleração extra, "além da gravidade". Independe da orientação do sensor.

### Detecção de evento

Verifica em ordem de prioridade:

1. **TRANCO**: `accLinear > THR_ACEL_LINEAR` — impacto/pancada.
2. **TOMBO**: `|pitch| > THR_INCLINACAO` **ou** `|roll| > THR_INCLINACAO` — ficou tombado.

TRANCO ganha de TOMBO porque, durante um impacto, o cálculo de pitch/roll fica contaminado pela aceleração extra e não é confiável.

Se há evento e o cooldown já passou, levanta `g_pedidoEnvio` para o próximo tick de envio.

### Envio (`tickSendIfDue`)

Dispara TX quando qualquer um for verdade:

- Chegou o momento do envio periódico (`porTempo`), ou
- Há pedido de envio por evento (`porEvento`).

Se o LMIC está no meio de outra TX (`OP_TXRXPEND`), segura e tenta no próximo loop. Chama `prepararPayload`, enfileira com `LMIC_setTxData2` e reinicia o relógio do envio periódico.

### Fase inicial x fase normal

Nos primeiros `ENVIOS_INICIAIS` pacotes, usa `INTERVALO_INICIAL_S`. Depois, troca para `INTERVALO_NORMAL_S` automaticamente. Útil para validar o link rapidamente sem desperdiçar energia depois.

---

## Monitor Serial

Baud rate: **115200**.

Saída típica:

```
BOOT: iniciou setup()
Sistema pronto.
Primeiros 10 envios: a cada 10 s
Demais envios: a cada 300 s
Amostragem: 200 ms | Thr tranco: 15.0 m/s^2 | Thr inclinacao: 45.0 deg | Cooldown: 30 s
Tick (TEMPO): TX LoRa | Pitch=0.3 Roll=-1.2 TiltZ=1.4
Canal: 10 | Freq: 923300000 | DR: 2
Envio #1 | proximo (tempo) em 10 s
...
!!! EVENTO: TRANCO | accLin=22.4 pitch=5.1 roll=-3.0 !!!
Tick (EVENTO): TX LoRa | Pitch=5.1 Roll=-3.0 TiltZ=6.0
```

---

## Ajustando os limites na prática

- **TRANCO** (`THR_ACEL_LINEAR` em m/s²): caminhada ~1–3, carro freando forte ~5–8, tranco/batida ~10–30, queda livre ~9.8. 15.0 é um bom ponto de partida para "pancada notável".
- **TOMBO** (`THR_INCLINACAO` em graus): 45° detecta "caiu de lado" sem falso-positivo em inclinações normais de operação. Para equipamentos que operam inclinados, aumentar.
- **COOLDOWN** (`COOLDOWN_ALARME_S`): aumentar se o evento gerar múltiplos frames (ex.: um tombo seguido de leituras ainda tombadas) e você quiser só um alarme por ocorrência.

---

## Limitações conhecidas

- **Pitch/roll são confiáveis só em repouso ou movimento quase constante**. Durante vibração ou aceleração, a gravidade se mistura com a aceleração do movimento. Por isso TRANCO tem prioridade.
- **Não detecta yaw** (rotação em torno do eixo vertical) — precisaria de giroscópio ou magnetômetro.
- **ADR não está ativo** (`LMIC_setLinkCheckMode(0)` e sem `LMIC_setAdrMode(1)`). Se o dispositivo é fixo, considere ligar ADR para a rede otimizar SF/potência.
- **ABP não negocia chaves**: contador de frames reinicia a cada boot — pode ser bloqueado por redes com verificação estrita de frame counter. Em caso de problema, resetar o contador na rede ou migrar para OTAA.