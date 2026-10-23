#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DHT.h>
#include "conf.h"

//Descomentar se usar cayenne
#include <CayenneLPP.h>


//Variáveis globais
//contador de transmissões/quadros
int contador = 0;
//Usado para diferenciar par e ímpar
int resto = 0;
// Indicador de recebimento de dados da TTN
bool recebido = false;
//Variável que armazena os dados recebidos da TTN
//char buffer[51]; 


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Define se usa buffer do Cayenne ou de byte.
#ifdef cayenne 
  CayenneLPP lpp(51);
#else
  static uint8_t  mydata[51];
#endif
//Talvez deixar só o sendjob
//static osjob_t initjob, sendjob, blinkjob;
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 6; //Padrão 60

#ifdef heltec
//Pin mapping heltec
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 35, 34}, //antigo 26,33,32
};
#else
// Pin mapping Dragino
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
}; 
#endif

//Função para esrever em tela
void escreveHeltec(String msg, int x ,int y){
  Heltec.display->drawString(x, y, msg);
  //Heltec.display->drawString(x, y, "teste");
  Heltec.display->display();
  return;
}

//Função para escrever que não foi localizado dado dos sensores
void escreveNaoLocalizado(){
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "Algum sensor");
  Heltec.display->drawString(0, 15, "não foi");
  Heltec.display->drawString(0, 30, "localizado!");
  Heltec.display->display();
  return;
}

String textoFrequencia(){
  Heltec.display->setFont(ArialMT_Plain_10);
        String texto = "Frq: ";
        texto = texto + String((LMIC.freq)/1000);
        texto = texto + " kHz - ";
        texto = texto + String(contador);
  return texto;
}

//Função para ler e retornar valor do sensor de umidade
float lerUmidade(){
  return dht.readHumidity();
  //return h;
}

//Função para ler e retornar valor do sensor de temperatura
float lerTemperatura(){
   // Armazena dados da temperatura, umidade e luz, se houver
  float t = dht.readTemperature();
  return t;
}

//Função para ler e retornar valor do sensor de luminosidade
//Valores retornados em binário
int lerLuminosidade(){
  int luz = digitalRead(PINO_LUZ);
  return luz;
}


//Função que recebe dados
char* recebeDados(){
  //Se recebeu dados, grava eles na variável dados_recebidos
  // que transmitirá via Lora o que foi recebido
  // da TTN e seta varíavel booleana da recebimento para false
  //char *buffer = (char *) *dados_recebidos;

  Serial.println("==========================================");
  Serial.print("Dados Recebidos: ");
  Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
  //Armazena dados recebidos na variável dados_recebidos e seta
  // variável booleana de recebimento de dados da TTN para true 
  char* dados_recebidos = (char *) LMIC.frame + LMIC.dataBeg + LMIC.dataLen;
  Serial.println(dados_recebidos);
  recebido = true;
  Serial.println("==========================================");
  Serial.println();
  //Converte String para array de char
  //char* dados_recebidos_char = &dados_recebidos[0];
  return dados_recebidos;

}


void do_send(osjob_t* j) {

  // Parâmetros DHT
  // Cria string que armazenará os dados de temperatura e umidade
    
  String myString;

  #ifdef heltec
    //Limpa display
    Heltec.display->clear();
  #endif
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    
    if (recebido){
        myString = (String) recebeDados();
    }
    //Se não medir temperatura e umidade escreve msg de erro
    // Talvez criar função aqui
    else if (isnan(lerUmidade()) || isnan(lerTemperatura())) {
      myString = "Sem dados de algum sensor!";
    }
    else {
      //Usa Mystring para formar um único texto para escrita
      //na console e transmissão para LoRa
      
      myString = "Temp: ";
      myString = myString + String(lerTemperatura());
      myString = myString + " ºC - ";
      myString = myString + "Umid: ";
      myString = myString + String(lerUmidade());
      myString = myString + " % - ";
      myString = myString + " Luz: ";
      #ifdef heltec
        escreveHeltec(myString, 0, 50);
      #endif
    }

    //Printa na Serial
    Serial.println(myString);

    #ifdef cayenne
    lpp.reset();
    //lpp.addTemperature(1, lerTemperatura());
    //lpp.addRelativeHumidity(3, lerUmidade());
    //lpp.addLuminosity(2,lerLuminosidade());
    #else
      //Converte para const void para copair para memória do LoRa
      const void * text = myString.c_str();

      //Copia dados para memória do LoRa
      memcpy(mydata, text, sizeof(mydata));
    #endif
    
    #ifdef cayenne
      LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    #else
      // Prepare transmission at the next possible time.
      LMIC_setTxData2(1, mydata, strlen((char*) mydata), 0); // 1 envia ACK
    #endif
    //LMIC_setTxData();
    Serial.println(); 
    Serial.println("Packet queued");
    Serial.print("TX nº: ");
    Serial.println(contador);
    #ifndef cayenne
      Serial.println((char*)mydata);
    #endif
    Serial.println(LMIC.freq);

    
    //Verifica se dados foram recebidos
    //em caso positivo, escreve no display
    //e altera valor de recebido para false
    if (recebido){
      #ifdef heltec
        Heltec.display->setFont(ArialMT_Plain_10);
        Heltec.display->drawString(0, 0, (String) recebeDados());
        Heltec.display->drawString(0, 50, textoFrequencia());
        Heltec.display->display();
        //Seta para não recebido.
        recebido = false;
      #endif
    }

    // se não foi recebido
    // testa se retorno do sensor é valido, 
    // caso contrário algo está errado.
    else if (isnan(lerTemperatura()) || isnan(lerUmidade())) 
    {
      Serial.println("Dados de temperatura e umidade não localizados");
      #ifdef heltec
        escreveNaoLocalizado();
        Heltec.display->drawString(0, 50, textoFrequencia());
        Heltec.display->display();
      #endif
    }
    //se dados não foram recebidos e está tudo OK com sensor
    //escreve temp e umidade na tela 
    else
    {
      #ifdef heltec

        //Mostra os dados no display
        //Talvez gerar função específica
        myString = String(lerTemperatura());
        myString = myString + " ºC";
        Heltec.display->setFont(ArialMT_Plain_10);
        Heltec.display->drawString(0, 0, "Temperatura");
        Heltec.display->setFont(ArialMT_Plain_24);
        Heltec.display->drawString(0, 10, myString);
        myString = "Umidade: ";
        myString = myString + String(lerUmidade());
        myString = myString + " %";
        Heltec.display->setFont(ArialMT_Plain_10);
        Heltec.display->drawString(0, 32, myString);
        Heltec.display->setFont(ArialMT_Plain_10);
        myString = "Frq: ";
        myString = myString + String((LMIC.freq)/1000);
        myString = myString + " kHz - ";
        myString = myString + String(contador);
        Heltec.display->drawString(0, 42, myString);
        myString = "Luminosidade: ";
        myString = myString + String(lerLuminosidade());
        Heltec.display->drawString(0, 52, myString);
        Heltec.display->display();
      #endif
    }

    //incrementa contador
    contador++;
  }
}
void onEvent (ev_t ev) {
  //sleep(5);
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.print("EV: ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      //LMIC_sendAlive();
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
      //LMIC_setPingable(1);
      //Serial.println("SCANNING...");
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
      
      //Se recebeu dados, chama função específica, senão escreve info na console.
      if (LMIC.dataLen) {
        //Rever        
        //char* dados_recebidos = recebeDados();
        //Seta flag de recebido
        recebido = true;

      }else{
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

void setup() {

  //Inicialza pinagem luminosidade
  #ifdef heltec
    pinMode (PINO_LUZ, INPUT);
  #endif

  #ifdef heltec
    //Inicialização Display
    Heltec.begin(true, false, true);
    Heltec.display->setContrast(255);
    Heltec.display->clear();
    Heltec.display->setFont(ArialMT_Plain_16);
    Heltec.display->drawString(0, 0, "Ligando sensor...");
    Heltec.display->display();
  #endif

  //Inicialização console
  Serial.begin(115200);
  Serial.println("DHTxx test!");
  
  //Inicialização DHT
  dht.begin();

  //Inicialização Lora
  while (!Serial);
  delay(5000);
  Serial.println("Starting...");
  
#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
 #else
   // If not running an AVR with PROGMEM, just use the arrays directly
   LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
 #endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF10, 14); // Ver se GW está no 10; 14 é 14dBM

  #ifdef canal_unico
    //Deixa canal único
    for (int i = 1; i < 64; i++)
    {
      LMIC_disableChannel(i);  // only the first channel 902.3Mhz works now.
      Serial.println("Desabilitando canal ");
    }
  #endif
  
  #ifdef ttn_caxias_915
    //Desabilita os canais desnecessários dos Gateways de Caxias.
    for (int i = 0; i < 7; i++)
    {
      LMIC_disableChannel(i);  
      Serial.println("Desabilitando canal ");
    }
    
    for (int i = 16; i < 63; i++) //alterado i de 15 para 16 para fechar envios a cada 10 min
    {
      LMIC_disableChannel(i);  
      Serial.println("Desabilitando canal ");
    }
  #endif

  #ifdef ttn_caxias
    //Original. Tem que alterar lorabase.h para usar essa sequencia
    for (int i = 0; i < 8; i++)
    {
    LMIC_disableChannel(i);  
    Serial.println("Desabilitando canal ");
    }

    for (int i = 16; i < 71; i++)
    {
    LMIC_disableChannel(i);  
    Serial.println("Desabilitando canal ");
    }
  #endif

  // Start job
  do_send(&sendjob);
  
}

void loop() {
  os_runloop_once();
}
