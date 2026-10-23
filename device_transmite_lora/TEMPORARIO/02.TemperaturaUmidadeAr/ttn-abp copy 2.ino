#include <Arduino.h>
#include <ArduinoJson.h>
#include <CayenneLPP.h>
CayenneLPP lpp(51);
int i = 0;
int c = 0;
int j = 0;
int timer = 0;
int mensagem[300];
int var = 0;

void setup() {
  Serial.begin(115200); // Monitor
  Serial1.begin(115200, SERIAL_8N1, 13, 2); // RX=16, TX=17
  delay(1000);
  
}

void mandamsg(int i,char buffer[]){

}

void processJson(const String &jsonStr) {
  Serial.println("JSON recebido:");
  Serial.println(jsonStr);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);

  if (error) {
    Serial.print("Erro no parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
   // lpp.reset();

  // Adiciona os campos (ajuste canais conforme quiser)
  int temperatura_ar;
  temperatura_ar      = doc["temperatura_ar"];
  lpp.addTemperature(1, doc["temperatura_ar"]);
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
int tamanhoPayload;
uint8_t payloadRecebido[51];
bool novoPayloadPronto = false;
tamanhoPayload = lpp.getSize();
memcpy(payloadRecebido, lpp.getBuffer(), tamanhoPayload);
novoPayloadPronto = true;
Serial.println("temperatura_ar:");
Serial.println(temperatura_ar);

Serial.println("→ Payload LPP armazenado para transmissão LoRa.");
}

void interptetaMensagem(){

}
void loop() {
  StaticJsonDocument<256> doc;

  doc["temperatura_ar"] = 12.34;
 // doc["umidade_ar"] = 12.34;
 // doc["umidade_solo"] = 12.34;
 // doc["precipitacao"] = 12.34;
 // doc["chovendo"] = 1;
 // doc["velocidade_do_vento"] = 12.34;
 // doc["direcao_do_vento"] = "SE";
 // doc["pressao_do_ar"] = 12.34;
 // doc["pressao_altitude"] = 12.34;
 // doc["co"] = 12.34;
 // doc["nh3"] = 12.34;
 // doc["no2"] = 12.34;
 // doc["co2"] = 12.34;
 // doc["tensao_alimentacao"] = 12.34;
 // doc["tempo_ligado"] = millis() / 1000; // contador em segundos

  char buffer[512];
  size_t len = serializeJson(doc, buffer);

/*
  Serial.println("Enviando JSON pela UART2:");
 Serial.println(buffer);
  Serial.println(len);
  Serial.write(buffer);
  Serial.write("\n");
  Serial1.write(buffer[j]);
  Serial.write("\n");
  Serial.write(buffer[j]);
  Serial.write("\n");
  j++;
  if(j==strlen(buffer)){
    Serial.println("Mensagem enviada completamente.");
    j=0;
    delay(1000);
  }
*/
  static String inputString = "";
  while (Serial1.available())
  {
    char c = Serial1.read();
    Serial.print(c);
    if (c != '\n') inputString += c;
    else
    {
      Serial.println(inputString);
      inputString = "";
    }
    if (!Serial1.available()) delay(10);
  }





 delay(200); // envia a cada 2s
}
