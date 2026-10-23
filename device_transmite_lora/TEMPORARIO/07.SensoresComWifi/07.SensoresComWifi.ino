#include "TemperaturaUmidadeAr.h"
#include "UmidadeSolo.h"
#include "Anemometro.h"
#include "DirecaoVento.h"
#include "PostWifi.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Leitura de sensores com envio via Wi-Fi");
  Serial.println("\tTemperatura e umidade do ar");
  Serial.println("\tUmidade do solo");
  Serial.println("\tVelocidade do vento");
  Serial.println("\tDireção do vento");
  Serial.println();
  setupWifi();
}

void loop()
{
  Serial.println("==================================================");
  temperaturaUmidadeAr();
  umidadeSolo();
  anemometro();
  direcaoVento();
  postWifi();
  delay(2000);
}
