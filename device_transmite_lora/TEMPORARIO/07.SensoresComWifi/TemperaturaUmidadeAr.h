#include "DHT.h"

#define DHTT_YPE DHT22
#define DHT_PIN 3 

DHT dht(DHT_PIN, DHTT_YPE);

float temperature;
float humidity;

void temperaturaUmidadeAr()
{
  dht.begin();
  delay(2000); // Delay necessário para o DHT22
  
  humidity = dht.readHumidity();
  temperature = dht.readTemperature(); // Celsius por padrão

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Falha ao ler do sensor DHT!");
    return;
  }

  // Mostra os valores no Serial Monitor
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.print(" °C\tUmidade: ");
  Serial.print(humidity);
  Serial.println(" %");
}
