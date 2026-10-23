#define SOIL_SENSOR_PIN 13

int moisturePercent;

void setup()
{
  Serial.begin(115200);
  Serial.println("Iniciando leitura do sensor de umidade do solo...");
}

void loop()
{
  // Leitura analógica (0 a 4095 no ESP32)
  int rawValue = analogRead(SOIL_SENSOR_PIN);

  // Converte para porcentagem (ajuste conforme seu sensor)
  moisturePercent = map(rawValue, 4095, 0, 0, 100); // seco = 4095, molhado = 0, minimo 0%, maximo 100%

  // Exibe os dados
  Serial.print("Valor bruto: ");
  Serial.print(rawValue);
  Serial.print(" | Umidade: ");
  Serial.print(moisturePercent);
  Serial.println(" %");

  delay(2000); // Aguarda 2 segundos entre leituras
}
