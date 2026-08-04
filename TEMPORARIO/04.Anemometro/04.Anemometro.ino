// Fio vermelho no 0V
// Fio preto no pino 21 do ESP32 com INPUT_PULLUP

#define ANEMO_PIN 21

volatile unsigned int pulseCount = 0;
void IRAM_ATTR pulsesCounter() { pulseCount++; }
unsigned long countingTime = 5000;

float windSpeedInKmh;

void setup()
{
  Serial.begin(115200);
  Serial.println("Iniciando leitura do anemômetro...");
  pinMode(ANEMO_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMO_PIN), pulsesCounter, RISING);
}

void loop()
{
  pulseCount = 0;
  delay(countingTime);

  unsigned int count = pulseCount;
  unsigned int rpm = (count*60)/(countingTime/1000);
  const float pi = 3.14159265;
  int radius = 147;
  float windSpeedInMs = ((4 * pi * radius * rpm) / 60) / 1000;
  windSpeedInKmh = windSpeedInMs*3.6;

  Serial.print("Pulsos em 5s: ");
  Serial.print(count);
  Serial.print(" | RPM: ");
  Serial.print(rpm);
  Serial.print(" | Velocidade: ");
  Serial.print(windSpeedInKmh, 2);
  Serial.println(" km/h");
}