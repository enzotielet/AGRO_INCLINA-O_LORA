// Verde no 3.3V
// Amarelo no pino 35 com resistor de pulldown de 10K

// Leituras estáveis com alimentação de 3.3V
// 300 pra SE
// 350 pra E
// 420 pra NE
// 520 pra N
// 650 pra NO
// 850 pra O
// 1190 pra SO
// 1850 pra S

#define WINDDIRECTION_PIN 35

String windDirection;

void direcaoVento()
{
  pinMode(WINDDIRECTION_PIN, INPUT);

  int samples = 1000;
  unsigned long samplesSum = 0;
  for (int i = 0; i < samples; i++) samplesSum = samplesSum + analogRead(WINDDIRECTION_PIN);
  int windSingal = samplesSum/samples;

  if (windSingal < 100) windDirection = "";
  else if (windSingal < 325) windDirection = "SE";
  else if (windSingal < 385) windDirection = "E";
  else if (windSingal < 470) windDirection = "NE";
  else if (windSingal < 590) windDirection = "N";
  else if (windSingal < 750) windDirection = "NO";
  else if (windSingal < 1000) windDirection = "O";
  else if (windSingal < 1500) windDirection = "SO";
  else if (windSingal < 2000) windDirection = "S";
  else windDirection = "";

  if (windDirection != "")
  {
    Serial.print("Direção do vento: ");
    Serial.print(windDirection);
    Serial.print("\t");
  }
  else Serial.print("Falha no sensor\t");
  Serial.println(windSingal);
}
