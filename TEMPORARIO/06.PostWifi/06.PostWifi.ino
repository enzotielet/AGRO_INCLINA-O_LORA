#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "Agrocity_2.4G";
const char* password = "Livinglab2025!";

const char* serverUrl = "http://mqtt.agrocitylivinglab.com.br/api/mqtt_data";
const char* tokenUrl = "http://mqtt.agrocitylivinglab.com.br/api/login";
const char* apiUsername = "admin@admin.com";
const char* apiPassword = "agro2admin";
String token = "";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -3 * 3600;
const int   daylightOffset_sec = 0;

String getToken()
{
  if (WiFi.status() != WL_CONNECTED) return "";

  HTTPClient http;
  http.begin(tokenUrl);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
  doc["email"] = apiUsername;
  doc["password"] = apiPassword;

  String body;
  serializeJson(doc, body);

  int httpCode = http.POST(body);
  if (httpCode == 200)
  {
    String payload = http.getString();
    StaticJsonDocument<1000> responseDoc;
    DeserializationError error = deserializeJson(responseDoc, payload);
    if (!error)
    {
      const char* accessToken = responseDoc["token"];
      http.end();
      return String(accessToken);
    }
    else Serial.println("Erro ao interpretar JSON de token");
  }
  else
  {
    Serial.print("Falha ao obter token, código HTTP: ");
    Serial.println(httpCode);
    Serial.println(http.getString());
  }
  http.end();
  return "";
}

void setup()
{
  Serial.begin(115200);
  Serial.println("BEGIN");
  Serial.println("Conectando a Wi-Fi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi!");

  // Buscar data e hora
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Erro ao obter hora com NTP");
    return;
  }
  Serial.println("Hora sincronizada:");
  Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");
  Serial.println();
}

void loop()
{
  // Obter token
  if (token == "")
  {
    Serial.println("Obtendo token...");
    token = getToken();
    if (token == "") return; // Abort if still failed
  }

  // Ler data e hora
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Erro ao obter hora!");
    return;
  }
  char isoDateTime[25];
  strftime(isoDateTime, sizeof(isoDateTime), "%Y-%m-%dT%H:%M:%S", &timeinfo);

  // Construir pacote de dados
  StaticJsonDocument<1000> innerDoc;
  innerDoc["datetime"] = isoDateTime;
  innerDoc["status"] = 1;
  innerDoc["datatype"] = "BATATA!";
  String innerJson;
  serializeJson(innerDoc, innerJson);
  StaticJsonDocument<1000> outerDoc;
  outerDoc["topic"] = "JUSTISSA";
  outerDoc["payload"] = innerJson;
  String postData;
  serializeJson(outerDoc, postData);

  // Conectar no servidor e enviar dados
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + token);
    int httpCode = http.POST(postData);
    Serial.print("Código de resposta: ");
    Serial.println(httpCode);
    if (httpCode > 0)
    {
      String response = http.getString();
      Serial.println("Resposta:");
      Serial.println(response);
    }
    else Serial.println("Falha no envio HTTP POST");
    Serial.println();
    http.end();
  }
  else Serial.println("WiFi desconectado.");

  // Delay entre envios
  delay(5000);
}
