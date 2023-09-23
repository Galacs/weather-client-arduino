#include <ArduinoJson.h> // Permet de créer des documents JSON pour l'envoie vers MQTT    // https://github.com/bblanchon/ArduinoJson
#include "esp_system.h" // Fonctions spécifique à l'ESP-IDF (pour le WTD)                 // ESP-IDF
#include <WiFiClientSecure.h> // Client wifi avec TLS                                     // ESP-IDF
#include <Adafruit_Sensor.h> // DHT*                                                      // https://github.com/adafruit/Adafruit_Sensor
#include <DHT.h>                                                                          // https://github.com/adafruit/DHT-sensor-library
#include <DHT_U.h>                                                                        // https://github.com/adafruit/DHT-sensor-library
#include <Adafruit_BMP085.h> // BMP180                                                    // https://github.com/adafruit/Adafruit_BMP085_Unified
#include "SparkFun_SGP30_Arduino_Library.h" //SGP30                                       // https://github.com/sparkfun/SparkFun_SGP30_Arduino_Library
#include <PubSubClient.h> // MQTT                                                         // https://github.com/knolleary/pubsubclient

// Informations de connection Wifi
const char* ssid = "wifi-ndsf";
const char* password = "ndsf2021PS";

// Identifiants serveur MQTT
const char* mqttServer = "mqtt.weather.ait-younes.fr";
const int mqttPort = 1883;
const char* mqttUser = "user";
const char* mqttPassword = "";

// CA Let's encrypt pour MQTT TLS. cf: https://letsencrypt.org/certificates/
const char* root_ca = \
      "-----BEGIN CERTIFICATE-----\n" \
      "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n" \
      "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n" \
      "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n" \
      "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n" \
      "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n" \
      "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n" \
      "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n" \
      "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n" \
      "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n" \
      "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n" \
      "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n" \
      "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n" \
      "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n" \
      "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n" \
      "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n" \
      "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n" \
      "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n" \
      "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n" \
      "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n" \
      "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n" \
      "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n" \
      "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n" \
      "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n" \
      "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n" \
      "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n" \
      "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n" \
      "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n" \
      "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n" \
      "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n" \
      "-----END CERTIFICATE-----";

// Function executé quand un message est recu depuis MQTT (NA)
void callback(char* topic, byte* payload, unsigned int length) { }

WiFiClientSecure wifiClient;
PubSubClient client(mqttServer, mqttPort, callback, wifiClient);

// Fonction
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "SF1-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(),mqttUser,mqttPassword)) {
      digitalWrite(13, HIGH);
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

DHT_Unified dht(18, DHT11); // DHT11
DHT_Unified dht2(19, DHT22); // DHT22
Adafruit_BMP085 bmp; // BMP180
SGP30 air_sensor; // SGP30

static int led_pins[] = {12, 13, 14, 27, 25}; // Pins des leds de status
bool wifiMode; // Variable determinant si le mode wifi est activé

// Fonction qui reset le microcontrolleur
void restart() {
  for (size_t i = 0; i < 4; i++) {
    digitalWrite(led_pins[i], LOW); // Eteindre les leds de status d'abord.
  }
  ESP.restart();
}

// Fonction de "watchdog" 1: reset le MCU si le wifi est deconnécté depuis plus de 15 secondes
void wdt(void * a) {
  uint32_t lastping = millis();
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
        lastping = millis();
    } else if (lastping - millis() > 15000) {
      restart();
    }
    delay(15000);
  }
}

const int wdtTimeout = 10000;
hw_timer_t *timer = NULL;

void setup() {
  // Watchdog materiel fournit par l'ESP32. cf: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &restart, true);
  timerAlarmWrite(timer, wdtTimeout * 1000, false);
  timerAlarmEnable(timer);
  
  // Lance la fonction de watchdog définit plus haut dans une tache parallele à la principale 
  xTaskCreate(wdt, "wdt", 10000, NULL, 1, NULL); // cf: https://www.freertos.org/a00125.html

    
  Serial.begin(115200); // Initialization de la connection serie à 115200 bauds (standard pour l'esp32)
  // Configure tout les pins de leds en entrées.
  for (size_t i = 0; i < 4; i++) {
    pinMode(led_pins[i], OUTPUT);
  }

  // Entrée pour l'interupteur de selecteur wifi (NA)
  pinMode(26, INPUT_PULLDOWN);

  // Lit son état dans wifiMode
  wifiMode = digitalRead(26) != HIGH;
  // Allume une led si le wifi est desactivé
  if(!wifiMode) {
    Serial.println("Wifi is off");
    digitalWrite(25, HIGH);
  } else {
    digitalWrite(25, LOW);
  }

  // Reset le microcontrolleur si l'état de l'interupteur change
  attachInterrupt(digitalPinToInterrupt(26), restart, CHANGE);
  
  if(wifiMode) {
    WiFi.persistent(false); // Evite d'écrire les info wifi dans la NVRAM pour ne pas la corrompre a force d'écrire
    WiFi.begin(ssid, password);
    // Attendre que le wifi se connecte.
    while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      digitalWrite(12, HIGH);
      delay(150);
      digitalWrite(12, LOW);
      delay(50);
    }
    digitalWrite(12, HIGH);
    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
    WiFi.setAutoReconnect(true);
    configTime(1 * 60 * 60, 0, "pool.ntp.org", "0.fr.pool.ntp.org", "2.fr.pool.ntp.org"); // Configuration de l'horloge pour TLS
    wifiClient.setCACert(root_ca); // CA
    // wifiClient.setInsecure(); // Pour desactiver la verification de CA
    reconnect(); // Connection a MQTT
  }

  client.setBufferSize(512);

  // Init BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  // Init SGP30
  if (air_sensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    while (1);
  }
  air_sensor.initAirQuality();

  // INIT DHT*
  dht.begin();
  dht2.begin();
}

void loop() {
  unsigned long time_now = millis(); // pour espacer chaque mesure de 1 seconde
  digitalWrite(14, HIGH); // Led indication prise de mesures en cours
  
  DynamicJsonDocument doc(1024); // document JSON comprenant toutes les mesures

  sensors_event_t event; // memoire pour contenire les info des capteurs DHT
  dht.temperature().getEvent(&event);
  doc["temperature_dht11"] = event.temperature;
  dht.humidity().getEvent(&event);
  doc["humidity_dht11"] = event.relative_humidity;

  dht2.temperature().getEvent(&event);
  doc["temperature_dht22"] = event.temperature;
  dht2.humidity().getEvent(&event);
  doc["humidity_dht22"] = event.relative_humidity;

  // Compensation SGP30
  double absHumidity = RHtoAbsolute(event.relative_humidity, event.temperature);
  uint16_t sensHumidity = doubleToFixedPoint(absHumidity);

  // Mesures BMP180
  doc["temperature_bmp180"] = bmp.readTemperature();
  doc["pressure_bmp180"] = bmp.readPressure();
  doc["sealevel_pressure_bmp180"] = bmp.readSealevelPressure(115);

  // SGP30
  air_sensor.measureAirQuality();
  air_sensor.measureRawSignals();

  doc["co2_sgp30"] = air_sensor.CO2;
  doc["tvoc_sgp30"] = air_sensor.TVOC;
  doc["h2_sgp30"] = air_sensor.H2;
  doc["ethanol_sgp30"] = air_sensor.ethanol;


  // UV sensor (NA)
  /*float sensorVoltage; 
  float sensorValue;
  analogReadResolution(11);
  analogSetAttenuation(ADC_6db);
  sensorValue = analogRead(34);
  sensorVoltage = sensorValue/1024*5.0;
  Serial.print("sensor reading = ");
  Serial.print(sensorValue);
  Serial.println("");
  Serial.print("sensor voltage = ");
  Serial.print(sensorVoltage);
  Serial.println(" V");*/

  // Capteur pluie
  doc["rain_rainsensor"] = (4095 - analogRead(33))/4095.0;

  // Signal Wifi
  doc["wifi_esp32"] = WiFi.RSSI();

  digitalWrite(14, LOW); // enteinte de la led de de prise de mesures

  // buffer contenant le doc JSON serializé
  char buffer[512];
  size_t n = serializeJson(doc, buffer);
  Serial.println(buffer); // Écriture des données sur la connection serie
  // Reconnection si celle-ci fut perdu
  if(wifiMode) {
    if (!client.connected()) {
      digitalWrite(13, LOW);
      reconnect();
  }
  digitalWrite(27, HIGH); // Allumage led d'envoi wifi
  client.publish("weather/sf1", buffer, n); // Envoie du JSON vers mqtt
  timerWrite(timer, 0); // Remise a zero du wtd
  digitalWrite(27, LOW);
  }

  // Debug serial
  Serial.print("Loop took: ");
  Serial.print(millis() - time_now);
  Serial.print("ms   //    ");
  Serial.print("Wifi: ");
  Serial.print(WiFi.RSSI());
  Serial.println("dBm");

  // Delaie de 1 seconde entre chaque loop
  while(millis() < time_now + 1000){delay(10);}
}

// Fonctions de compensation
double RHtoAbsolute (float relHumidity, float tempC) {
  double eSat = 6.11 * pow(10.0, (7.5 * tempC / (237.7 + tempC)));
  double vaporPressure = (relHumidity * eSat) / 100;
  double absHumidity = 1000 * vaporPressure * 100 / ((tempC + 273) * 461.5);
  return absHumidity;
}
uint16_t doubleToFixedPoint( double number) {
  int power = 1 << 8;
  double number2 = number * power;
  uint16_t value = floor(number2 + 0.5);
  return value;
}