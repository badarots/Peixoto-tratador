#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// configuraçoes para o tratador
#define FREQ_PIN 16
#define FREQ_MIN 500
#define FREQ_MAX 2500
#define FREQ_STD 1500
#define PULSO_PIN 14
#define N_PARAMETROS 3  //numero de parametros em um pedido

// variável do wifi
const char* ssid = "PEIXECAIDO";
const char* pass = "informatica";
const unsigned long wifi_rcinterval = 1000;
unsigned long wifi_lastrc;
WiFiClient wclient;
char msg[50];

//variável do cliente MQTT
const char* mqtt_server = "192.168.1.70";
const unsigned long mqtt_rcinterval = 5000;
unsigned long mqtt_lastrc;
PubSubClient mqtt_client(wclient);

// variávels para configurações dos pinos
const byte output_n = 3;
const int output_pins[] = {16, 14, 2};
const byte input_n = 2;
const int input_pins[] = {1, 12, 13};

// variavaies de controle de inputs
volatile boolean presenca_flag;
unsigned long presenca_temp;
volatile boolean motor_flag;
unsigned long motor_temp;
const unsigned int sendGET_tempmin = 1000; //tempo minimo entre dois envios em ms

// variaveis para controle de pulsos
int pulse_pin = -1;
unsigned long pulse_end;

int pinType(int pin) {
  int type = 0;
  for (unsigned int i = 0; i < output_n; i++)
    if (pin == output_pins[i]) type = 1;

  for (unsigned int i = 0; i < input_n; i++){
    if (pin == input_pins[i]) type = 2;
  }
  
  return type;
}

void setFrequency(int pin, int freq) {
  analogWriteFreq(freq);
  analogWrite(pin, 512);
}

void setPulse(int pin, unsigned long pulse) {
  pulse_pin = pin;
  pulse_end = millis() + pulse;
  digitalWrite(pulse_pin, HIGH);
}

void interPresenca() {
  presenca_flag = true;
}

void interMotor() {
  motor_flag = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.

  // Allocate the correct amount of memory for the payload copy
  byte* p = (byte*)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  mqtt_client.publish("tratador", p, length);
  // Free the memory
  free(p);
}

void reconnectWifi() {
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println("...");
  WiFi.begin(ssid, pass);

if (WiFi.waitForConnectResult() != WL_CONNECTED)
  return;
Serial.println("WiFi connected");
}

boolean reconnectMQTT() {
  Serial.println("Connecting to the MQTT broker...");
  if (mqtt_client.connect("tratador")) {
    // Once connected, publish an announcement...
    mqtt_client.publish("tratador","hello world", true);
    // ... and resubscribe
    mqtt_client.subscribe("controlador");
  }
  return mqtt_client.connected();
}

void setup()  {
  //configuração dos pinos
  for (int i = 0; i < output_n; i++) {
    pinMode(output_pins[i], OUTPUT);
  }
  for (int i = 0; i < input_n; i++) {
    pinMode(input_pins[i], INPUT_PULLUP);
  }

  // configura freq padrao
  setFrequency(FREQ_PIN, FREQ_STD);

  // configura interruptor dos sinais de entrada
  attachInterrupt(digitalPinToInterrupt(input_pins[0]), interPresenca, RISING);
  attachInterrupt(digitalPinToInterrupt(input_pins[1]), interMotor, CHANGE);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // configuracao do wifi
  WiFi.mode(WIFI_STA);

  // configuração do MQTT 
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(callback);

  //---------------------------------------------//
  //      configuracao do OTA
  //---------------------------------------------//

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("esptratador");

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // lida com OTA
  ArduinoOTA.handle();

  // reconecta ao wifi
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if(now - wifi_lastrc > wifi_rcinterval) {
      reconnectWifi();
      wifi_lastrc = now;
    }
  }

  // reconecta ao servido MQTT
  if (WiFi.status() == WL_CONNECTED && !mqtt_client.connected()) {
    unsigned long now = millis();
    if (now - mqtt_lastrc > mqtt_rcinterval) {
      reconnectMQTT();
      mqtt_lastrc = now;
    }
  }

  // executa loop do MQTT 
  if (mqtt_client.connected()) mqtt_client.loop();

  // controla pulso
  if (pulse_pin != -1 && millis() >= pulse_end) {
    digitalWrite(pulse_pin, LOW);
    pulse_pin = -1;
  }
}
