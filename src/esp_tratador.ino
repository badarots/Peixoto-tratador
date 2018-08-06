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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (byte i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(output_pins[0], LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(output_pins[0], HIGH);  // Turn the LED off by making the voltage HIGH
  }
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
    mqtt_client.publish("tratador","hello world");
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
  // attachInterrupt(digitalPinToInterrupt(input_pins[0]), interPresenca, RISING);
  // attachInterrupt(digitalPinToInterrupt(input_pins[1]), interMotor, CHANGE);

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // configuracao do wifi
  WiFi.mode(WIFI_STA);

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

  // controla pulso
  if (pulse_pin != -1 && millis() >= pulse_end) {
    digitalWrite(pulse_pin, LOW);
    pulse_pin = -1;
  }

  // //---------------------------------------------//
  // //      lida com pinos de entrada
  // //---------------------------------------------//
  // // a flag eh levantada com a mudanca LOW -> HIGH
  // // pino LOW: sem presenca, HIGH: presenca detectada
  // if (presenca_flag && (millis() - presenca_temp) > sendGET_tempmin) {
  //   presenca_temp = millis();
  //   Serial.println("interrupt presenca");
  //   String resposta = sendGET("?tratador_presenca=true");
  //   // o programa vai continuar reeviando a msg ate receber uma resposta do servidor
  //   if (resposta = "OK") presenca_flag = false;
  // }
  
  // // a flag eh levantada com a mudanca do estado do pino
  // // pino LOW: motor ligado, HIGH: desligado
  // if (motor_flag && (millis() - motor_temp) > sendGET_tempmin) {
  //   motor_temp = millis();
  //   Serial.println("interrupt motor");
  //   boolean motor_state = digitalRead(input_pins[1]);
    
  //   String parametros;
  //   if (motor_state) parametros = "?tratador_motor=desligado";
  //   else parametros = "?tratador_motor=ligado";
  //   String resposta = sendGET(parametros);
  //   if (resposta = "OK") motor_flag = false;
  // }

  // //---------------------------------------------//
  // //      lida com pedidos ao servidor
  // //---------------------------------------------//
  // WiFiClient client = server.available();
  // if (!client) return;

  // // Wait until the client sends some data
  // Serial.println("new client");
  // unsigned long temp = millis();
  // while (!client.available()) {
  //   delay(1);
  //   // impede do loop travar se o cliente nao terminar o pedido (?)
  //   if ((millis() - temp) > connection_timeout) {
  //     Serial.println("conexao fechada: excedido tempo limite");
  //     client.stop();
  //     return;
  //   }
  // }

  // // Read the first line of the request
  // String req = client.readStringUntil('\r');
  // Serial.println(req);
  // client.flush();

  // // formata pedido
  // // formato esperado: /senha/funcao/opcoes
  // String pedido[N_PARAMETROS];
  // int i = req.indexOf('/');

  // for (int k = 0; k < N_PARAMETROS && i != -1; k++) {
  //   int f = req.indexOf('/', i + 1);
  //   int f2 = req.indexOf(' ', i + 1);
  //   if (f2 > -1 && f2 < f) f = f2;
  //   pedido[k] = req.substring(i + 1, f);
  //   i = f;
  // }
  // //for (int k = 0; k < N_PARAMETROS && 1; k++) Serial.println(pedido[k]);

  // // processa pedido
  // String resp;
  // if (pedido[0] == senha) {

  //   if (pedido[1] == "tratar") {
  //     if (pedido[2].indexOf("freq") == 0 ) {
  //       int freq = pedido[2].substring(4).toInt();
  //       if (freq < FREQ_MIN || freq > FREQ_MAX) {
  //         resp = "frequencia nao permitida: " + String(freq) + 
  //             "\nfaixa permitida: " + String(FREQ_MIN) + '-' + String(FREQ_MAX);
  //       } else {
  //         setFrequency(FREQ_PIN, freq); //configura velocidade do motor
  //         delay(100); //espera o inversor ler corretamente a velocidade
  //         setPulse(PULSO_PIN, 1000); //gera pulso que inicia o ciclo de tratamento
  //         resp = "tratamento freq=" + String(freq);
  //       }

  //     }
  //   } else if (pedido[1] == "envia") {
  //     resp = sendGET("?oi=true");

  //   } else if(pedido[1] == "presenca") {
  //     presenca_flag = true;
  //     resp = "flag de prefenca levantada";

  //   } else if(pedido[1] == "motor") {
  //     motor_flag = true;
  //     resp = "flag do motor levantada";
  //   }
  //   else if (pedido[1].indexOf("pin") == 0) {
  //     int pin = pedido[1].substring(3).toInt();
  //     //tipos de pinos:
  //     //1: output, 2: input, 3: invalido
  //     int pin_type = pinType(pin);

  //     if (pedido[2] == "on" && pin_type == 1) {
  //       digitalWrite(pin, HIGH);
  //       resp = "ligado pino " + String(pin);

  //     } else if (pedido[2] == "off" && pin_type == 1) {
  //       digitalWrite(pin, LOW);
  //       resp = "desligado pino " + String(pin);

  //     } else if (pedido[2] == "estado" && pin_type) {
  //       boolean estado = digitalRead(pin);
  //       resp = "pino " + String(pin) + "=" +
  //              String(estado);

  //     } else if (pedido[2].indexOf("pulso") == 0 && pin_type == 1) {
  //       int pulso = pedido[2].substring(5).toInt();
  //       if (pin > 1 && pin < 14 && pulse_pin == -1 ) {
  //         setPulse(pin, pulso);

  //         resp = "pulso pino " + String(pin) + "=" +
  //                String(pulso);
  //       }

  //     } else if (pedido[2].indexOf("freq") == 0 && pin_type == 1) {
  //       int freq = pedido[2].substring(4).toInt();
  //       setFrequency(pin, freq);

  //       resp = "frequencia pino " + String(pin) + "=" +
  //              String(freq);
  //     }
  //   }
  // } else resp = "Senha incorreta";

  // if (resp == "") resp = "Pedido invalido";
  // Serial.println(resp);
  // if (resp == "Senha incorreta") {
  //   client.stop();
  //   return;
  // }

  // client.flush();

  // // Prepare the response
  // String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n";
  // s += resp;
  // s += "</html>\n";

  // // Send the response to the client
  // client.print(s);
  // delay(1);
  // Serial.println("Client disconnected");


  // // The client will actually be disconnected
  // // when the function returns and 'client' object is detroyed
}
