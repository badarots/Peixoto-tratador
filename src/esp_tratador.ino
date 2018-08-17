/*
Pinos ligados à placa:
saídas: 4, 5, 14, 16
relês: 14, 4(?)
entradas: 12, 13

Pinos em uso:
serial 13(RX) e 15(TX)
saida de frequência: 16
pulso de inicio de ciclo: 14
desabilita temporizador externo: 4(?)
estado do ciclo: 12
presença: 1(?)
*/

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
#define PULSE_PIN 14
#define PULSE_MAX 10000
#define PULSE_MIN 1

#define MOTOR_PIN 12
#define ATIVO_PIN 4
#define PRESENCA_PIN 1
#define MOTOR2_PIN 13
#define PRESENCA2_PIN 3


// classe para monitorar pinos de entrada
class InterruptMonitor
{
	// Class Member Variables

  // variáveis internas
  volatile bool flag;   // flag que avisa quando há mudança de estado
  bool do_read;         // flag que marca o agendamento de um leitura atrasada
  bool last_read;       // estado da última leitura
  unsigned long last_schedule;  // último agendamento da leitura

  public:
  // variáveis de configuração
	byte pin;             // numero do pino de entrada
  unsigned int delay;   // atraso em ms entre o agendamento e a leitura

  // Constructor - cria o monitor
  // and initializes the member variables and state
  InterruptMonitor(byte input_pin, unsigned int msdelay)
  {
	pin = input_pin;
	pinMode(pin, INPUT_PULLUP);

  delay = msdelay;
  flag = do_read = false;
  last_read = digitalRead(pin);
  last_schedule = 0;
  }

  // função que levanta a flag de mudança de estado
  // deve ser chama externamente, a flag gera um agendamendo no proxumo update
  void raiseFlag() {
    flag = true;
  }

  int update() {
    // variável que guarda resposta
    // -1: não houve mudanças, 0: pino mudou para low, 1: pino mudou para high
    int resp = -1;

    // agenda leitura atrasada:
    if (flag) {
      last_schedule = millis();
      flag = false;
      do_read = true;
    }
    // executa leitura atrasada
    if (do_read && millis() - last_schedule > delay) {
      do_read = false;
      bool read = digitalRead(pin);

      if (read != last_read) resp = last_read = read;
    }
    return resp;
  }
};

//---------------------------------------------//
//            VARIÁVEIS GLOBAIS
//---------------------------------------------//
// variável do wifi
const char* ssid = "PEIXECAIDO";
const char* pass = "informatica";
const unsigned long wifi_rcinterval = 1000;
unsigned long wifi_lastrc;
WiFiClient wclient;
char msg[50];

//variável do cliente MQTT
const char* mqtt_server = "192.168.1.70";
const char* mqtt_clientname = "tratador";   // nome do tópico de publicação
const char* mqtt_hostname = "controlador";  // nome do tópico de inscrição
const unsigned long mqtt_rcinterval = 5000;
unsigned long mqtt_lastrc;
PubSubClient mqtt_client(wclient);

// variávels para configurações dos pinos
const byte output_pins[] = {2, 4, 5, 14, 16};
const byte input_pins[] = {1, 12, 3};

// variavaies de controle de inputs
const unsigned int read_delay = 300;
InterruptMonitor motor(MOTOR_PIN, read_delay);
InterruptMonitor presenca(PRESENCA_PIN, read_delay);
InterruptMonitor motor2(MOTOR2_PIN, read_delay);
InterruptMonitor presenca2(PRESENCA2_PIN, read_delay);

// variaveis para controle de pulsos
int pulse_pin = -1;
unsigned long pulse_end;

// variáveis de controle da ativação do módulo
unsigned long ativo_temp;
bool ativo;

//------------------------------------//
//      DECLARAÇÃO DE FUNÇÕES
//------------------------------------//
// funçoes a serem chamadas pelos interrupts
void interrupt_presenca() {
  presenca.raiseFlag();
}
void interrupt_motor() {
  motor.raiseFlag();
}
void interrupt_presenca2() {
  presenca2.raiseFlag();
}
void interrupt_motor2() {
  motor2.raiseFlag();
}

// callback que lida com as mensagem recebidas
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

    // Copy the payload to the new char buffer
    char* msg = (char*)malloc(length + 1);
    memcpy(msg, (char*)payload, length);
    msg[length] = '\0';

    // prepara a msg de resposta
    char resp[30];
    strncpy(resp, msg, sizeof(resp));
    resp[sizeof(resp) -1] = '\0';

    Serial.println(msg);
    char* func = strtok(msg, ":");
    char* command = strtok(0, ":");

    if(strcmp(msg, "on")) {
      ativar_modulo(true);
    }
    else if (strcmp(msg, "off")) {
      ativar_modulo(false);
    }
    else if (strcmp(msg, "reset") == 0) {
      reset_pins();
    }
    else if (strcmp(func, "cycle") == 0) {
      if(strstr(command, "freq")) {
        strncpy(command, "0000", 4);
        int freq = atoi(command);
        if (initCycle(freq) == 1) sprintf(resp, "invalid request");
      }
    }
    else if (strstr(func, "pin")) {
      strncpy(func, "000", 3);
      int pin = atoi(func);
      int pin_type = pinType(pin);

      if (strcmp(command, "1") == 0 && pin_type == 1) {
        digitalWrite(pin, HIGH);
      }
      else if (strcmp(command, "0") == 0 && pin_type == 1) {
        digitalWrite(pin, LOW);
      }
      else if (strcmp(command, "state") == 0) {
        byte state = digitalRead(pin);
        sprintf(resp, "pin%d:%d", pin, state);
      }
      else if (strstr(command, "pulse") && pin_type == 1) {
        strncpy(command, "00000", 5);
        int pulse = atoi(command);
        if (setPulse(pin, pulse) == 1) sprintf(resp, "invalid request");
      }
      else if(strstr(command, "freq") && pin_type == 1) {
        strncpy(command, "0000", 4);
        int freq = atoi(command);
        if (setFrequency(pin, freq) == 1) sprintf(resp, "invalid request");
      }
      else sprintf(resp, "invalid request");
    }
    else sprintf(resp, "invalid request");
    Serial.print("resp: "); Serial.println(resp);

    mqtt_client.publish(mqtt_clientname, resp);

    free(msg);
}

int pinType(int pin) {
  int type = 0;
  for (byte i = 0; i < sizeof(output_pins); i++)
    if (pin == output_pins[i]) type = 1;

  for (byte i = 0; i < sizeof(input_pins); i++)
    if (pin == input_pins[i]) type = 2;

  return type;
}

void ativar_modulo(bool state) {
  ativo = state;
  ativo_temp = millis();
  digitalWrite(ATIVO_PIN, state);
}

int setFrequency(int pin, int freq) {
  if ( freq > FREQ_MAX || (freq < FREQ_MIN && freq != 0)) return 1;

  analogWriteFreq(freq);
  analogWrite(pin, 512);
  return 0;
}

int setPulse(int pin, unsigned long pulse) {
  if (pulse > PULSE_MAX || pulse < PULSE_MIN) return 1;

  pulse_pin = pin;
  pulse_end = millis() + pulse;
  digitalWrite(pulse_pin, HIGH);
  return 0;
}

int initCycle(int freq) {
  if (setFrequency(FREQ_PIN, freq) == 1 || !ativo) return 1; //configura velocidade do motor

  delay(100); //espera o inversor ler corretamente a velocidade
  setPulse(PULSE_PIN, 1000); //gera pulso que inicia o ciclo de tratamento

  return 0;
}

void reset_pins() {
  for (byte i = 0; i < sizeof(output_pins); i++) {
    if (output_pins[i] == FREQ_PIN) setFrequency(FREQ_PIN, FREQ_STD);
    else {
      analogWrite(output_pins[i], 0);
      digitalWrite(output_pins[i], LOW);
    }
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

bool reconnectMQTT() {
  Serial.println("Connecting to the MQTT broker...");
  if (mqtt_client.connect(mqtt_clientname)) {
    // Once connected, publish an announcement...
    mqtt_client.publish(mqtt_clientname, "hello world", true);
    // ... and resubscribe
    mqtt_client.subscribe(mqtt_hostname);
  }
  return mqtt_client.connected();
}

//---------------------------------------------//
//                  SETUP
//---------------------------------------------//
void setup()  {
  Serial.begin(115200);
  Serial.swap();        // muda pinos do serial para 13(RX) e 15(TX)
  Serial.setDebugOutput(true);
  Serial.println();

  //configuração dos pinos
  for (byte i = 0; i < sizeof(output_pins); i++) {
    pinMode(output_pins[i], OUTPUT);
  }
  for (byte i = 0; i < sizeof(input_pins); i++) {
    pinMode(input_pins[i], INPUT_PULLUP);
  }

  // configura freq padrao
  setFrequency(FREQ_PIN, FREQ_STD);

  // configura interruptor dos pinos de entrada
  attachInterrupt(digitalPinToInterrupt(presenca.pin), interrupt_presenca, RISING);
  attachInterrupt(digitalPinToInterrupt(motor.pin), interrupt_motor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(presenca2.pin), interrupt_presenca2, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2.pin), interrupt_motor2, CHANGE);

  // configuracao do wifi
  WiFi.mode(WIFI_STA);

  // configuração do MQTT
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(mqtt_callback);

  // ativa módulo
  ativar_modulo(true);

  //---------------------------------------------//
  //            CONFIGURAÇÃO DO OTA
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

//---------------------------------------------//
//                  LOOP
//---------------------------------------------//
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

  // pinos de entrada
  // <pin>.update()= -1: sem mudanças, 0: mudou para LOW, 1: mudou para HIGH

  // presença= LOW: sem presenca, HIGH: presenca detectada
  int presenca_state = presenca.update();
  if (presenca_state == HIGH) {
    //envia mensagem
    mqtt_client.publish(mqtt_clientname, "presenca");
  }

  // estado do ciclo de tratamento= LOW: em execução, HIGH: ocioso
  int motor_state = motor.update();
  if (motor_state != -1) {
    //envia mensagem
    char msg[] = "motor:0";
    if (motor_state == LOW) msg[6] = '1';
    mqtt_client.publish(mqtt_clientname, msg);
  }


  int presenca2_state = presenca2.update();
  if (presenca2_state == HIGH) {
    //envia mensagem
    mqtt_client.publish(mqtt_clientname, "presenca2");
  }
  int motor2_state = motor.update();
  if (motor2_state != -1) {
    //envia mensagem
    char msg[] = "motor2:0";
    if (motor2_state == LOW) msg[6] = '1';
    mqtt_client.publish(mqtt_clientname, msg);
  }

  // checa a presença do controlador
  // é esperando que a cada 300s o controlador envie uma msg='on' para avisar que está tudo bem
  // se essa msg não for recebida por dois ciclos seguidos o módulos será desativado e o temp. externo
  // tomará seu lugar

  unsigned long now = millis();
  if (ativo && now - ativo_temp > 600000L + 10000L) {
    ativar_modulo(false);
  }
}
