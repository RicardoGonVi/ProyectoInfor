// IMPORTANDO LIBRERIAS Y COSAS NECESARIAS -------------
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266httpUpdate.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>


#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif
// ------------------------------------------------------
#define HTTP_OTA_ADDRESS      F("192.168.1.138")       // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update") // Path to update firmware
#define HTTP_OTA_PORT         1880                     // Port of update server
                                                       // Name of firmware
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"

// ------------------------------------------------------
ADC_MODE(ADC_VCC);
DHTesp dht;
WiFiClient espClient;
PubSubClient client(espClient);
// ------------------------------------------------------


// CONFIGURACION DE LA RED ------------------------------
const char* ssid = "MIWIFI_dgXC";                       // MIWIFI_2G_jvp9 // Richy iPhone       // VIRGIN-telco_C4F4  //MIWIFI_dgXC       // maria
const char* password = "u6EN4QXC";                      // qhrKrgbv       // noruju132823       // hRRXE9Y5ybyeRs     //u6EN4QXC          // mariaza3
const char* mqtt_server = "192.168.1.138";              // 192.168.1.133  // 172.20.10.5        // 192.168.1.134      //192.168.1.138     //192.168.43.164
const char* mqtt_user = "infind";
const char* mqtt_pass = "zancudo";
// ------------------------------------------------------


// Topics -----------------------------------------------
char topic_datos[256];
char topic_config[256];
char topic_led_cmd[256];
char topic_led_status[256];
char topic_switch_cmd[256];
char topic_switch_status[256];
char topic_fota[256];


// DEFINIENDO PARAMETROS INICIALES ---------------------
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
char cadena[512];                       // char que contiene la cadena del monitor serial
char mensaje_valoresLED[128];           // mensaje de los valores del LED
char mensaje_valoresSWITCH[128];        // mensaje de los valores del SWITCH
char mensaje_LWTon[128];                // mensaje cuando se esta activo 
char mensaje_LWToff[128];               // mensaje del lastwill
char ChipID_char[16];                   // char con el ID del ESPX


int boton = 0;                          // GPIO0 = boton uso dos tipos de estructuras una de interrupción y la otra es un process()
int estado_int = HIGH;
volatile int lectura = HIGH;            // -------
volatile unsigned long ahora=0;         // -------
volatile unsigned long ultima_int = 0;

struct registro_datos {                 // estructura de datos para el sensor
  float temperatura;
  float humedad;
  unsigned long tiempo;
  };
struct registro_datos misdatos;         // inicializando una variable de la estructura

unsigned long lastMsg = 0;                        // valor que contiene el tiempo del ult msj
unsigned long lastMsg_fota = 0;                   // valor que contiene el tiempo del ult msj

int value = 0;                                    // contador 
int cont = 0;                                     // contador del uptime

volatile int valorLED2 = 0;                       // int con el valor del GPIO2 actual (0-255) 
volatile int valorLED16 = 0;                      // int con el valor del GPIO2 actual (0 o 1)  
volatile int tdatos = 60 * 5 * 1000;              // tiempo inicial de envio de datos = 5min (valor en ms)
volatile int t_revisaFOTA = 0;                    // tiempo para revisar si se debe actualizar el OTA
volatile int valorLED_ajustado;
volatile int velocidad_led = 30;                  // tiempo de actualizacion al cambiar el LED (en ms)
volatile int ChipID = ESP.getChipId();            // obtiene el ID de CHIP
volatile int LEDGuardado = 255;
volatile int clicks =0;

volatile bool estadoInversorSwitch2 = false;      // estado del switch del GPIO16
volatile bool estadoInversorSwitch16 = false;     // estado del switch del GPIO16

//String clientId = "ESP8266Client-";               // inicio del ID del ESP
String origen_led;
String origen_switch;
String id_mqtt = "esperandoID";

volatile long timePress = 0;
volatile long timePressLimit = 0;
volatile bool Prolong = false;                    //inicio variable booleana que indica si es pulsación prolongada
volatile bool fota_flag = false;

void progreso_OTA(int,int);
void final_OTA();
void inicio_OTA();
void error_OTA(int);
// ------------------------------------------------------


// FOTA -------------------------------------------------
void intenta_OTA(){ 
  Serial.println( "--------------------------" );  
  Serial.println( "Comprobando actualización:" );
  Serial.print(HTTP_OTA_ADDRESS);Serial.print(":");Serial.print(HTTP_OTA_PORT);Serial.println(HTTP_OTA_PATH);
  Serial.println( "--------------------------" );  
  ESPhttpUpdate.onError(error_OTA);
  ESPhttpUpdate.onProgress(progreso_OTA);
  ESPhttpUpdate.onEnd(final_OTA);
  WiFiClient wClient;
  switch(ESPhttpUpdate.update(wClient, HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(" HTTP update failed: Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F(" El dispositivo ya está actualizado"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F(" OK"));
      break;
  }
}

void final_OTA(){ Serial.println("Fin OTA. Reiniciando..."); }

void inicio_OTA(){ Serial.println("Nuevo Firmware encontrado. Actualizando..."); }

void error_OTA(int e){
  char cadena[64];
  snprintf(cadena,64,"ERROR: %d",e);
  Serial.println(cadena);
}

void progreso_OTA(int x, int todo)
{
  char cadena[256];
  int progress=(int)((x*100)/todo);
  if(progress%10==0){
    snprintf(cadena,256,"Progreso: %d%% - %dK de %dK",progress,x/1024,todo/1024);
    Serial.println(cadena);
  }
}
//-----------------------------------------------------


// INTERRUPCION DEL BOTON -----------------------------
ICACHE_RAM_ATTR void RTI() {                                                          // función de lectura de pulsación (Interrupción)
  ahora = millis();                                                                   // calculamos cuando empezó la interrupción
  lectura = digitalRead(boton);                                                       // leemos el botón
  
  if(lectura==estado_int || ahora-ultima_int<50)return;                               // evitamos rebotes si se detecta pulsación en menos de 50 ms de la última interrupción se ignora
  if(lectura == LOW){                                                                 // si se pulsa el botón ponemos automáticamente el led al valor último establecido 
      Prolong = true;                                                                 // la pongo a true para que hasta que no se suelte la pulsación no pueda decidir si es pulsacion larga o corta
      Serial.println("Button Pressed once");                                          // si se pulsa
      estado_int = LOW;
      if (clicks == 0){                                                               // cuando click=0 
        timePress = millis();                                                         // cogemos el tiempo en el que se pulsó        
        timePressLimit = timePress + 1000;                                            // establecemos limite
        Serial.println(LEDGuardado);
      }    
      clicks = clicks +1;                                                             // si hay un click se añade un valor a click 
      Serial.println(clicks);
  
     if (clicks == 2 && millis() < timePressLimit){                                   // cuando se pulsa 2 veces y no se superó el tiempo límite
        Serial.println("Doble click");                                                // millis saca el tiempo actual y como timePresslimit se hizo de manera relativa al tiempo actual detecta bien el tiempo limite
        timePress = 0;                                                                // se resetean los valores
        timePressLimit = 0;
        clicks = 0;
        valorLED2 = 100;
        analogWrite(2,100 - valorLED2);
      }
      origen_led = "pulsador";                                                   
  }   
  if(lectura == HIGH)                                                                   // cuando termina la pulsación vemos si es una pulsación prolongada
  {                                                                                                  
     estado_int = HIGH;
     Prolong = false;                                                                   // aquí termina la pulsación y ya si es prolongada lo pone a true y si no lo es false y se ilumina el led
     if (ahora-ultima_int > 500){                                                       // si hemos estado pulsando el botón más de 0.5s detecta pulsación prolongada
        Serial.println("pulsacion prolongada");                                         // AQUÍ MARÍA METE EL FOTA
        Prolong = true;                                                                 // como la pulsación es prolongada es true  
        fota_flag = true;                                                               // activa el flag para actualizar el FOTA                              
     }
  }
  ultima_int = ahora;                                                                   // actualizamos cuando ha sido la última interrupción
}
// ---------------------------------------------------


// MENSAJE DEL ESPX/config ----------------------------
void serializa_sprintf(struct registro_datos datos, char *cadena, int tamaño, 
                  const char *ip, const char *SSid, int adc, char* ChipID, short int rssi, int LED, int switchLED)  // funcion que serializa lo mostrado en el monitor_serie
{
  char comillas='"';
  snprintf(cadena, tamaño, 
            "{\"CHIPID\":%c%s%c, \"Uptime\":%d,\"Vcc\":%d, \"DHT11\":{\"temp\":%g,\"hum\":%g}, \"LED\":%i, \"SWITCH\":%i,\"WiFi\":{\"SSid\":%c%s%c, \"IP\":%c%s%c, \"RSSI\":%hi}}",
           comillas, ChipID, comillas, datos.tiempo, adc, datos.temperatura, datos.humedad, LED, switchLED, comillas, SSid, comillas, comillas, ip, comillas, comillas, rssi, comillas);
}
// ---------------------------------------------------


// INCIO DEL WIFI ------------------------------------
void setup_wifi() {
  Serial.printf("\nConnecting to %s:\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
}
// -------------------------------------------------


// FUNCIONES NECESARIAS EN EL CALLBACK -------------
void estadoGPIOX(int GPIO, int boolNumero){
  if (boolNumero == 1) { digitalWrite(GPIO, LOW); }                             // si el valor es 1 se apaga (logica inversa)
  else{ digitalWrite(GPIO, HIGH); }                                             // si el valor es 0 se enciende (logica inversa)
}

void velocidad(int valori, int valorf){
  int valormed = valori;                                                        // inicializa el valor que cambia en vi
  unsigned long tanterior = millis();
  
  if (valorf - valori >= 0){                                                    // si el vf es mayor a vi (aumento)
    int incremento = ceil((valorf-valori)/100.0);                               // calcula el 1% del rango a cambiar (redondea hacia arriba)       
    while (valormed < valorf){                                                  // aumenta desde vi a vf
      unsigned long tactual = millis();
      if ((tactual - tanterior) >= velocidad_led){
        valormed+= incremento;                                                  // actualiza vmedio
        if (valorf < valormed) { valormed = valorf; }                           // asegura que el vmedio no supere el vf deseado
        analogWrite(2,(int)valormed);                                           // muestra el valor siguiente en el GPIO2
        tanterior = tactual;
      }
    }
  }
  else{                                                                         // si el vf es menor a vi (disminuyo)
    int incremento = ceil((valori-valorf)/100.0);                               // calcula el 1% del rango a cambiar (redondea hacia arriba)  
    while (valormed > valorf){                                                  // disminuyo desde vf a vi
      unsigned long tactual = millis();
      if ((tactual - tanterior) >= velocidad_led){
        valormed = valormed - incremento;                                       // actualiza vmedio
        if (valorf > valormed) { valormed = valorf; }                           // asegura que el vmedio no este por debajo que el vf deseado
        analogWrite(2,(int)valormed);                                           // muestra el valor siguiente en el GPIO2
        tanterior = tactual;
      }
    }
  }
  LEDGuardado=valormed;                                                        // valor guardado para el boton 0
}
// ---------------------------------------------------



// ---------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {              // funcion que realiza acciones al recibir info el ESP
  String string_topic = String(topic);                                        // convierte el topic en string (era char)
  
  char *mensaje = (char *)malloc(length+1);                                   // reservo memoria para copia del mensaje
  strncpy(mensaje, (char*)payload, length);                                   // copio el mensaje en cadena de caracteres
  mensaje[length]='\0';                                                       // caracter cero marca el final de la cadena
  Serial.printf("Mensaje recibido [%s] %s\n", topic, mensaje);

  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, mensaje);                 // deserealiza el json
  if (error) { return; }
  
  // TOPIC DEL FOTA  --------------------------------------------
  if (string_topic == topic_fota) { fota_flag = true; }                        
  
  // TOPIC DEL GPIO2: LED PWM -----------------------------------
  if (string_topic == topic_led_cmd){                                         // si el topic es el indicado entra 
    int level = doc["level"];
    id_mqtt = String(doc["id"]);
    origen_led = "mqtt";
    
    int valor_inicial = valorLED_ajustado;
    if (estadoInversorSwitch2 == false) { valorLED2 = level; }                // condicion para la salida normal
    else { valorLED2 = 100 - level; }                                         // condicion para la salida invertida
    valorLED_ajustado = floor(valorLED2*2.55*-1 + 255);                       // cambia de rango 0-100 a 0-255 (con logica invertida)
    velocidad(valor_inicial,valorLED_ajustado);
  }

  // TOPICS DEL GPIO16: SWITCH -----------------------------------
  if (string_topic == topic_switch_cmd){                                      // si el topic es el indicado entra
    int level = doc["level"];
    id_mqtt = String(doc["id"]);

    if (estadoInversorSwitch16 == false) { valorLED16 = level; }              // cuando el inversor es false se deja la lógica inversa    
    else{ valorLED16 = 1 - level; }                                           // cuando es true se va a logica normal
    estadoGPIOX(16, valorLED16);                                              // llamado a funncion que enciende y apaga el GPIO
  }
  
  // TOPIC DE LA CONFIG DEL ESP -----------------------------------
  if (string_topic == topic_config){                                          // si el topic es el indicado entra 
    int envia = doc["envia"];
    int actualiza = doc["actualiza"];
    int velocidad = doc["velocidad"];
    int LED = doc["LED"];
    int SWITCH = doc["SWITCH"];

    tdatos = envia * 1000;                                                    // modifica el tiempo de conf (ms)
    t_revisaFOTA = actualiza * 1000 * 60;                                     // modifica el tiempo de actualizar fota (ms)
    velocidad_led = velocidad;                                                // modifica el tiempo de conf (ms)

    // LED 
    if (LED == 1){
      if (estadoInversorSwitch2 == false){
        valorLED2 = 100 - valorLED2;                                           // cambia la salida
        valorLED_ajustado = floor(valorLED2*2.55*-1 + 255);                    // cambia de rango 0-100 a 0-255 (con logica invertida)
        analogWrite(2,(int) valorLED_ajustado);  
      }
      estadoInversorSwitch2 = true;                                            // actualiza el estado de inversion del GPIO2
    }
    else{
      if (estadoInversorSwitch2 == true){
        valorLED2 = 100 - valorLED2;                                           // cambia la salida
        valorLED_ajustado = floor(valorLED2*2.55*-1 + 255);                    // cambia de rango 0-100 a 0-255 (con logica invertida)
        analogWrite(2,(int) valorLED_ajustado);  
      }
      estadoInversorSwitch2 = false;
    }

    // SWITCH
    if (SWITCH == 1){
      if (estadoInversorSwitch16 == false){
        valorLED16 = 1 - valorLED16;                                              // cambia el estado del GPIO
        estadoGPIOX(16, valorLED16);                                              // llamado a funncion que enciende y apaga el GPIO
      }
      estadoInversorSwitch16 = true;
    }                                                                             // modifica el estado del inversor
    else{
      if (estadoInversorSwitch16 == true){
        valorLED16 = 1 - valorLED16;                                              // cambia el estado del GPIO
        estadoGPIOX(16, valorLED16);                                              // llamado a funncion que enciende y apaga el GPIO
      }
      estadoInversorSwitch16 = false;
    }
  }
  free(mensaje);
}
// -------------------------------------------------


// -------- CONEXION MQTT --------------------------
void reconnect() {
  while (!client.connected()) {                                               // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
    //clientId += String(random(0xffff), HEX);                                // ID unica de c/u de los clientes
    
    
    
    
    if (client.connect((String(ChipID)).c_str())) {     // Attempt to connect
    //if (client.connect((String(ChipID)).c_str(), mqtt_user, mqtt_pass)) {     // Attempt to connect
      
      
      
      
      Serial.printf(" conectado a broker: %s\n",mqtt_server);
      client.publish("G14conexion", "Conectado");                             // Once connected, publish an announcement...
      // ... and resubscribe
      client.subscribe(topic_led_cmd);                                        // suscripcion LED
      client.subscribe(topic_switch_cmd);                                     // suscripcion SWITCH
      client.subscribe(topic_config);                                         // suscripcion configuracion del dispositivo
      client.subscribe(topic_fota);                                           // suscripcion al FOTA  
    } else {
      Serial.printf("failed, rc=%d  try again in 5s\n", client.state());
      delay(5000);                                                            // Wait 5 seconds before retrying
    }
  }
}


// --------------------SETUP------------------------
void setup() {
  // inicializacion de los GPIO y botones
  Serial.begin(115200);
  Serial.println("Empieza setup...");
  
  pinMode(2, OUTPUT);                                                                                 // Initialize the BUILTIN_LED pin as an output
  pinMode(16, OUTPUT);                                                                                // Initialize the BUILTIN_LED pin as an output
  pinMode(boton, INPUT_PULLUP);                                                                       // inicializamos el boton GPI0
  analogWrite(2,255);                                                                                 // estado inicial off
  digitalWrite(16,HIGH);                                                                              // estado inicial off

  // inicializacion de la interrupcion
  attachInterrupt(digitalPinToInterrupt(boton), RTI, CHANGE);                                         // añadimos interrupción en boton 0

  // inicializaciones generales
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); 
  reconnect();
  
  // inicializacion del OTA
  intenta_OTA();

  // inicializacion del sensor de temperatura
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard = ARDUINO_BOARD;
  Serial.println(thisBoard); 
  dht.setup(5, DHTesp::DHT11);                                                                        // Connect DHT11 sensor to GPIO 5-D1

  // inicializacion de textos
  snprintf(mensaje_LWTon, 128, "{\"online\": %s}", "true");                                           // estado last will cuando la placa se enciende
  snprintf(mensaje_LWToff, 128, "{\"online\": %s}", "false");                                         // estado last will cuando la placa se apaga
  snprintf(ChipID_char, 16, "ESP%s",(String(ChipID)).c_str());
  snprintf(topic_datos, 128, "II14/ESP%s/datos", (String(ChipID)).c_str());
  snprintf(topic_config, 128, "II14/ESP%s/config", (String(ChipID)).c_str());
  snprintf(topic_led_cmd, 128, "II14/ESP%s/led/cmd", (String(ChipID)).c_str());
  snprintf(topic_led_status, 128, "II14/ESP%s/led/status", (String(ChipID)).c_str());
  snprintf(topic_switch_cmd, 128, "II14/ESP%s/switch/cmd", (String(ChipID)).c_str());
  snprintf(topic_switch_status, 128, "II14/ESP%s/switch/status", (String(ChipID)).c_str());
  snprintf(topic_fota, 128, "II14/ESP%s/FOTA", (String(ChipID)).c_str());

  Serial.print("ChipID: ");
  Serial.println(ChipID); 
  Serial.printf("Termina setup en %lu ms\n\n", millis());
}
// -------------------------------------------------


// -------------------- LOOP -----------------------
void loop() {
  
  if (!client.connected()) { reconnect();}                                                            // en caso de desconexion reconecta al ESP

  // last will
  if (client.connect((String(ChipID)).c_str(), "", "", "infind/GRUPO14/conexion", 0, true, mensaje_LWToff))   
  {client.publish("infind/GRUPO14/conexion", mensaje_LWTon, true);}
  
  client.loop();
  delay(dht.getMinimumSamplingPeriod());

  // PULSOS 
  if (timePressLimit != 0 && millis()> timePressLimit){
    timePress = 0;                                                                                
    timePressLimit = 0;
    clicks = 0;
    Serial.println("Tiempo superado una pulsación");
    if (Prolong==false){                                                        // si la pulsación no es prolongada 
      analogWrite(2,(int)LEDGuardado);
      valorLED2 = 100 - (LEDGuardado/2.55);
    } 
  }

  // FOTA
  if (fota_flag == true){                                                           // revisa si se el flag de actualizar FOTA esta on
    intenta_OTA();                                                                  // actualiza
    fota_flag = false;                                                              // baja el flag
  }
  unsigned long now_fota = millis();
  if (now_fota - lastMsg_fota > t_revisaFOTA ){                                      // si pasó el tiempo requerido actualiza
    lastMsg_fota = now_fota;                                                        // pone en el t anterior el recien 
    if (t_revisaFOTA != 0){ intenta_OTA(); }                                        // al ser cero el tiempo no se debe buscar actualización
  }
    
  // LOOP DE ACTUALIZACION (BASADO EN UN TIEMPO X)
  unsigned long now = millis();
  if (now - lastMsg > tdatos) {                                                     // si el t_ahora - t_ultMSG > t configuracion de los datos entra y actualiza datos                                    
    lastMsg = now;
    cont = cont + tdatos;                                                           // genero el uptime
    ++value;
    
    // muestra en el monitor serie
    snprintf (msg, MSG_BUFFER_SIZE, "Publico valor nº #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    Serial.println("JSON generados: ");
    
    // actualizacion del LED 
    if (origen_led == "mqtt"){
      snprintf(mensaje_valoresLED, 128, "{\"CHIPID\":%c%s%c,\"LED\":%i,\"origen\":%c%s%c,\"id\":%c%s%c}",
                       '"',ChipID_char,'"',valorLED2,'"',origen_led.c_str(),'"','"',id_mqtt.c_str(),'"');
    }
    else{
      snprintf(mensaje_valoresLED, 128, "{\"CHIPID\":%c%s%c,\"LED\":%i,\"origen\":%c%s%c}",
                       '"',ChipID_char,'"',valorLED2,'"',origen_led.c_str(),'"');
    }
    Serial.println(mensaje_valoresLED);
    client.publish(topic_led_status, mensaje_valoresLED);                                                              // publica en el topic


    // actualizacion del SWITCH 
    snprintf(mensaje_valoresSWITCH, 128, "{\"CHIPID\":%c%s%c,\"SWITCH\":%i,\"origen\":%c%s%c,\"id\":%c%s%c}",
                       '"',ChipID_char,'"',valorLED16,'"',"mqtt",'"','"',id_mqtt.c_str(),'"');
    Serial.println(mensaje_valoresSWITCH);
    client.publish(topic_switch_status, mensaje_valoresSWITCH); 

    
    // lectura de valores del sensor y ESP
    float humidity = dht.getHumidity();
    float temperature = dht.getTemperature();
    misdatos.temperatura = temperature;
    misdatos.humedad = humidity;
    misdatos.tiempo = cont;
    int valor_vcc = ESP.getVcc();
    short int rssi = WiFi.RSSI();                                                                                       // obtiene el rssi del wifi
    
    serializa_sprintf(misdatos, cadena, 512, mqtt_server, ssid, valor_vcc, ChipID_char, rssi, valorLED2, valorLED16);   // generamos cadena en monitor serie
    Serial.println(cadena);
    client.publish(topic_datos,cadena);                                                                                 // publicamos la cadena 
  }
}
