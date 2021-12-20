// IMPORTANDO LIBRERIAS Y COSAS NECESARIAS -------------
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <ArduinoJson.h>
#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif
// ------------------------------------------------------


// ------------------------------------------------------
ADC_MODE(ADC_VCC);
DHTesp dht;
WiFiClient espClient;
PubSubClient client(espClient);
// ------------------------------------------------------



// CONFIGURACION DE LA RED ------------------------------
const char* ssid = "MIWIFI_dgXC";                 // MIWIFI_2G_jvp9 // Richy iPhone // VIRGIN-telco_C4F4 //MIWIFI_dgXC
const char* password = "u6EN4QXC";                // qhrKrgbv // noruju132823      //  hRRXE9Y5ybyeRs    //u6EN4QXC
const char* mqtt_server = "192.168.1.138";        // 192.168.1.133 // 172.20.10.5 //   192.168.1.134     //192.168.1.138
// ------------------------------------------------------


// DEFINIENDO PARAMETROS INICIALES ---------------------
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
char cadena[512];                       // char que contiene la cadena del monitor seria
char mensaje_valoresLEDs[128];          // mensaje de los valores del GPIO
char mensaje_LWTon[128];                // mensaje cuando se esta activo 
char mensaje_LWToff[128];               // mensaje del lastwill

int boton=0;                             // GPIO0 =boton uso dos tipos de estructuras una de interrupción y la otra es un process()
int estado_int=HIGH;
volatile int lectura=HIGH;               // -------
volatile unsigned long ahora=0;          // -------
volatile unsigned long ultima_int = 0;

struct registro_datos {                 // estructura de datos para el sensor
  float temperatura;
  float humedad;
  unsigned long tiempo;
  };
struct registro_datos misdatos;         // inicializando una variable de la estructura


unsigned long lastMsg = 0;                        // valor que contiene el tiempo del ult msj
int value = 0;                                    // contador 
int cont = 0;                                     // contador del uptime
volatile int valorLED2 = 0;                       // int con el valor del GPIO2 actual (0-255) 
volatile int valorLED16 = 0;                      // int con el valor del GPIO2 actual (0 o 1)  
volatile int tdatos = 60 * 5 * 1000;              // tiempo inicial de envio de datos = 5min (valor en ms)
volatile int valorLED_ajustado;
volatile int velocidad_led = 30;                  // tiempo de actualizacion al cambiar el LED (en ms)
volatile bool estadoInversorSwitch2 = false;      // estado del switch del GPIO16
volatile bool estadoInversorSwitch16 = false;     // estado del switch del GPIO16
String clientId = "ESP8266Client-";               // inicio del ID del ESP

volatile int LEDGuardado=255;
volatile int clicks =0;
volatile long timePress=0;
volatile long timePressLimit=0;
volatile bool Prolong=false;                      //inicio variable booleana que indica si es pulsación prolongada
// ------------------------------------------------------


// INICIO DE FUNCIONES ----------------------------------

ICACHE_RAM_ATTR void RTI() {                                                        // función de lectura de pulsación (Interrupción)
  ahora= millis();                                                                  // calculamos cuando empezó la interrupción
  lectura=digitalRead(boton);                                                       // leemos el botón
  
  if(lectura==estado_int || ahora-ultima_int<50)return;                             // evitamos rebotes si se detecta pulsación en menos de 50 ms de la última interrupción se ignora
  if(lectura==LOW)                                                                  // si se pulsa el botón ponemos automáticamente el led al valor último establecido 
    { 
      Prolong=true;                                                                 // la pongo a true para que hasta que no se suelte la pulsación no pueda decidir si es pulsacion larga o corta
      Serial.println("Button Pressed once");                                        // si se pulsa
      estado_int=LOW;
      if (clicks==0){                                                               // cuando click=0 
        timePress = millis();                                                       // cogemos el tiempo en el que se pulsó        
        timePressLimit = timePress + 1000;                                          // establecemos limite
        analogWrite(2,(int)LEDGuardado);                                            // escribimos el valor almacenado
        Serial.println(LEDGuardado);
      }    
      clicks=clicks +1;                                                             // si hay un click se añade un valor a click 
      Serial.println(clicks);
  
     if (clicks == 2 && millis() < timePressLimit){                                 // cuando se pulsa 2 veces y no se superó el tiempo límite
        Serial.println("Doble click");                                              // millis saca el tiempo actual y como timePresslimit se hizo de manera relativa al tiempo actual detecta bien el tiempo limite
        timePress = 0;                                                              // se resetean los valores
        timePressLimit = 0;
        clicks = 0;
        
        analogWrite(2,0);
      }
                                                          
  }   
  if(lectura==HIGH)                                                                 // cuando termina la pulsación vemos si es una pulsación prolongada
  {                                                                                                  
     estado_int=HIGH;
     Prolong=false;                                                                 // aquí termina la pulsación y ya si es prolongada lo pone a true y si no lo es false y se ilumina el led
     if (ahora-ultima_int>500){                                                     // si hemos estado pulsando el botón más de 0.5s detecta pulsación prolongada
        Serial.println("pulsacion prolongada");                                     // AQUÍ MARÍA METE EL FOTA
        Prolong=true;                                                               // como la pulsación es prolongada es true
     }  
  }
    
  ultima_int = ahora;                                                                   // actualizamos cuando ha sido la última interrupción
                                 
}
 
void serializa_sprintf(struct registro_datos datos, char *cadena, int tamaño, 
                  const char *ip, const char *SSid, int adc)                    // funcion que serializa lo mostrado en el monitor serie
{
  char comillas='"';
  snprintf(cadena, tamaño, "{\"Uptime\":%d,\"Vcc\":%d, \"DHT11\":{\"temp\":%g,\"hum\":%g}, \"LED\":%i,\"WiFi\":{\"SSid\":%c%s%c, \"IP\":%c%s%c}}",
                        datos.tiempo, adc, datos.temperatura, datos.humedad, valorLED2, comillas, SSid, 
                        comillas, comillas, ip, comillas );
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    Serial.println(WiFi.status());
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// FUNCIONES NECESARIAS EN EL CALLBACK -------------
void estadoGPIOX(int GPIO, int boolNumero){
  if (boolNumero == 1){digitalWrite(GPIO, LOW);}                              // si el valor es 1 se apaga (logica inversa)
  else{digitalWrite(GPIO, HIGH);}                                             // si el valor es 0 se enciende (logica inversa)
  }

void velocidad( int valori, int valorf){
  int valormed=valori;                                                        // inicializa el valor que cambia en vi
  Serial.println(valormed);
  Serial.println(valorf);
  unsigned long tanterior = millis();
  
  if (valorf-valori>=0){                                                        // si el vf es mayor a vi (aumento)
    int incremento = ceil((valorf-valori)/100.0);                               // calcula el 1% del rango a cambiar (redondea hacia arriba)       
    Serial.println(incremento);
    while (valormed < valorf){                                                  // aumenta desde vi a vf
      unsigned long tactual = millis();
      if ((tactual - tanterior) >= velocidad_led){
        valormed+= incremento;                                                  // actualiza vmedio
        if (valorf < valormed){valormed = valorf;}                              // asegura que el vmedio no supere el vf deseado
        Serial.println(valormed);
        analogWrite(2,(int)valormed);                                           // muestra el valor siguiente en el GPIO2
        tanterior = tactual;
      }
    }
  }
  else{                                                                         // si el vf es menor a vi (disminuyo)
    int incremento = ceil((valori-valorf)/100.0);                               // calcula el 1% del rango a cambiar (redondea hacia arriba)  
    Serial.println(incremento);
    while (valormed > valorf){                                                  // disminuyo desde vf a vi
      unsigned long tactual = millis();
      if ((tactual - tanterior) >= velocidad_led){
        valormed = valormed - incremento;                                       // actualiza vmedio
        if (valorf > valormed){valormed = valorf;}                              // asegura que el vmedio no este por debajo que el vf deseado
        Serial.println(valormed);
        analogWrite(2,(int)valormed);                                           // muestra el valor siguiente en el GPIO2
        tanterior = tactual;
      }
    }
  }
  LEDGuardado=valormed;                                                        // valor guardado para el boton 0
}
// -------------------------------------------------
void callback(char* topic, byte* payload, unsigned int length) {              // funcion que realiza acciones al recibir info el ESP
  String string_topic = String(topic);                                        // convierte el topic en string (era char)
  
  Serial.print("Message arrived [");                                          // muestra en el monitor serie el topic del msj proveniente
  Serial.print(topic);                                                
  Serial.print("] ");

  int construyendoNum = 0;                                                    // los mensajes son numeros que deben construirse (se inicia de cero siempre)
  for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      construyendoNum =  10*construyendoNum + (int(payload[i]) - '0');        // construyendo el num de izq a der
    }

  // TOPIC DEL GPIO2: LED PWM -------
  if (string_topic == "infind/GRUPO14/led/cmd"){                              // si el topic es el indicado entra 
    int valor_inicial = valorLED_ajustado;
    if (estadoInversorSwitch2 == false){valorLED2 = construyendoNum;}         // condicion para la salida normal
    else{valorLED2 = 100 - construyendoNum;}                                  // condicion para la salida invertida
    valorLED_ajustado = floor(valorLED2*2.55*-1 + 255);                       // cambia de rango 0-100 a 0-255 (con logica invertida)
    velocidad(valor_inicial,valorLED_ajustado);
    //analogWrite(2,(int) valorLED_ajustado);                                 // escribe sobre el GPIO2 el valor
  }
  
  if (string_topic == "infind/GRUPO14/led/inversion"){                        // si el topic es el indicado entra 
    if (construyendoNum == 1){estadoInversorSwitch2 = true;}                  // actualiza el estado de inversion del GPIO2
    else{estadoInversorSwitch2 = false;}   
    valorLED2 = 100 - valorLED2;                                              // cambia la salida
    valorLED_ajustado = floor(valorLED2*2.55*-1 + 255);                       // cambia de rango 0-100 a 0-255 (con logica invertida)
    analogWrite(2,(int) valorLED_ajustado);                                   
  }// ----------------------------

  // TOPICS DEL GPIO16: SWITCH -------
  if (string_topic == "infind/GRUPO14/led/GPIO16"){                           // si el topic es el indicado entra
    if (estadoInversorSwitch16 == false){valorLED16 = construyendoNum;}       // cuando el inversor es false se deja la lógica inversa    
    else{valorLED16 = 1 - construyendoNum;}                                   // cuando es true se va a logica normal
    estadoGPIOX(16, valorLED16);                                              // llamado a funncion que enciende y apaga el GPIO
  }

  if (string_topic == "infind/GRUPO14/led/GPIO16/inversion"){                 // si el topic es el indicado entra 
    if (construyendoNum == 1){estadoInversorSwitch16 = true;}                 // modifica el estado del inversor
    else{estadoInversorSwitch16 = false;}
    valorLED16 = 1 - valorLED16;                                              // cambia el estado del GPIO
    estadoGPIOX(16, valorLED16);                                              // llamado a funncion que enciende y apaga el GPIO
  }// ----------------------------
  
  // TOPIC DE LA CONFIG DEL ESP
  if (string_topic == "infind/II14/ESPX/config"){                             // si el topic es el indicado entra 
    tdatos = construyendoNum * 1000;                                          // modifica el tiempo de conf (ms)
  }

  if (string_topic == "infind/II14/ESPX/config/velocidad"){                   // si el topic es el indicado entra 
    velocidad_led = construyendoNum;                                          // modifica el tiempo de conf (ms)
  }
  Serial.println();
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    // String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);                                  // ID unica de c/u de los clientes
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("G14conexion", "Conectado");                             // publicacion
      // ... and resubscribe
      client.subscribe("infind/GRUPO14/led/cmd");                             // suscripcion LED
      client.subscribe("infind/GRUPO14/led/inversion");                       // suscripcion LED inversion
      client.subscribe("infind/GRUPO14/led/GPIO16");                          // suscripcion SWITCH
      client.subscribe("infind/GRUPO14/led/GPIO16/inversion");                // suscripcion SWITCH inversion
      client.subscribe("infind/II14/ESPX/config");                            // suscripcion configuracion del dispositivo
      client.subscribe("infind/II14/ESPX/config/velocidad");                  // suscripcion configuracion del dispositivo
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
      //client.publish("G14conexion", "Conectado");
}
//----------------------------------------------------------------------SETUP---------------------------------------------------------------
void setup() {
  pinMode(2, OUTPUT);                                                                                 // Initialize the BUILTIN_LED pin as an output
  pinMode(16, OUTPUT);                                                                                // Initialize the BUILTIN_LED pin as an output
  pinMode(boton, INPUT_PULLUP);                                                                       // inicializamos el boton GPI0

  attachInterrupt(digitalPinToInterrupt(boton), RTI, CHANGE);                                         // añadimos interrupción en boton 0
  
  analogWrite(2,255);                                                                                 // Inicia al LED del GPIO2 apagado
  digitalWrite(16,HIGH);                                                                              // Inicia al switch del GPIO16 apagado
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);  
  
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard); 
  dht.setup(5, DHTesp::DHT11);                                                                        // Connect DHT11 sensor to GPIO 5-D1


  snprintf(mensaje_LWTon, 128, "{\"online\": %s}", "true");
  snprintf(mensaje_LWToff, 128, "{\"online\": %s}", "false");
}
//------------------------------------------------------------------------------LOOP--------------------------------------
void loop() {
  // detalles necesarios para el ESP
  if (!client.connected()) { reconnect();}                                                            // en caso de desconexion reconecta al ESP

  if (client.connect(clientId.c_str(), "", "", "infind/GRUPO14/conexion", 0, true, mensaje_LWToff))   // last will
  {client.publish("infind/GRUPO14/conexion", mensaje_LWTon, true);}                                    
  client.loop();
  delay(dht.getMinimumSamplingPeriod());

  if (timePressLimit != 0 && millis()> timePressLimit){
        timePress = 0;                                                                // Cuando se pasa el tiempo se consideran pulsos independiente
        timePressLimit = 0;
        clicks = 0;
        Serial.println("Tiempo superado una pulsación");
        if (Prolong==false){                                                          // si la pulsación no es prolongada 
          analogWrite(2,(int)LEDGuardado);
        } 
  }
                                                        
  // lectura de valores del sensor
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature(); 
  
  unsigned long now = millis();
  if (now - lastMsg > tdatos) {                                                     // si el t_ahora - t_ultMSG > t configuracion de los datos entra                                     
    lastMsg = now;
    cont = cont + tdatos;                                                           // genero el uptime
    ++value;

    // muestra en el monitor serie
    snprintf (msg, MSG_BUFFER_SIZE, "Publico valor nº #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);

    // actualizacion de valores de la config cada tdatos (ms)
    snprintf(mensaje_valoresLEDs, 128, "{\"LEDS\":{\"GPIO2\":%i,\"GPIO16\":%i}}", valorLED2, valorLED16); // genera JSON de valores del GPIO en vivo
    client.publish("infind/GRUPO14/led/status", mensaje_valoresLEDs);                                     // publica en el topic
    

    misdatos.temperatura = temperature;
    misdatos.humedad = humidity;
    misdatos.tiempo = cont;
    
    int valor_vcc = ESP.getVcc();
    
    Serial.println("JSON generado con sprintf:");
    serializa_sprintf(misdatos, cadena, 512, mqtt_server, ssid, valor_vcc);       // generamos cadena en monitor serie
    Serial.println(cadena);
        
    client.publish("G14temperatura",cadena);                                      // publicamos la cadena 
    Serial.println(valor_vcc);                                                    // obtenemos el voltaje de alimentacion del led y lo publicamos
  }
  
}
