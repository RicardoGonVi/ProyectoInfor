
/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include <ArduinoJson.h>

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif
ADC_MODE(ADC_VCC);

DHTesp dht;

// Update these with values suitable for your network.

const char* ssid = "MIWIFI_dgXC";                 // MIWIFI_2G_jvp9
const char* password = "u6EN4QXC";                // qhrKrgbv
const char* mqtt_server = "192.168.1.138";        // 192.168.1.133

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
int cont=0;
volatile int valorLED = 0;
String clientId = "ESP8266Client-";

struct registro_datos {
  float temperatura;
  float humedad;
  unsigned long tiempo;
  };
  
struct registro_datos misdatos;
char cadena[512];
char mensaje_valorLED[128];
char mensaje_LWTon[128];
char mensaje_LWToff[128];

/*
char ID_PLACA[16];
char topic_PUB[256];
char topic_SUB[256];
*/
void serializa_sprintf(struct registro_datos datos, char *cadena, int tamaño, 
                  const char *ip, const char *SSid, int adc)
{
  char comillas='"';
  snprintf(cadena, tamaño, "{\"Uptime\":%d,\"Vcc\":%d, \"DHT11\":{\"temp\":%g,\"hum\":%g}, \"LED\":%i,\"WiFi\":{\"SSid\":%c%s%c, \"IP\":%c%s%c}}",
                        datos.tiempo, adc, datos.temperatura, datos.humedad, valorLED, comillas, SSid, 
                        comillas, comillas, ip, comillas ); // probar a poner %s
}

String serializa_JSON (struct registro_datos datos, const char *ip, const char *SSid, int adc)
{
  StaticJsonDocument<300> jsonRoot;
  String jsonString;
 
  jsonRoot["Uptime"]= datos.tiempo;
  JsonObject DHT11=jsonRoot.createNestedObject("DHT11"); // objeto 1 dht11
  DHT11["temp"] = datos.temperatura;
  DHT11["hum"] = datos.humedad;
  jsonRoot["LED"]= valorLED;
  JsonObject Wifi=jsonRoot.createNestedObject("Wifi"); // objeto 2 wifi
  Wifi["SSid"] = SSid; // probar algo a lo SSid.c_str();
  Wifi["IP"]= ip;
  
  
  serializeJson(jsonRoot,jsonString);
  return jsonString;
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

void callback(char* topic, byte* payload, unsigned int length) {
  int construyendoNum = 0;
  int valorLED_ajustado = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    construyendoNum =  10*construyendoNum + (int(payload[i]) - '0'); 
    
  }
  valorLED = construyendoNum;
  valorLED_ajustado = floor(valorLED*2.55);
  Serial.println();
  
  analogWrite(2,(int) valorLED_ajustado);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    // String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("G14conexion", "Conectado");
      // ... and resubscribe
      client.subscribe("infind/GRUPO14/led/cmd");
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

void setup() {
  pinMode(2, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);  
  
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard); 
  dht.setup(5, DHTesp::DHT11); // Connect DHT11 sensor to GPIO 5-D1


  snprintf(mensaje_LWTon, 128, "{\"online\": %s}", "true");
  snprintf(mensaje_LWToff, 128, "{\"online\": %s}", "false");
}

void loop() {
  if (!client.connected()) { 
    reconnect();
  }

  if (client.connect(clientId.c_str(), "", "", "infind/GRUPO14/conexion", 0, true, mensaje_LWToff))
  {
    client.publish("infind/GRUPO14/conexion", mensaje_LWTon, true);
  }
  client.loop();
  delay(dht.getMinimumSamplingPeriod());

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature(); 
  
  unsigned long now = millis();
  if (now - lastMsg > 15000) {
    lastMsg = now;
    cont = cont + 15000;                // genero el uptime
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "Publico valor nº #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
    
    snprintf(mensaje_valorLED, 128, "{\"led\": %i}", valorLED);
    
    client.publish("infind/GRUPO14/led/status", mensaje_valorLED);

    
    misdatos.temperatura = temperature;
    misdatos.humedad = humidity;
    misdatos.tiempo = cont;
    
    int valor_vcc = ESP.getVcc();
    
    Serial.println("JSON generado con sprintf:");
    serializa_sprintf(misdatos, cadena, 512, mqtt_server, ssid, valor_vcc); // generamos cadena en monitor serie
    Serial.println(cadena);
    Serial.println("JSON generado con ArduinoJson:");
    Serial.println(serializa_JSON(misdatos, mqtt_server, ssid, valor_vcc)); // generamos cade en Json
    
    client.publish("G14temperatura",cadena); // publicamos la cadena 
    Serial.println(valor_vcc); // obtenemos el voltaje de alimentacion del led y lo publicamos
  }
  
}
