#include "RTClib.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

// Configuración de Wi-Fi y MQTT
const char* ssid = "ProyectosElectronica";                                    //Identificador de red WIFI
const char* password = "proyectos";
const char* MQTTHOST = "192.168.60.80";                       //Dirección IP
const int   MQTTPORT = 1883;
const char* MQTTUSERNAME = "jayrito";                          //Establece el nombre de usuario para la autenticación en el servidor MQTT.
const char* MQTTPASSWORD = "3128148301";
String enviadohost;
uint32_t notConnectedCounter = 0;
WiFiClient client;
// Clientes y publicadores MQTT
Adafruit_MQTT_Client mqtt(&client, MQTTHOST, MQTTPORT, MQTTUSERNAME, MQTTPASSWORD);
Adafruit_MQTT_Publish datos =Adafruit_MQTT_Publish(&mqtt,"luz");

/*
 * @brief Configuración inicial del microcontrolador.
 * @details Inicialización de comunicación serial, conexion a Wi-Fi y MQTT y el temporizador.
 */
RTC_DS3231 rtc;
//DHT_Unified dht(DHTPIN, DHTTYPE);

void setup() {
  
  delay(1);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);               //Conexión a Wifi como cliente- Wi-Fi existente.
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    notConnectedCounter++;
  if(notConnectedCounter > 50) {  // Ajusta este valor según tus necesidades
    Serial.println("Resetting due to Wifi not connecting...");
    ESP.restart();
  }
  }

  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: "); 
  Serial.println(WiFi.localIP());
  MQTT_connect();
  Serial.println("El temporizador se establecio a 10 segundos.");

  // Inicializa el RTC
  if (!rtc.begin()) {
    Serial.println("No se pudo encontrar el RTC");
    while (1);
  }
  //rtc.adjust(DateTime(2024, 5, 27, 15, 45 , 50));  // Configura la fecha y hora al momento de compilación
  if (rtc.lostPower()) {
    Serial.println("RTC perdió el poder, configurando la fecha y hora.");   
  }
  //Inicializa el VEML7700
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

  // Leer la hora del RTC
  DateTime now = rtc.now();

  String fechaHora = String(now.year(), DEC) + "/" +
  String(now.month(), DEC) + "/" +
  String(now.day(), DEC) + " " +
  String(now.hour(), DEC) + ":" +
  String(now.minute(), DEC) + ":" +
  String(now.second(), DEC);
  
  Serial.print("lux: "); Serial.println(veml.readLux());
  float a=veml.readLux();
  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold");
  }
  delay(500);

  // Para la conexion y petcición
    //Se rectifica la conexion a WIFI
    if(WiFi.status()== WL_CONNECTED){
      DynamicJsonDocument doc(1024);
      doc["fecha_y_hora"] = fechaHora;
      doc["luz"] = String(a);
      String output;
      serializeJson(doc, output);
      enviadohost = output;
      Serial.println(enviadohost);
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  datos.publish(enviadohost.c_str(), 2);

  Serial.print("lux: "); Serial.println(veml.readLux());

  irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold");
  }
  delay(500);
  
  WiFi.disconnect(true);
  delay(100); 
  // Configura el ESP32 para entrar en modo de deep sleep durante 55 segundos
  esp_sleep_enable_timer_wakeup(55 * 1000000);  // 55 segundos en microsegundos
  Serial.println("Entrando en modo de deep sleep...");
  Serial.flush();  // Asegura que los mensajes seriales se envíen antes de dormir
  esp_deep_sleep_start();
}

/**
 * @brief Función principal del bucle de ejecución.
 * @details Se haceo solicitud HTTP GET a OpenWeatherMap, obtiene datos climáticos, y publica la información a través de MQTT.
 */
void loop() {

}
void MQTT_connect(){
  int8_t ret;
  if(mqtt.connected()){
    return;
  }
  Serial.print("Conectando a mqtt-");
  uint8_t retries = 10;
  while((ret = mqtt.connect())!=0){
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reintentando");
    mqtt.disconnect();
    delay(1000);
    retries--;
    if (retries==0) {
      Serial.println("No conectado");
    }
  }
  Serial.println("Conectado a mqtt");
}
