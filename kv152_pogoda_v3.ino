

/*
 * Система КВ152. Метеостанция.
 * ---------------------------
 * 
 * Работает на модуле 
 *   - ESP8266 NodeMCU.
 * Сбор данных с датчиков:
 *   - движения
 *   - температуры и влажности HTU21D 
 *   - давления и температуры  BMP280
 * Отображение данных
 *   - LCD1602 через модуль i2c
 * Отправка данных
 *  - на сервер mqtt  (на сервере программа, получив измерения 
 *    метеостанции отправит их в narodmon.ru)
 * 
 * Arduino IDE поддержка ESP8266 NodeMCU
 * --------------------------------------
 * Файл->Настройки
 * В пункт "Дополнительные ссылки для Менеджера плат" вставляем
 * http://arduino.esp8266.com/stable/package_esp8266com_index.json
 * 
 * Инструменты -> Платы -> Менеджет плат
 * в поиск вставить esp8266
 * Установить esp8266 by ESP8266 Community
 * 
 * Выбрать плату NodeMCU
 * 
 * (см Adafuit MQTT Library)
 * 
 * Библиотека
 * -----------
 * LCD1602 I2C
 * 
 * Меню -> Скеч -> Подключить библиотеку -> Управлять библиотеками -> Диалог Менеджер библиотек
 * В строке поиска ввести LiquidCrystal I2C
 * Найти и установить LiquidCrystal I2C by Frank de Brabander
 * 
 * mqtt
 * Меню -> Скеч -> Подключить библиотеку -> Управлять библиотеками -> Диалог Менеджер библиотек
 * В строке поиска ввести PubSubClient
 * Установить PubSubClient by Nick O'Leary.
 * 
 * HTU21D
 * Меню -> Скеч -> Подключить библиотеку -> Управлять библиотеками -> Диалог Менеджер библиотек
 * В строке поиска ввести HTU21D
 * Установить SparkFunHTU21D.
 * 
 * BMP280
 * Меню -> Скеч -> Подключить библиотеку -> Управлять библиотеками -> Диалог Менеджер библиотек
 * В строке поиска ввести Adafruit Unified Sensor
 * Установить Adafruit Unified Sensor.
 *  
 * Меню -> Скеч -> Подключить библиотеку -> Управлять библиотеками -> Диалог Менеджер библиотек
 * В строке поиска ввести Adafruit_BMP280
 * Установить Adafruit_BMP280.
 * 
 * Подключение
 * ------------
 * HTU21D    ----   ESP8266
 * SCL        -     D1   (GPIO 5)
 * SDA        -     D2   (GPIO 4)
 * +          -     3v3
 * -          -     GND
 * 
 * 
 * BMP280    ----   ESP8266
 * SCL        -     D1
 * SDA        -     D2
 * +          -     3v3
 * -          -     GND
 *   Подключение датчика движения
 *   Vin VCC
 *   GND GND
 *   D3  OUT
 * 
 */


#include <kv152_cfg.h>
  
//Определяем, где работает программа: на Ардуине?, тогда true
#define INARDUINO  true
#define DEBUG      false

//Для отладки: использовать ли датчик температуры и влажности
#define USE_HTU21D true
//Для отладки: использовать ли датчик давления
#define USE_BMP280 true
//Для отладки: использовать WIFI
#define USE_WIFI   true
//Для отладки: использовать MQTT
#define USE_MQTT   true

 #define MQTT_TOPIC_MOTION         "kv152/pogoda/motion"
 #define MQTT_TOPIC_TEMP_VAL       "kv152/pogoda/temp/value"
 #define MQTT_TOPIC_TEMP_TIME      "kv152/pogoda/temp/time"
 #define MQTT_TOPIC_HUM_VAL        "kv152/pogoda/hum/value"
 #define MQTT_TOPIC_HUM_TIME       "kv152/pogoda/hum/time"
 #define MQTT_TOPIC_BMP_PRESS_VAL  "kv152/pogoda/press/value"
 #define MQTT_TOPIC_BMP_PRESS_TIME "kv152/pogoda/press/time"
 #define MQTT_TOPIC_BMP_TEMP_VAL   "kv152/pogoda/temp2/value" //температура внутри
 #define MQTT_TOPIC_BMP_TEMP_TIME  "kv152/pogoda/temp2/time" 
 #define MQTT_TOPIC_IN             "kv152/pogoda/value" //байт 0 - команде: 1 == отобразить последующие 4 байта; со 2-го 4-байта параметр




//Пауза в мс для цикличной поддержки связи с MQTT и проверки наличия движения
//Для поддержки связи с MQTT требуется систематические запросы в серверу, чтобы он нас не отключил
//но не так часто, как выполняется основной цикл Ardino
//Одновременно датчик движения требуется также достаточно часто опрашивать, чтобы
//он как можно быстро реагировал на присутствие человека
#define DELAY_MQTT 1000

//Пауза с мс между опросами датчиков погоды (погода не моментально меняет показания)
#define DELAY_POGODA 25000


#if USE_MQTT
  //Настройка MQTT сервера
  const char *mqtt_server    = MQTTSERVER;
  const char *mqtt_clientId  = "pogoda";
  const char *mqtt_user      = MQTTUSER;
  const char *mqtt_password  = MQTTPASSWORD;
#endif

  //Найстройка Wifi роутера
  const char* ssid     = WIFISSID;
  const char* password = WIFIPASSWORD;


#if INARDUINO
  #include <Arduino.h>
  #include <Wire.h> 
  #include <ESP8266WiFi.h>
  #include <PubSubClient.h>
  #include <Adafruit_BMP280.h>                            // Подключаем библиотеку Adafruit_BMP280
  #include <Adafruit_Sensor.h>                            // Подключаем библиотеку Adafruit_Sensor
  #include <SparkFunHTU21D.h>
  #include <LiquidCrystal_I2C.h>
  


 

  WiFiClient espClient;
 #if USE_MQTT 
  PubSubClient client( espClient );
#endif
  LiquidCrystal_I2C lcd(0x27,16,2);
  
  long lastMsg = 0;
  char msg[50];
  int value = 0;
  
#else
  #include <cstdlib>
  using namespace std;
  #include <stdio.h>
  #include <string.h>
  #include <cmath>
  #include "Serial.h"
  #include "Arduino.h"

  #include "Adafruit_BMP280.h"
  #include "Adafruit_Sensor.h"
  #include "SparkFunHTU21D.h"

  
  ObjSerial Serial;
  
#endif 


 #if USE_HTU21D
   HTU21D myHumidity;
 #endif

 int BmpIsValid = 0;
 #if USE_BMP280 
  Adafruit_BMP280 BMP; 
#else
   //Если не используем датчик, тогда соот-й флаг
   BmpIsValid = 0;  
#endif

  //Датчик движения
  #define PinOfPir  0
  int  PIRst     = LOW;
  int  PIRst_old = LOW;



  /**
   * Старые значение влажности и температуры, чтоб лишний раз не отправлять
   */
  float HTU21D_HumOld = -1;
  float HTU21D_TempOld = -1;
  float HTU21D_TempMin = 32768;
  float HTU21D_TempMax = -32768;


  /**
   * Старые значение давления и температуры, чтоб лишний раз не отправлять
   */
  float BMP_PressOld = -1;
  float BMP_TempOld = -1;


  


/*
 * Инициализация дисплея
 */
void display_init(){
 #if INARDUINO
  lcd.init(); 
  lcd.noCursor();
  lcd.backlight();
#else
    printf("void display_init()\r\n");
#endif  
  return;
}

/**
 * Очистка дисплея LCD1602
 */
void display_clear(){
 #if INARDUINO
   lcd.clear(); 
#else
    printf("void display_clear()\r\n");
#endif  
  
}
  

//неопределено
#define STATE_UNKNOWN 0
//подключение 
#define STATE_CONNECTING 1
//ошибка подключения 
#define STATE_ERROR 2
//успешно подключено
#define STATE_ONLINE 3 





/**
 * Отобразить строку на LCD1602
 * @param byte lineno  номер строки на дисплее (1..)
 * @param byte pos     номер позиции в строке (1...)
 * @param str          отображаемый текст
 */
void display_string(byte lineno, byte pos, char *str){
  #if DEBUG
    Serial.print("display_string(");Serial.print(lineno);Serial.print(",");Serial.print(pos);
    Serial.print(",");Serial.print(str);
    Serial.println(")");
  #endif 
  
#if INARDUINO
  lcd.setCursor(pos,lineno);
  lcd.print(str);
#else
    printf("display_string(%d,%d,'%s');\r\n", lineno, pos, str);
#endif    
}


/**
 * Включитть\выключить подсвеку дисплея LCD1602
 * @param BOOL turnon
 */
void display_backlight( bool turnon ){
#if INARDUINO
  if(turnon) lcd.backlight();
  else       lcd.noBacklight();
#else
    printf("void display_backlight( bool %d );\r\n", turnon);
#endif

}



/**
 * Отображение состояния связи по WiFi
 * @param byte type  0 - состояние по wifi
 *                   1 - состояние по mqtt
 * @param byte state см STATE_
 */
void displayConnectStatus( byte type, byte state ){
  char str[32];
  byte lineno = 0 ;
  
  if( type == 0 ){ 
      //Статус Wifi отобразить в первой строке
      lineno=0; 
      strcpy(str, "WIFI:");
  } else
  if( type == 1 ){
      //Статус Wifi отобразить во второй строке
      lineno = 1;
      strcpy(str, "MQTT:");
  } else
      strcpy(str, "err:");
    
 //расшифровка статуса и оторажение  
  if( state == STATE_UNKNOWN ){
      strcpy(str, strcat(str, "unkn      ") );
      display_string(lineno, 0, str);
    }
  if( state == STATE_CONNECTING ){
      strcpy(str, strcat(str, "connecting") );
      display_string(lineno, 0, str);
    }
  if( state == STATE_ERROR ){
      strcpy(str, strcat(str, "error     ") );
      display_string(lineno, 0, str);
    }
  if( state == STATE_ONLINE ){
      strcpy(str, strcat(str, "ok        ") );
      display_string(lineno, 0, str);
    }
    return;
}




/**
 * Настройка WiFi
 */
void wifi_init() {

    displayConnectStatus( 0, STATE_UNKNOWN );
    delay(100);
  
    #if DEBUG
      Serial.print("Connecting to ");
      Serial.println(ssid);
    #endif

    displayConnectStatus( 0, STATE_CONNECTING );
    #if INARDUINO
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED){
        delay(500);
        #if DEBUG
          Serial.print(".");
        #endif
       }
      randomSeed(micros());
      #if DEBUG
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
      #endif
    #endif
  
  displayConnectStatus( 0, STATE_ONLINE );
  return;
}




#if USE_MQTT
/**
 * MQTT колбэк подписки.
 * Пришло сообщение от сервера, выполни преписанную команду
 */
void mqtt_callback(char* topic, byte* payload, unsigned int len) {
      byte cmd;
    byte value[4];
    
    display_clear();
#if DEBUG
  Serial.print("mqtt_callback : [");
  Serial.print(topic);
  Serial.println("]");
  Serial.print(" publish data is:");
  for (int i = 0; i < len; i++) {
    Serial.print( (char)payload[i] );
  }
#endif
/*
   //Ставлю жесткое ограничение: длина 5 байт
   //первый - это код команды
   //со второго 4 байта - параметр/выводимое сообщение
   if( len == 5 ){
    cmd = payload[0];
   for (int i = 0; i < len; i++)value[i] = payload[i+1];
  //Сообщение печатаем с начала панели, внутри процедуры длинное сообщение разделиться на подстроки
  //LcdPrint(0, 0, value, length  );
   if( cmd == (byte)'1' );
   display_raw( value, len);
   }
*/
  //поув выводим все (контроль длины внутри процедуры)
//  display_raw( payload, len);
#if DEBUG
Serial.println("");
#endif
} //end mqtt_callback
#endif




#if USE_MQTT
/**
 * MQTT переподключение
 */
void mqtt_reconnect() {
  char msg[21];
  // Loop until we're reconnected
  displayConnectStatus( 1, STATE_CONNECTING );
  #if INARDUINO 
    while (!client.connected())  {
      #if DEBUG
        Serial.print("Attempting MQTT connection...");
      #endif
  
    
 
//Подуключаемся к mqtt серверу
      if (client.connect(mqtt_clientId, mqtt_user, mqtt_password  )){
        #if DEBUG
          Serial.println("connected");
        #endif
        displayConnectStatus(1, STATE_ONLINE );
        //once connected to MQTT broker, subscribe command if any
        client.subscribe(MQTT_TOPIC_IN);
      } else {
       #if DEBUG
         Serial.print("failed, rc=");
         Serial.print(client.state());
         Serial.println(" try again in 5 seconds");
       #endif
       displayConnectStatus(1, STATE_ERROR );
       // Wait 6 seconds before retrying
       delay(6000);
      }
   }
  #endif
 return; 
} //end reconnect()
#endif




void mqtt_publish_str(const char *topic, const char *value){

    #if DEBUG
    Serial.print("mqtt_publish_str(");Serial.print(topic);Serial.print(",");Serial.print(value);
    Serial.println(")");
  #endif 
  
    #if INARDUINO
      #if USE_MQTT 
        client.publish(topic, value);
     #endif   
   #else     
        printf("void mqtt_publish(const char '%s', const char '%s')\r\n", topic,value);
#endif     
}

void mqtt_publish_int(const char *topic, int value){
   char str[30];
  #if DEBUG
    Serial.print("mqtt_publish_int(");Serial.print(topic);Serial.print(",");Serial.print(value);
    Serial.println(")");
  #endif 
     
   sprintf(str, "%d", value);
    #if INARDUINO 
       #if USE_MQTT
        client.publish(topic, str);
       #endif 
   #else     
        printf("void mqtt_publish(const char '%s', int %s)\r\n", topic,str);
#endif 
    
}


void mqtt_publish_float(const char *topic, float value){
   char str[30];
  #if DEBUG
    Serial.print("mqtt_publish_float(");Serial.print(topic);Serial.print(",");Serial.print(value);
    Serial.println(")");
  #endif 
     
   sprintf(str, "%f", value);  
    #if INARDUINO 
    #if USE_MQTT
        client.publish(topic, str);
    #endif    
   #else     
        printf("void mqtt_publish(const char '%s', float %s)\r\n", topic,str);
#endif     

  
}



/**
 * Обновить показания погоды на дисплее
 */
void updateDisplay(){
  char str[17];

  sprintf(str, "%3.1f%c %2.0f%% %3.0fmm",HTU21D_TempOld,223, HTU21D_HumOld,BMP_PressOld );
  display_string(0, 0,str);
  sprintf(str, "%3.0f%c(%3.0f%c %3.0f%c)",BMP_TempOld, 223, HTU21D_TempMin,223,HTU21D_TempMax,223 );
 display_string(1, 0,str);
  
  
}


#if USE_HTU21D
/**
 * Опрос датчика влажности и температуры
 * Получить значения
 * Отобразить значения
 * Отправить на MQTT сервер
 */
void HumidityUpdate(){
float humd = myHumidity.readHumidity();
float temp = myHumidity.readTemperature();


if( abs( HTU21D_HumOld - humd ) > 0.1 ){
  mqtt_publish_float(MQTT_TOPIC_HUM_VAL,  humd);
  mqtt_publish_int(MQTT_TOPIC_HUM_TIME, millis() );
}
if( abs( HTU21D_TempOld - temp ) > 0.1 ){
  mqtt_publish_float(MQTT_TOPIC_TEMP_VAL, temp);
  mqtt_publish_int(MQTT_TOPIC_TEMP_TIME, millis() );
}

  if( temp < HTU21D_TempMin ) HTU21D_TempMin = temp;
  if( temp > HTU21D_TempMax)  HTU21D_TempMax = temp;

  #if DEBUG
Serial.print("Time:");
Serial.print(millis());
Serial.print(" Temperature:");
Serial.print(temp, 1);
Serial.print("C");
Serial.print(" Humidity:");
Serial.print(humd, 1);
Serial.print("%");

Serial.println();
#endif

HTU21D_HumOld = humd;
HTU21D_TempOld = temp;
}

#endif



#if USE_BMP280
/**
 * Опрос датчика давления и температуры
 * Получить значения
 * Отобразить значения
 * Отправить на MQTT сервер
 */
void BMPUpdate(){
   if( BmpIsValid ){
    float pressVal = BMP.readPressure() / 100.0F * 0.7500638;
    float temp = BMP.readTemperature();
    if( abs( BMP_PressOld - pressVal ) > 0.1 ){
      mqtt_publish_float(MQTT_TOPIC_BMP_PRESS_VAL,  pressVal);
      mqtt_publish_int(MQTT_TOPIC_BMP_PRESS_TIME,  millis() );
    }
    if( abs( BMP_TempOld - temp ) > 0.1 ){
      mqtt_publish_float(MQTT_TOPIC_BMP_TEMP_VAL, temp);
      mqtt_publish_int(MQTT_TOPIC_BMP_TEMP_TIME , millis() );
    }
#if DEBUG
    Serial.print("Time:");
    Serial.print(millis());
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.print(" Press:");
    Serial.print(pressVal, 1);
    Serial.print("mm");
    Serial.println();
#endif

    BMP_PressOld = pressVal;
    BMP_TempOld = temp;
   }else{
    BMP_PressOld = 0;
    BMP_TempOld = 0;
    }
}
#endif



/**
 * Функция проверки датчика движения.
 * Выполнения операций на начало\завершения движения.
 * Отправки на MQTT сервер статус движения
 */
void motionDetection(){
    #if INARDUINO  
      PIRst = digitalRead(PinOfPir);
    #else  
      PIRst = (rand() > 596516649)?HIGH:LOW;
    #endif 


    if (PIRst_old == LOW && PIRst == HIGH) {
      #if DEBUG
        Serial.println("motion on");
      #endif
      display_backlight( true );
      mqtt_publish_int(MQTT_TOPIC_MOTION, 1);
    }
    
    if (PIRst_old == HIGH && PIRst == LOW) {
      #if DEBUG
         Serial.println("motion off");
      #endif
      display_backlight( false );
      mqtt_publish_int(MQTT_TOPIC_MOTION, 0);
    }
    PIRst_old = PIRst;
    
  return;   
}



/**
 * 
 */
void setup() {
  #if DEBUG
    Serial.begin(115200);
    Serial.println("setup->dispay_init");
  #endif
  display_init();  


  #if USE_WIFI
    #if DEBUG
      Serial.println("setup->wifi init");
    #endif  
    wifi_init();
  #endif  

#if INARDUINO 
  #if USE_MQTT
  #if DEBUG
    Serial.println("setup -> mqtt_init");
  #endif     
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);
  client.loop();
  #endif
  pinMode (PinOfPir, INPUT);
#endif

#if USE_HTU21D   
 myHumidity.begin();
#endif

#if USE_BMP280  
 BmpIsValid = BMP.begin(BMP280_ADDRESS_ALT,BMP280_CHIPID);

 if( BmpIsValid ){
    /* Default settings from datasheet. */
  BMP.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }else{
  #if DEBUG
    Serial.println("Sensor BMP280 is invalid! ");
  #endif     
    }
#endif




   #if DEBUG
    Serial.println("setup->return");
  #endif    
return;  
}



 
unsigned long  cdelay_mqtt   = DELAY_MQTT;
unsigned long  cdelay_pogoda = DELAY_POGODA;


/**
 * Основной цикл программы
 */
void loop() {


  
 #if INARDUINO  
   unsigned long curTime = millis();

   //Обработка состояния MQTT (не в каждом цикле Ардуино, но реже)
   if( ( curTime - cdelay_mqtt ) > DELAY_MQTT ){
/*    
  #if DEBUG
    Serial.println("loop -> time -> mqtt");
  #endif    
*/  
    #if USE_MQTT
      if (!client.connected()) {
        mqtt_reconnect();
      }
     client.loop();
    #endif
     cdelay_mqtt = curTime;

     //Опрос датчика движения (этот датчик чаще остальных требуется проверять, чтоб моментально на
     //движение человека реагировал)
     motionDetection();     
   }   
  #endif

 #if INARDUINO
  //Опрос дачиков
  if( ( curTime - cdelay_pogoda ) > DELAY_POGODA ){
#endif
  #if DEBUG
    Serial.print(curTime);
    Serial.println(" loop -> time -> sersors");
  #endif   
     //Опрос датчика движения - переместил в более быстрый цикл
    //motionDetection();
#if USE_HTU21D   
    //Опрос датчика температуры и влажности
    HumidityUpdate();
#endif  

#if USE_BMP280  
    //Опрос датчика давления и температуры
    BMPUpdate();
#endif    
#if INARDUINO
  updateDisplay();
  cdelay_pogoda = curTime;
  }
#endif    
  
  
  return; 
}
 





#if INARDUINO
#else
int main(int argc, char** argv) {

    setup();
    
    while( 1==1 ) loop();
    
    return (EXIT_SUCCESS);
}
#endif
