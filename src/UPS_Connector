
/*-----Подключение необходимых библиотек-----*/
#include <ESP8266WiFi.h>                //для работы с сетью
#include <GyverPortal.h>                //для работы страницы настроек, доступной из сети
#include <EEPROM.h>                     //для работы с памятью
#include <ArduinoHA.h>
extern "C" {
#include "user_interface.h"             //для переименования устройства в списке роутера
}

/*-----Объявление необходимых переменных-----*/
GyverPortal portal;                     //переменная портала
WiFiClient espClient;                   //переменная wi-fi клиента, через которого в дальнейшем подключаемся к сети

/*Переменные для обмена с HA*/
HADevice device;
HAMqtt mqtt(espClient, device, 20);     //20 - количество сенсоров. Взято с запасом, на всякий случай. Думаю, что можно уменьшить до реального количества сенсоров
HASensorNumber sIPV("InputVoltage", HASensorNumber::PrecisionP1);
HASensorNumber sFV("FailVoltage", HASensorNumber::PrecisionP1);
HASensorNumber sOPV("OutputVoltage", HASensorNumber::PrecisionP1);
HASensorNumber sOPC("OutputCurrent");
HASensorNumber sIPF("InputFrequency", HASensorNumber::PrecisionP1);
HASensorNumber sBATV("BatteryVoltage", HASensorNumber::PrecisionP1);
HASensorNumber sTEMP("Tempetature", HASensorNumber::PrecisionP1);
HABinarySensor sUF("UtilityFail");//b7
HABinarySensor sLB("LowBattery");//b6
HABinarySensor sBA("Bypass");//b5
HABinarySensor sGF("GeneralFail");//b4
HABinarySensor sOL("OnLine");//b3
HABinarySensor sTP("TestInProgress");//b2
HABinarySensor sSD("ShutDownActive");//b1
HABinarySensor sBP("Beeper");//b0
HASwitch sCF("ConnectionFail");


static DNSServer _SP_dnsServer;         //переменная для DNS сервера

#define SP_AP_IP 192,168,1,1            //IP адрес портала для работы в режиме поднятой точки доступа на ESP
IPAddress apIP(SP_AP_IP);               //переменная для адреса, определенного в строке выше
IPAddress subnet(255, 255, 255, 0);     //подсеть
IPAddress nullAddr(0,0,0,0);            //пустой адрес, для сброса настроек

String WIFI_AP_NAME;                    //наименование точки доступа для подключения к wi-fi
String WIFI_PASSWORD;                   //пароль для подключения к точке доступа
String MQTT_ADR;                        //адрес mqtt сервера
String MQTT_PORT;                       //порт mqtt сервера
String MQTT_USER;                       //имя пользователя для подключения к mqtt
String MQTT_PAS;                        //пароль для  подключения к mqtt
String WIFI_SOFTAP_PAS;                 //пароль для точки доступа, которая поднимается на ESP. Если не задан, то по умолчанию 1234567890
int status = WL_IDLE_STATUS;            //статус соединения Wi-Fi
unsigned long lastReconnectAttempt = 0; //время последней попытки подключения к mqtt
int mqttReconnectsCount = 0;


bool WiFiStatus;                        //Статус Wi-fi подключения к роутеру: 1-ОК, 0 не ОК
bool mode = 0;                          //Переменная режима работы сети: 1 - точка доступа на ESP, 0 - работа в локальной сети

bool isConnectionCrash = 0;             //флаг того, что соединение с Wi-Fi
unsigned long lastCrashReconnect = 0;   //переменная для хранения времени последней попытки подключения при аварии
unsigned long UPSConnectCrash = 0;      //переменная для хранения времени отвала связи с ИБП для попытки восстановления

byte firstStart;                        //Переменная для первого запуска. 137 - НЕ первый старт, всё остальное - первый старт

// Переменные для получения данных от ИБП. Стандарт написан тут: https://networkupstools.org/protocols/megatec.html
float ipv;        //Входное напряжение
float fv;         //Напряжение ошибки
float opv;        //Выходное напряжение
int opc;          //Ток потребителей
float ipf;        //Входная частота тока
float batv;       //Напряжение батареи
float temp;       //Температура
int b7;           //Ошибка сети
int b6;           //Батарея разряжена
int b5;           //Байпас включен
int b4;           //Общая ошибка ИБП
int b3;           //ИБП в режиме ожидания, 0 в работе
int b2;           //Идет тест
int b1;           //Включено завершение работы
int b0;           //Сигнал включен

//Дополнительные переменные
int cDelay = 100;               //Период отправки сообщений. То что тут указано позже в коде меняется на 60000
int cTimer = 0;                 //Переменная таймер для отправки сообщений
int rDelay = 50000;             //Период ожидания ответа от ИБП после отправки команды
int rTimer = 0;                 //Переменная таймер для ожидания ответа
int notGetDataCounter = 0;      //Счётчик неполученных ответов
bool isDataGetted = true;       //Флаг успешного приема данных
bool isUPSUnavailable = false;  //Флаг недоступности ИБП
String devGuid;                 //идентификатор устройства
String HADiscoveryTopic;        //Топик для автоопределения в HA 

/*-----Функция разбора строки, полученной от ИБП-----*/
void parseUPSResponse(String str){
  str = str.substring(1);  //первым символом там скобка или решетка, отрезаем его
  ipv = (str.substring(0,5)).toFloat();
  fv = (str.substring(6,11)).toFloat();
  opv = (str.substring(12,17)).toFloat();
  opc = (str.substring(18,21)).toInt();
  ipf = (str.substring(22,26)).toFloat();
  batv = (str.substring(27,31)).toFloat();
  temp = (str.substring(32,36)).toFloat();
  b7 = (str.substring(37,38)).toInt();
  b6 = (str.substring(38,39)).toInt();
  b5 = (str.substring(39,40)).toInt();
  b4 = (str.substring(40,41)).toInt();
  b3 = (str.substring(41,42)).toInt();
  b2 = (str.substring(42,43)).toInt();
  b1 = (str.substring(43,44)).toInt();
  b0 = (str.substring(44,45)).toInt();
}

/*-----Конструктор страницы портала-----*/
void build(){
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);
  GP.PAGE_TITLE("UPS Control");
  GP.UPDATE("wfst,mqst,sw,pIPV,pFV,pOPV,pOPC,pIFP,pBATV,pTEMP,pUF,pLB,pBA,pGF,pOL,pTP,pSD,pBP",cDelay);

  GP.LABEL("Wi-Fi ");
  GP.LED_GREEN("wfst",WiFiStatus);            
  GP.LABEL(" MQTT ");
  GP.LED_GREEN("mqst",mqtt.isConnected());
  GP.BREAK();
  GP.HR();

  GP.LABEL("UPS error ");
  GP.SWITCH("sw", isUPSUnavailable);
  GP.BREAK();
  GP.HR();
  GP.BREAK();

  GP.BLOCK_BEGIN(GP_THIN,"100%","UPS data");
  GP.BOX_BEGIN();
  GP.LABEL("Input voltage");
  GP.NUMBER_F("pIPV", "", ipv, 1, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Fail voltage");
  GP.NUMBER_F("pFV", "", fv, 1, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Output voltage");
  GP.NUMBER_F("pOPV", "", opv, 1, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Output current");
  GP.NUMBER("pOPC", "", opc, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Input frequency");
  GP.NUMBER_F("pIFP", "", ipf, 1, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Battery voltage");
  GP.NUMBER_F("pBATV", "", batv, 1, "25%", true);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Temperature");
  GP.NUMBER_F("pTEMP", "", temp, 1, "25%", true);
  GP.BOX_END();
  GP.BLOCK_END();

  GP.BLOCK_BEGIN(GP_THIN,"100%","UPS states");
  GP.BOX_BEGIN();
  GP.LABEL("Utility fail");
  GP.LED_RED("pUF",b7);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Low battery");
  GP.LED_RED("pLB",b6);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Bypass");
  GP.LED_RED("pBA",b5);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("General fail");
  GP.LED_RED("pGF",b4);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("On-line");
  GP.LED_GREEN("pOL",!b3);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Test in progress");
  GP.LED_RED("pTP",b2);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Shutdown active");
  GP.LED_GREEN("pSD",b1);
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Beeper");
  GP.LED_GREEN("pBP",b0);
  GP.BOX_END();
  GP.BLOCK_END();

  GP.BREAK();
  GP.HR();
  GP.BREAK();
  
  GP.FORM_BEGIN("/update");
  GP.BLOCK_BEGIN(GP_THIN,"100%","Settings");
  GP.BLOCK_BEGIN(GP_THIN,"95%","Wi-Fi connection");
  GP.BOX_BEGIN();
  GP.LABEL("AP name");
  GP.TEXT("wfname", "Name", WIFI_AP_NAME,"60%");
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("AP pass");
  GP.PASS("wfpas", "Pass", WIFI_PASSWORD,"60%");
  GP.BOX_END();
  GP.BLOCK_END();
  
  GP.BLOCK_BEGIN(GP_THIN,"95%","MQTT connection");
  GP.BOX_BEGIN();
  GP.LABEL("Server");
  GP.TEXT("ser", "Server", MQTT_ADR,"60%");
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Port");
  GP.TEXT("port", "Port", MQTT_PORT,"60%");
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("User");
  GP.TEXT("user", "User", MQTT_USER,"60%");
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("Pass");
  GP.TEXT("pas", "Password", MQTT_PAS,"60%");
  GP.BOX_END();
  GP.BOX_BEGIN();
  GP.LABEL("HA DT","hadtl");
  GP.HINT("hadtl", "HomeAssistant discovery topic");
  GP.TEXT("hadt", "homeassistant", HADiscoveryTopic,"60%");
  GP.BOX_END();
  GP.BLOCK_END();

  GP.LABEL("Passwords for AP on ESP");
  GP.BREAK();
  GP.BOX_BEGIN();
  GP.LABEL("AP Pass");
  GP.TEXT("sappas", "Password", WIFI_SOFTAP_PAS,"60%");
  GP.BOX_END();
  GP.SUBMIT("Save and connect");
  GP.FORM_END();
  GP.BLOCK_END();

  GP.BLOCK_BEGIN(GP_THIN,"300px");
  GP.LABEL("You chose to open this page from local network (LN) or from access point (AP), runing on controller","lbLNAP",GP_DEFAULT,0,0,1); GP.BREAK();
  GP.LABEL("LN");
  GP.SWITCH("mode", mode);
  GP.LABEL("AP");
  GP.BREAK();
  GP.BLOCK_END();
  
  GP.BUILD_END();
}

/*-----Функция записи строки в память. Записывает строку побайтно в память начиная с addrOffset, и возвращает адрес, с которого можно начинать писать следующую строку-----*/
int writeStringToEEPROM(int addrOffset, const String &strToWrite){
  byte len = strToWrite.length();
  EEPROM.begin(512);
  EEPROM.write(addrOffset, len);
  EEPROM.end();
  for (int i = 0; i < len; i++){
    EEPROM.begin(512);
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
    EEPROM.end();
  }
  return addrOffset + 1 + len;
}
/*-----Функция чтения строки из памяти. Побайтно считывает строку начиная с addrOffset, и возвращает адрес, с которого можно начинать читать следующую строку-----*/
int readStringFromEEPROM(int addrOffset, String *strToRead){
  EEPROM.begin(512);
  int newStrLen = EEPROM.read(addrOffset);
  EEPROM.end();
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++){
    EEPROM.begin(512);
    data[i] = EEPROM.read(addrOffset + 1 + i);
    EEPROM.end();
  }
  data[newStrLen] = '\0';
  *strToRead = String(data);
  return addrOffset + 1 + newStrLen;
}

/*-----Функция обработчик событий на портале*/
void action(){
  if (portal.click()){
    if (portal.click("sw")){ //Обработка клика по выключателю
      if (isUPSUnavailable){  
        notGetDataCounter = 0; 
        isUPSUnavailable = false;
      }
    }
    if (portal.click("mode")){ //Обработка клика по переключателю режимов работы сети
      mode = !mode;
      EEPROM.begin(512);
      EEPROM.write(2, mode);
      EEPROM.end();
      if(mode){
        connect_AP();
      }
      else{
        connect_STA();
      }
    }
  }

  // обработка клика по форме
  if (portal.form()) {
    if (portal.form("/update")) {
      //забираем значения и обновляем переменные
      WIFI_AP_NAME = portal.getString("wfname");
      WIFI_PASSWORD = portal.getString("wfpas");
      MQTT_ADR = portal.getString("ser");
      MQTT_PORT = portal.getString("port");
      MQTT_USER = portal.getString("user");
      MQTT_PAS = portal.getString("pas");
      WIFI_SOFTAP_PAS = portal.getString("sappas");
      HADiscoveryTopic = portal.getString("hadt");
      //запишем полученное в память
      int eepromOffset = 4;
      int newStr1AddrOffset = writeStringToEEPROM(eepromOffset, WIFI_AP_NAME);
      int newStr2AddrOffset = writeStringToEEPROM(newStr1AddrOffset, WIFI_PASSWORD);
      int newStr3AddrOffset = writeStringToEEPROM(newStr2AddrOffset, MQTT_ADR);
      int newStr4AddrOffset = writeStringToEEPROM(newStr3AddrOffset, MQTT_PORT);
      int newStr5AddrOffset = writeStringToEEPROM(newStr4AddrOffset, MQTT_USER);
      int newStr6AddrOffset = writeStringToEEPROM(newStr5AddrOffset, MQTT_PAS);
      int newStr7AddrOffset = writeStringToEEPROM(newStr6AddrOffset, WIFI_SOFTAP_PAS);
      int newStr8AddrOffset = writeStringToEEPROM(newStr7AddrOffset, devGuid);
      writeStringToEEPROM(newStr8AddrOffset, HADiscoveryTopic);
      
      mqtt.disconnect();//отключаем mqtt, он в основном цикле включится уже с новыми параметрами
      mqtt.setDiscoveryPrefix(HADiscoveryTopic.c_str()); //обновляем топик для дискавери
      
      //Изменим режим работы сети на работу в локальной сети и сохраним это в памяти
      mode = 0;
      EEPROM.begin(512);
      EEPROM.write(2, mode);
      EEPROM.end();
      //Подключимся к локальной сети
      connect_STA();
    }
  }
  if (portal.update()){//обновляем значения в полях на портале
    portal.updateBool("wfst",WiFiStatus);
    portal.updateBool("mqst",mqtt.isConnected());
    portal.updateBool("sw", isUPSUnavailable);
    portal.updateFloat("pIPV", ipv, 1);
    portal.updateFloat("pFV", fv, 1);
    portal.updateFloat("pOPV", opv, 1);
    portal.updateInt("pOPC", opc);
    portal.updateFloat("pIFP", ipf, 1);
    portal.updateFloat("pBATV", batv, 1);
    portal.updateFloat("pTEMP", temp, 1);
    portal.updateBool("pUF",b7);
    portal.updateBool("pLB",b6);
    portal.updateBool("pBA",b5);
    portal.updateBool("pGF",b4);
    portal.updateBool("pOL",!b3);
    portal.updateBool("pTP",b2);
    portal.updateBool("pSD",b1);
    portal.updateBool("pBP",b0);
  }
}

/*-----Функция подключения к локальной сети-----*/
void connect_STA(){
  WiFi.softAPdisconnect (true);                   //отключаем точку доступа на ESP
  WiFi.softAPConfig(nullAddr, nullAddr, subnet);  //сбиваем её настройки
  _SP_dnsServer.stop();                           //останавливаем DNS сервер
  WiFi.mode(WIFI_STA);                            //переключаем wi-fi модуль в режим подключения к роутеру
  wifi_station_set_hostname("UPS Control");       //устанавливаем наименование устройства (будет видно в списке роутера)
  InitWiFi();                                     //подключаемся к роутеру
  _SP_dnsServer.start(53, "*", apIP);             //стартуем заново DNS сервер
  portal.stop();                                  //Перезапускаем портал на всякий случай
  portal.start();
}

/*-----Функция создания точки доступа на ESP-----*/
void connect_AP(){
  if (WiFi.softAPIP() != apIP){                   //проверяем, что точка доступа уже не поднята
    WiFi.mode(WIFI_AP);                           //переключаем wi-fi модуль в режим точки доступа
    WiFi.softAPConfig(apIP, apIP, subnet);        //настраиваем точку доступа
    //настроим наименование точки доступа в зависимости от наименования клиента
    String SoftAPName = "UPS_Control";
    //настроим пароль точки доступа в зависимости от настроек
    String SoftAPPas;
    if (WIFI_SOFTAP_PAS == ""){                   //если пароль не был задан через портал, то оставляем стандартный, если был задан, то используем его
      SoftAPPas = "1234567890";
    }
    else{
      SoftAPPas = WIFI_SOFTAP_PAS;
    }
    WiFi.softAP(SoftAPName.c_str(), SoftAPPas.c_str()); //включаем точку доступа
    _SP_dnsServer.start(53, "*", apIP);                 //запускаем DNS сервер
    portal.stop();                                      //Перезапускаем портал на всякий случай
    portal.start();
  }
}

/*-----Функция подключения к Wi-fi-----*/
void InitWiFi(){
  int attemptsCount = 0;                    //количество попыток подключения
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);  //пробуем подключиться
  while (WiFi.status() != WL_CONNECTED){    //ждём подключения
    attemptsCount++;
    if (attemptsCount > 60){                //если попыток подключения слишком много, то прекращаем их и устанавливаем флаг плохого подключения
      WiFiStatus = 0;
      break;
    }
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED){
    WiFiStatus = 1;                         //если подключились, то ставим флаг хорошего подключения
  }
}

/*Функция обработчик получения значения от HA по сбросу обрыва связи. Если ранее был выставлен флаг потерянного соединения с ИБП, в HA его можно выключить, и тогда тут снимаем флаг, что позволит попробовать подключиться ещё раз. Отключать связь нельзя, поэтому если это попытались сделать, то посылаем обратно выключенный статус*/
void onSwitchCF(bool state, HASwitch* sender){
  if (isUPSUnavailable && !state){  
    notGetDataCounter = 0; 
    isUPSUnavailable = false;
    sender->setState(state);
  }
  else{
    sender->setState(!state);
  }
}

void setup() 
{
  /*Получим настройки из памяти*/
  EEPROM.begin(512);
  firstStart = EEPROM.read(1);        //проверим бит первого запуска
  EEPROM.end();
  if (firstStart == 137){             //если он равен 137, значит это не первый запуск и можно читать настройки из памяти
    int eepromOffset = 4;             //Начиная с адреса 4, потому что в 1 - бит первого запуска, в 2 - текущий режим работы сети, в 3 - флаг выключенного портала
    int newStr1AddrOffset = readStringFromEEPROM(eepromOffset, &WIFI_AP_NAME);
    int newStr2AddrOffset = readStringFromEEPROM(newStr1AddrOffset, &WIFI_PASSWORD);
    int newStr3AddrOffset = readStringFromEEPROM(newStr2AddrOffset, &MQTT_ADR);
    int newStr4AddrOffset = readStringFromEEPROM(newStr3AddrOffset, &MQTT_PORT);
    int newStr5AddrOffset = readStringFromEEPROM(newStr4AddrOffset, &MQTT_USER);
    int newStr6AddrOffset = readStringFromEEPROM(newStr5AddrOffset, &MQTT_PAS);
    int newStr7AddrOffset = readStringFromEEPROM(newStr6AddrOffset, &WIFI_SOFTAP_PAS);
    int newStr8AddrOffset = readStringFromEEPROM(newStr7AddrOffset, &devGuid);
    readStringFromEEPROM(newStr8AddrOffset, &HADiscoveryTopic);
    EEPROM.begin(512);
    mode = EEPROM.read(2);            //считываем текущий режим работы сети
    EEPROM.end();
  }
  else{                                //Если это первый запуск
    char guid[16]; //генерируем новый идентификатор устройства, записываем его в память, и дальше будем использовать его всегда
    const char possible[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    for(int p = 0, i = 0; i < 16; i++){
      int r = random(0, strlen(possible));
      guid[p++] = possible[r];
    }
    guid[16] = '\0';
    devGuid = guid;
    HADiscoveryTopic = "homeassistant";
    EEPROM.begin(512);
    EEPROM.write(1, 137);             //устанавливаем флаг первого запуска
    EEPROM.end();
    int eepromOffset = 4;             //записываем в память пустоту, чтобы обнулить тот мусор, который там есть с завода
    int str1AddrOffset = writeStringToEEPROM(eepromOffset, "");
    int str2AddrOffset = writeStringToEEPROM(str1AddrOffset, "");
    int str3AddrOffset = writeStringToEEPROM(str2AddrOffset, "");
    int str4AddrOffset = writeStringToEEPROM(str3AddrOffset, "");
    int str5AddrOffset = writeStringToEEPROM(str4AddrOffset, "");
    int str6AddrOffset = writeStringToEEPROM(str5AddrOffset, "");
    int str7AddrOffset = writeStringToEEPROM(str6AddrOffset, "");
    int str8AddrOffset = writeStringToEEPROM(str7AddrOffset, devGuid);
    writeStringToEEPROM(str8AddrOffset, HADiscoveryTopic);
  }
  mqtt.setDiscoveryPrefix(HADiscoveryTopic.c_str()); //обновляем топик для дискавери
  byte devGuidBytes[17];
  devGuid.getBytes(devGuidBytes, sizeof(devGuidBytes));   //Запихнуть строку нельзя, надо приводить к массиву байт. ХЗ, может строка это и есть массив байт, но лучше перестрахуюсь
  device.setUniqueId(devGuidBytes,sizeof(devGuidBytes)); //Устанавливаем наименование устройства
  device.enableExtendedUniqueIds(); //Делаем так, чтобы для сенсоров идентификаторы включали в себя ид устройства
  device.enableSharedAvailability(); //Для поддержания состояния устройства в брокере и HA
  device.enableLastWill();
  
  // подключаем конструктор и обработчик действий портала
  portal.attachBuild(build);
  portal.attach(action);
  //настраиваем список автообновления для переключателя, чтобы можно было управлять выключателем из портала без перезагрузки страницы
  portal.list.init(1);
  portal.list.add(&isUPSUnavailable, "sw", T_CHECK);
  //подключаем возможность OTA обновлений через портал, через отдельную страницу /ota_update
  portal.enableOTA();

  /*---Настройка сенсоров для HA---*/
  sIPV.setIcon("mdi:flash-triangle-outline");
  sIPV.setName("Input voltage");
  sIPV.setUnitOfMeasurement("V");

  sFV.setIcon("mdi:alpha-u-circle-outline");
  sFV.setName("Fail voltage");
  sFV.setUnitOfMeasurement("V");

  sOPV.setIcon("mdi:alpha-u-box-outline");
  sOPV.setName("Output voltage");
  sOPV.setUnitOfMeasurement("V");

  sOPC.setIcon("mdi:alpha-a-box-outline");
  sOPC.setName("Output current");
  sOPC.setUnitOfMeasurement("A");

  sIPF.setIcon("mdi:current-ac");
  sIPF.setName("Input Frequency");
  sIPF.setUnitOfMeasurement("Hz");

  sBATV.setIcon("mdi:car-battery");
  sBATV.setName("Battery voltage");
  sBATV.setUnitOfMeasurement("V");

  sTEMP.setIcon("mdi:thermometer");
  sTEMP.setName("UPS Temperature");
  sTEMP.setUnitOfMeasurement("°C");

  sUF.setIcon("mdi:flash-off-outline");
  sUF.setName("Utility fail");

  sLB.setIcon("mdi:battery-alert-variant-outline");
  sLB.setName("Low battery");

  sBA.setIcon("mdi:flash-alert-outline");
  sBA.setName("Bypass");

  sGF.setIcon("mdi:alert-octagon-outline");
  sGF.setName("General fail");

  sOL.setIcon("mdi:flash");
  sOL.setName("On-Line");

  sTP.setIcon("mdi:battery-clock-outline");
  sTP.setName("Test in progress");

  sSD.setIcon("mdi:power");
  sSD.setName("Shutdown active");

  sBP.setIcon("mdi:volume-high");
  sBP.setName("Beeper");

  sCF.setIcon("mdi:battery-unknown");
  sCF.setName("Connection fail");
  sCF.onCommand(onSwitchCF);

  Serial.begin(2400); //Эта скорость обязательна для обмена по Megatec. Для отладки можно ставить любую
}



void loop() {
  if (WiFi.status() != WL_CONNECTED && !mode){
    connect_STA(); //если отвалился коннект к роутеру и в режиме работы в локальной сети, то пробуем подключиться заново
  }
  if (!portal.tick()){ //проверяем состояние работы портала
      portal.start();
    }  
    
  if (!mode && WiFiStatus){  //если включен режим работы в локальной сети и вайфай подключен, то подключаемся к брокеру
    if(!mqtt.isConnected()){
      if(MQTT_ADR != ""){
        if (millis() - lastReconnectAttempt > 5000 || millis() < 5000){
          mqtt.begin(MQTT_ADR.c_str(), MQTT_PORT.toInt(), MQTT_USER.c_str(), MQTT_PAS.c_str()); //включаем MQTT
        }
      }
    }
    mqtt.loop();//Если поключено, то вызовем тик mqtt
  }
  
  if(!WiFiStatus){  //если соединение с wi-fi отвалилось
    if(isConnectionCrash && millis() - lastCrashReconnect > 180000 && WiFi.softAPgetStationNum() == 0){ //проверяем, что с момента обрыва прошло три минуты и никто не подключен к точке доступа. Это нужно чтобы пытаться восстанавливать связь при обрывах
      if (mode){ //меняем режим на подключение по локальной сети
        mode = false;
        EEPROM.begin(512);
        EEPROM.write(2, mode);
        EEPROM.end();
        isConnectionCrash = false;  //и снимаем флаг того, что соединение отвалилось
      }
    }
    else{
      if (!mode){  //для этого меняем режим, если он не был поменян до этого
        mode = true;
        EEPROM.begin(512);
        EEPROM.write(2, mode);
        EEPROM.end();
        lastCrashReconnect = millis();  //запоминаем время возникновения обрыва
        isConnectionCrash = true;       //и взводим флаг обрыва
      }  
      connect_AP();       //и создаем точку доступа на ESP
    }
  }

  //Раз в период отправки команды и если порт чист (всё считано ранее) отправляем команду на получение данных. Команда не отправляется, если ИБП недоступен
  if (millis() - cTimer > cDelay && !isUPSUnavailable){
    if (cDelay != 60000){ //это для первого прохода. Если не делать, то первая команда отправляется через минуту после старта контроллера, что не очень приятно
      cDelay = 60000;
    }
    if (isDataGetted){
      isDataGetted = false;
      notGetDataCounter = 0;
    }
    else{
      notGetDataCounter++;
      if (notGetDataCounter > 10){
        isUPSUnavailable = true;
        UPSConnectCrash = millis(); //запомним, когда отвалилась связь
        sCF.setState(true); //отправим в HA печальную весть
      }
      else{
        isDataGetted = false;
      }
    }
    if (!isUPSUnavailable){
      Serial.readString(); //очищаем ввод
      Serial.write("F\r");  //Отправляем команду на ИБП для получения данных
      cTimer = millis();
      rTimer = millis();
    }
  }

  //После отправки команды в ИБП пробуем получить ответ и разобрать его. Если ИБП недоступен, то считывание данных не производится
  if (millis() - rTimer < rDelay && !isUPSUnavailable){
    if(Serial.available() > 0 && !isDataGetted && !isUPSUnavailable){ //Если данные получены от ИБП, то разбираем их на переменные
      String str = Serial.readString();
      parseUPSResponse(str);
      isDataGetted = true;
      
      /*Отправим полученные данные в HA*/
      if (mqtt.isConnected()){
        sIPV.setValue(ipv);
        sFV.setValue(fv);
        sOPV.setValue(opv);
        sOPC.setValue(opc);
        sIPF.setValue(ipf);
        sBATV.setValue(batv);
        sTEMP.setValue(temp);
        sUF.setState(b7);
        sLB.setState(b6);
        sBA.setState(b5);
        sGF.setState(b4);
        sOL.setState(!b3);
        sTP.setState(b2);
        sSD.setState(b1);
        sBP.setState(b0);
      }
    }
  }
  if(isUPSUnavailable && millis() - UPSConnectCrash > 300000){ //если связи с ИБП нет, то пробуем раз в 5 минут её восстановить
    notGetDataCounter = 9; //При такой установке проверка будет делаться только один раз, и если связь не восстановилась, то опять уйдёт в ошибку
    isUPSUnavailable = false;
  }

}
