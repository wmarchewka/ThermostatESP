#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <WiFiUdp.h>
#include <Modbus.h>
#include <ModbusIP_ESP8266.h>
#include <DHT.h>
#include <TaskScheduler.h>
#include <EEPROM.h>
#include "Adafruit_MCP23017.h"
#include <Wire.h>
#include "SPI.h"
#include "TFT_eSPI.h"
#include "Free_Fonts.h"

//defines
#define WL_NO_SSID_AVAIL     1
#define WL_SCAN_COMPLETED    2
#define WL_CONNECTED         3
#define WL_CONNECT_FAILED    4
#define WL_CONNECTION_LOST   5
#define WL_DISCONNECTED      6

//Modbus Registers Offsets (0-9999)
const int ANALOG_SENSOR_MB_HREG = 1;
const int TEMPERATURE_SENSOR_MB_HREG = 2;
const int HUMIDITY_SENSOR_MB_HREG = 3;
const int THERMOSTAT_HEAT_CALL_PULSE_VALUE_MB_HREG = 4;
const int THERMOSTAT_COOL_CALL_PULSE_VALUE_MB_HREG = 5;
const int THERMOSTAT_FAN_CALL_PULSE_VALUE_MB_HREG = 6;
const int DHT_STATUS_ERR_TIMEOUT_COUNTER_MB_HREG = 100;
const int DHT_STATUS_ERR_CHECKSUM_COUNTER_MB_HREG = 101;
const int DHT_STATUS_ERR_MB_HREG = 102;
const int BLINK_ERROR_CODE_MB_HREG = 103;
const int WIFI_STATUS_ERR_MB_HREG = 104;
const int THERMOSTAT_STATUS_ERR_MB_HREG = 105;
const int ESP_RESET_REASON_MB_HREG = 106;
const int ESP_CHIP_ID_HIGH_MB_HREG = 107;
const int ESP_CHIP_ID_LOW_MB_HREG = 108;

//modbus COILS
const int HEAT_OVERRIDE_MB_COIL = 1;
const int HEAT_CONTROL_MB_COIL = 2;
const int COOL_OVERRIDE_MB_COIL = 3;
const int COOL_CONTROL_MB_COIL = 4;
const int FAN_OVERRIDE_MB_COIL = 5;
const int FAN_CONTROL_MB_COIL = 6;
const int THERMOSTAT_HEAT_CALL_MB_COIL = 7;
const int THERMOSTAT_COOL_CALL_MB_COIL = 8;
const int THERMOSTAT_FAN_CALL_MB_COIL = 9;//const int THERMOSTAT_STATUS_MB_COIL = 10;
const int ESP_RESTART_MB_COIL = 11;

//pin mappping to io expander
const int HEAT_OVERRIDE_PIN = 0;
const int HEAT_CONTROL_PIN = 1;
const int COOL_OVERRIDE_PIN = 2;
const int COOL_CONTROL_PIN = 3;
const int FAN_OVERRIDE_PIN = 4;
const int FAN_CONTROL_PIN = 5;


//pin mappings to esp8266
const int LIGHT_SENSOR_PIN = A0;
const int DHT11_DATA_PIN = 2;
const int I2C_CLOCK_PIN = 4;
const int I2C_DATA_PIN = 5;
const int THERMOSTAT_HEAT_CALL_PIN = 16;
const int THERMOSTAT_COOL_CALL_PIN = 15;
const int THERMOSTAT_FAN_CALL_PIN = 10;

//misc variables
const char* ssid     = "WALTERMARCHEWKA";         //Router login
const char* password = "Alignment67581";
const word eepromCoilOffset = 127;
const word maxEEpromSize = 256;
const word maxHregSize = 128;
const word maxCoilSize = 128;
bool pinstate = 1;
bool COIL_OFF = false;
bool COIL_ON = true;
int status = WL_IDLE_STATUS;
int led_on_counter;
int blinkDefault = 200;
int blinkErrorCode = 1;
long aval;
long humidity;
long temperature;
unsigned long heatPulseDuration;
unsigned long coolPulseDuration;
unsigned long fanPulseDuration;
unsigned long blinkTime;
int errorDHT = 0;
int errorThermostat = 0;
int errorWiFI = 0;
word eepromHregCopy[maxHregSize] = { };
bool eepromCoilCopy[maxCoilSize] = { };

IPAddress ip;
String dhtStatusError;
String networkStatus = "";
String thermostatStatus = "";

//function declarations
void doTempHumidity();
void doWifiStatus();
void doIoControl();
void doThermostatDetect();
void doStatusLED();
void doDebugPrint();
void turnLedOn();
void turnLedOff();
void WrapperCallback();
bool BlinkOnEnable();
void BlinkOnDisable();
void LEDOn();
void LEDOff();
void doProcessErrorCodes();
void doProcessModbus();
void doProcessEEprom();
void restartESP();
void stop();
void testdrawtext(int, int, String, uint16_t,uint16_t);
void tftPrintTest();
void testRoutine();

//classes
ModbusIP mb;                                      //ModbusIP object
DHT dht;                                          //DHY11 object
Adafruit_MCP23017 mcp;
TFT_eSPI tft = TFT_eSPI();
Scheduler runner;

Task taskWrapper            (5000, TASK_FOREVER, &WrapperCallback, NULL , true);
Task taskDoTempHumidity     (2000, TASK_FOREVER, &doTempHumidity);
Task taskDoWifiStatus       (5000, TASK_FOREVER, &doWifiStatus);
Task taskDoThermostatDetect (1000, TASK_FOREVER, &doThermostatDetect);
Task taskDoIoControl        (200, TASK_FOREVER, &doIoControl);
Task taskDoDebugPrint       (1000, TASK_FOREVER, &doDebugPrint);
Task taskBlink              (2000, TASK_ONCE, NULL, NULL, false, &BlinkOnEnable, &BlinkOnDisable);
Task taskLED                (TASK_IMMEDIATE, TASK_FOREVER, NULL, NULL, false, NULL, &LEDOff);
Task taskDoProcessErrors    (500, TASK_FOREVER, &doProcessErrorCodes);
Task taskDoProcessModbus    (1000, TASK_FOREVER, &doProcessModbus);
Task taskDoProcessEEprom    (1000, TASK_FOREVER, &doProcessEEprom);
Task taskMBcoilReg11        (TASK_IMMEDIATE, TASK_ONCE, NULL, NULL, false, NULL, &restartESP);


void testRoutine{


}

void restartESP()
{
  bool debug = 1;
  if (debug) Serial.println("Restarting ESP");
  delay(0);
  ESP.restart();
}

void doProcessModbus()
{
  bool debug = 0;
  if (debug) Serial.println("Processing Modbus...");
  for(int x = 1 ; x <= maxCoilSize; x++)
  {
    if ( eepromCoilCopy[x] != mb.Coil(x) )
    {
      if (debug) Serial.print("mb coil:");
      if (debug) Serial.print(x);
      if (debug) Serial.print(" value:");
      if (debug) Serial.print(mb.Coil(x));
      if (debug) Serial.print(" eepromcopy:");
      if (debug) Serial.println (eepromCoilCopy[x] );
      if (debug) Serial.println (x);

      switch (x)
      {
        case ESP_RESTART_MB_COIL:
        if (debug) Serial.println("case 11");
        mb.Coil(ESP_RESTART_MB_COIL, COIL_OFF);
        eepromCoilCopy[ESP_RESTART_MB_COIL] = int(mb.Coil(x));
        Serial.print("Rebooting unit in 3 seconds...");
        taskMBcoilReg11.enableDelayed(3000);
        break;
      }
    }
  }
}

void doProcessEEprom(){
  //test
  bool debug = 1;
  if (debug) Serial.println("Processing Updated EEPROM...");
  int x;
//check foir hreg change
  for(int x = 1; x < maxHregSize; x++)
  {
    if ( mb.Hreg(x) != eepromHregCopy[x] )
    {
      if (debug) Serial.print("mb hreg:");
      if (debug) Serial.print(x);
      if (debug) Serial.print(" value:");
      if (debug) Serial.print(mb.Hreg(x));
      if (debug) Serial.print(" eepromcopy:");
      if (debug) Serial.println (eepromHregCopy[x] );
      //EEPROM.write( x, int(mb.Hreg(x)) );
      eepromHregCopy[x] = mb.Hreg(x);
    }
  }

  //check for coil change
  for(int y = 1 ; y <= maxCoilSize; y++)
  {
    if ( bool(eepromCoilCopy[y] ) != mb.Coil(y) )
    {
      if (debug) Serial.print("mb coil:");
      if (debug) Serial.print(y);
      if (debug) Serial.print(" value:");
      if (debug) Serial.print(mb.Coil(y));
      if (debug) Serial.print(" eepromcopy:");
      if (debug) Serial.println (eepromCoilCopy[y] );
      if (debug) Serial.print(" y + offset:");
      if (debug) Serial.println (y + eepromCoilOffset);
      //EEPROM.write(y + eepromCoilOffset, int( mb.Coil(y) ) );
      eepromCoilCopy[y] = int( mb.Coil(y) );
      if (debug) Serial.print(" New eepromcopy:");
      if (debug) Serial.println(eepromCoilCopy[y]);
    }
  }
  //EEPROM.commit();
  //if (debug) Serial.println("commit");
}


void doProcessErrorCodes()
{
  bool debug = 0;

  if (debug) Serial.println("Processing Error Codes...");

  if (debug) Serial.print("DHT error code:");
  if (debug) Serial.println(errorDHT);
  if (debug) Serial.print("Wifi error code:");
  if (debug) Serial.println(errorWiFI);
  if (debug) Serial.print("Thermostat error code:");
  if (debug) Serial.println(errorThermostat);

  if (errorDHT != 0)
  {
    blinkErrorCode = errorDHT;
  }
  else if (errorThermostat != 0)
  {
    blinkErrorCode = errorThermostat;
  }
  else
  {
    blinkErrorCode = errorWiFI;
  }

  mb.Hreg(BLINK_ERROR_CODE_MB_HREG, blinkErrorCode);
  if (debug) Serial.print("Blink Error Code:");
  if (debug) Serial.println(blinkErrorCode);
}

void WrapperCallback()
{
    bool debug = 0;
    if (debug) Serial.println("Wrapper callback");
    taskBlink.restartDelayed();
    led_on_counter = 0;
}
// Upon being enabled, taskBlink will define the parameters
// and enable LED blinking task, which actually controls
// the hardware (LED in this example)
bool BlinkOnEnable()
{
    bool debug = 0;
    taskLED.setInterval(200);
    taskLED.setCallback(&LEDOn);
    taskLED.enable();
    taskBlink.setInterval( (blinkErrorCode * 400) - 100 );
    if (debug) Serial.print("Blink Error Code");
    if (debug) Serial.println(blinkErrorCode);
    return true;  // Task should be enabled
}

// taskBlink does not really need a callback method
// since it just waits for 5 seconds for the first
// and only iteration to occur. Once the iteration
// takes place, taskBlink is disabled by the Scheduler,
// thus executing its OnDisable method below.
void BlinkOnDisable()
{
  taskLED.disable();
}

void LEDOn()
{
    digitalWrite(LED_BUILTIN, LOW);
    taskLED.setCallback( &LEDOff);
}

void LEDOff()
{
    digitalWrite(LED_BUILTIN, HIGH);
    taskLED.setCallback( &LEDOn);
}

void doDebugPrint()
{
  Serial.print("LightSensor=");
  Serial.println(aval);
  Serial.print("Temp&Humidity Status=");
  Serial.println(dhtStatusError);
  Serial.print("Humidity=");
  Serial.println(humidity);
  Serial.print("Temperature=");
  Serial.println(temperature);
  ip = WiFi.localIP();
  Serial.print("WiFi Status=");
  Serial.println(WiFi.status());
  Serial.print("IP=");
  Serial.println(ip);
  Serial.print("Heat duration=");
  Serial.println(heatPulseDuration);
  Serial.print("Cool duration=");
  Serial.println(coolPulseDuration);
  Serial.print("Fan duration=");
  Serial.println(fanPulseDuration);
  Serial.print("Thermostat Status=");
  Serial.println(thermostatStatus);
  Serial.print("Error Code:");
  Serial.println(blinkErrorCode);

  Serial.print("Coils=");
  for(int x=23; x >15;x--)
  {
    Serial.print(mb.Coil(x));
  }
  Serial.print(" ");
  for(int x=15; x>7;x--)
  {
    Serial.print(mb.Coil(x));
  }
  Serial.print(" ");

  for(int x=7; x>=0;x--)
  {
    Serial.print(mb.Coil(x));
  }
  Serial.println();

  Serial.print("PINS=");

  for(int x=23; x >15;x--)
  {
    Serial.print(digitalRead(x));
  }
  Serial.print(" ");

  for(int x=15; x >7;x--)
  {
    Serial.print(digitalRead(x));
  }
  Serial.print(" ");

  for(int x=7; x>=0;x--)
  {
    Serial.print(digitalRead(x));
  }
  Serial.println();

  for(int x = 1 ; x <= maxCoilSize; x++)
  {
    Serial.print("Modbus coil ");
    Serial.print(x);
    Serial.print(" value:");
    int val = mb.Coil(x);
    Serial.println(val);
  }
  for(int x = 1 ; x <= maxHregSize; x++)
  {
    Serial.print("Modbus hReg ");
    Serial.print(x);
    Serial.print(" value:");
    int val = mb.Hreg(x);
    Serial.println(val);
  }
}

void doThermostatDetect()
{
  bool debug = 0;

  if (debug) Serial.println("Processing thermostat detect...");
  //each time this is called increment pulseCounter
  //if duration of pulse is ok then increment pulseCounter
  //for each pin
  static int pulseCounter;
  static int heatPulseCounter;
  static int coolPulseCounter;
  static int fanPulseCounter;
  word localval = 0;
  pulseCounter++;
  yield();
  heatPulseDuration = pulseInLong(THERMOSTAT_HEAT_CALL_PIN, LOW, 40000);
  yield();
  coolPulseDuration = pulseInLong(THERMOSTAT_COOL_CALL_PIN, LOW, 40000);
  yield();
  //fanPulseDuration = pulseInLong(THERMOSTAT_FAN_CALL_PIN, LOW, 40000);

  //heatPulseDuration = 0;
  //coolPulseDuration = 0;
  //fanPulseDuration = 0;

  mb.Hreg(THERMOSTAT_HEAT_CALL_PULSE_VALUE_MB_HREG, heatPulseDuration);
  mb.Hreg(THERMOSTAT_COOL_CALL_PULSE_VALUE_MB_HREG, coolPulseDuration);
  mb.Hreg(THERMOSTAT_FAN_CALL_PULSE_VALUE_MB_HREG, fanPulseDuration);

  if (heatPulseDuration > 100) heatPulseCounter++;
  if (coolPulseDuration > 100) coolPulseCounter++;
  if (fanPulseDuration > 100) fanPulseCounter++;

  if (debug) Serial.print(" HP:");
  if (debug) Serial.print(heatPulseDuration);
  if (debug) Serial.print(" CP:");
  if (debug) Serial.print(coolPulseDuration);
  if (debug) Serial.print(" FP:");
  if (debug) Serial.print(fanPulseDuration);

  if (debug) Serial.print(" PC:");
  if (debug) Serial.print(pulseCounter);
  if (debug) Serial.print(" HPC:");
  if (debug) Serial.print(heatPulseCounter);
  if (debug) Serial.print(" CPC:");
  if (debug) Serial.print(coolPulseCounter);
  if (debug) Serial.print(" FPC:");
  if (debug) Serial.print(fanPulseCounter);
  if (debug) Serial.print(" BEC:");
  if (debug) Serial.print(blinkErrorCode);

  if (pulseCounter == 3) {
    pulseCounter = 0;
    if (heatPulseCounter == 3)
    {
      mb.Coil(THERMOSTAT_HEAT_CALL_MB_COIL, COIL_ON);
    }
    else
    {
      mb.Coil(THERMOSTAT_HEAT_CALL_MB_COIL, COIL_OFF);
    }
    heatPulseCounter = 0;

    if (coolPulseCounter == 3)
    {
      mb.Coil(THERMOSTAT_COOL_CALL_MB_COIL, COIL_ON);
    }
    else
    {
      mb.Coil(THERMOSTAT_COOL_CALL_MB_COIL, COIL_OFF);
    }
    coolPulseCounter = 0;

    if (fanPulseCounter == 3)
    {
      mb.Coil(THERMOSTAT_FAN_CALL_MB_COIL, COIL_ON);
    }
    else
    {
      mb.Coil(THERMOSTAT_FAN_CALL_MB_COIL, COIL_OFF);
    }
    fanPulseCounter = 0;
  }

  if (mb.Coil(THERMOSTAT_HEAT_CALL_MB_COIL)) thermostatStatus = "H";
  if (mb.Coil(THERMOSTAT_COOL_CALL_MB_COIL)) thermostatStatus = "C";
  if (mb.Coil(THERMOSTAT_FAN_CALL_MB_COIL)) thermostatStatus = "F";

  //check more than one call. if so then error
  int pulseCheck = int(mb.Coil(THERMOSTAT_HEAT_CALL_MB_COIL) + int(mb.Coil(THERMOSTAT_COOL_CALL_MB_COIL)) \
   + int(mb.Coil(THERMOSTAT_FAN_CALL_MB_COIL)) );
  if (debug) Serial.print(" EC:");
  if (debug) Serial.print(pulseCheck);
  if (pulseCheck > 1)
  {
  thermostatStatus = "E";
  errorThermostat = 9;
  localval = 1;
  }

  if (pulseCheck == 1)
  {
  thermostatStatus = "N";
  errorThermostat = 0;
  localval = 0;
  }
  if (debug) Serial.println(THERMOSTAT_STATUS_ERR_MB_HREG);
  mb.Hreg(THERMOSTAT_STATUS_ERR_MB_HREG, localval);
  if (debug) Serial.print(" TS:");
  if (debug) Serial.println(thermostatStatus);
}

void doIoControl()
{
  bool debug = 0;
  static int counter;
  counter++;
  if (counter == 20)
  {
    if (debug) Serial.println("Processing IO control...");
    counter = 0;
  }
  //read analog to get light sensor value
  aval = analogRead(A0);
  mb.Hreg(ANALOG_SENSOR_MB_HREG, (word) aval);

  //do digital writes
  mcp.digitalWrite(FAN_OVERRIDE_PIN,  (bool)(mb.Coil(FAN_OVERRIDE_MB_COIL)));
  mcp.digitalWrite(FAN_CONTROL_PIN,   (bool)(mb.Coil(FAN_CONTROL_MB_COIL)));
  mcp.digitalWrite(HEAT_OVERRIDE_PIN, (bool)(mb.Coil(HEAT_OVERRIDE_MB_COIL)));
  mcp.digitalWrite(HEAT_CONTROL_PIN,  (bool)(mb.Coil(HEAT_CONTROL_MB_COIL)));
  mcp.digitalWrite(COOL_OVERRIDE_PIN, (bool)(mb.Coil(COOL_OVERRIDE_MB_COIL)));
  mcp.digitalWrite(COOL_CONTROL_PIN,  (bool)(mb.Coil(COOL_CONTROL_MB_COIL)));


}
void doWifiStatus()
{
  bool debug = 0;

  if (debug) Serial.println("Processing WiFI status...");

  if (WiFi.status() != WL_CONNECTED)
    WiFi.begin(ssid, password);
    errorWiFI = WiFi.status();
    mb.Hreg(WIFI_STATUS_ERR_MB_HREG, errorWiFI);
    if (errorWiFI == 0) errorWiFI == 10;
}

void setup()
{
  int wifiCounter = 0;
  word wordTmp = 0;
  bool boolTmp = 0;

  //initialize display
  tft.init();
  tft.setRotation(0);

  Serial.begin(115200);
  delay(4000);
  Serial.println();
  Serial.println();
  Serial.print("Reset Reason:");
  Serial.println(ESP.getResetReason());
  Serial.print("Chip ID:");
  Serial.println(ESP.getChipId());
  Serial.print("Sketch size:");
  Serial.println(ESP.getSketchSize());
  Serial.print("Free sketch size:");
  Serial.println(ESP.getFreeSketchSpace());
  Serial.print("Core version:");
  Serial.println(ESP.getCoreVersion());
  Serial.print("CPU Frequency:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("CPU Frequency:");
  Serial.println("mhz");
  Serial.print("Max EEPROM size:");
  Serial.println(maxEEpromSize);
  EEPROM.begin(maxEEpromSize);  //hregs 0 - 255  coils 256-512

  Serial.println("Booting ESP8266");
  Serial.print("Free RAM ");
  Serial.println(ESP.getFreeHeap());

  mb.config("WALTERMARCHEWKA", "Alignment67581");
  Serial.print("Trying to connect to WiFI...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    wifiCounter++;
    if (wifiCounter > 5){
      Serial.println("Failed! Rebooting...");
      //ESP.restart();
    }
  }


  // i2c mode
  // used to override clock and data pins
  Wire.begin(I2C_DATA_PIN, I2C_CLOCK_PIN);
  mcp.begin(0);

  //setup pins
  Serial.println();
  Serial.println("Initializing IO pins");

  //mcp pin io setup
  mcp.pinMode(HEAT_OVERRIDE_PIN, OUTPUT);
  mcp.pinMode(HEAT_CONTROL_PIN, OUTPUT);
  mcp.pinMode(FAN_OVERRIDE_PIN, OUTPUT);
  mcp.pinMode(FAN_CONTROL_PIN, OUTPUT);
  mcp.pinMode(COOL_OVERRIDE_PIN, OUTPUT);
  mcp.pinMode(COOL_CONTROL_PIN, OUTPUT);

  //esp pin io setup
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(THERMOSTAT_HEAT_CALL_PIN, INPUT);
  pinMode(THERMOSTAT_COOL_CALL_PIN, INPUT);
  //pinMode(THERMOSTAT_FAN_CALL_PIN, INPUT);

  //zero out eeprom registers
  // for(int x = 1; x <= maxEEpromSize; x++)
  // {
  //   EEPROM.write(x, 0);
  // }
  // EEPROM.commit();
  //
  // for(int x = 1; x <= maxEEpromSize; x++)
  // {
  //   int tmp = EEPROM.read(x);
  //   Serial.print(tmp);
  // }


  //create modbus registers and copy contents from eeprom
  Serial.print("Creating Modbus Holding Registers Max size : ");
  Serial.println(maxHregSize);
  for(int x = 1; x <= maxHregSize; x++)
    {
      mb.addHreg(x);
      //wordTmp = EEPROM.read(x);
      wordTmp = x;
      mb.Hreg(x, wordTmp);
      eepromHregCopy[x] = mb.Hreg(x);
    }

  //create coil registers and copy contents from eepromCopy
  Serial.print("Creating Modbus Coil Registers Max size : ");
  Serial.println(maxCoilSize);
  for(int x = 1; x <= maxCoilSize; x++)
    {
      mb.addCoil(x);
      //tmp = EEPROM.read(x);
      mb.Coil(x, COIL_OFF);
      eepromCoilCopy[x] = COIL_OFF;
    }

  //shutdown thermostat override
  Serial.println("Shutting down Thermostat override and control...");
  mb.Coil(HEAT_OVERRIDE_MB_COIL, COIL_OFF);     //COIL 1
  mb.Coil(HEAT_CONTROL_MB_COIL, COIL_OFF);      //COIL 2
  mb.Coil(COOL_OVERRIDE_MB_COIL, COIL_OFF);     //COIL 3
  mb.Coil(COOL_CONTROL_MB_COIL, COIL_OFF);      //COIL 4
  mb.Coil(FAN_OVERRIDE_MB_COIL, COIL_OFF);      //COIL 5
  mb.Coil(FAN_CONTROL_MB_COIL, COIL_OFF);       //COIL 6

  //set additional startup coil states


  //setup dht11 pin
  Serial.println("Initializing DHT11 sensor...");
  dht.setup(DHT11_DATA_PIN);

  Serial.println("Initializing OTA update routines...");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Starting OTA update");
  });

  ArduinoOTA.onEnd([]()
  {
    Serial.println("\nOTA Update complete");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Update Ready..");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Adding Tasks...");
  //setup task TaskScheduler
  runner.init();
  Serial.println("Adding task taskDoTempHumidity...");
  runner.addTask(taskDoTempHumidity);
  Serial.println("Adding task taskDoWifiStatus...");
  runner.addTask(taskDoWifiStatus);
  Serial.println("Adding task taskDoThermostatDetect...");
  runner.addTask(taskDoThermostatDetect);
  Serial.println("Adding task taskDoIoControl...");
  runner.addTask(taskDoIoControl);
  Serial.println("Adding task taskWrapper...");
  runner.addTask(taskWrapper);
  Serial.println("Adding task taskDoDebugPrint...");
  runner.addTask(taskDoDebugPrint);
  Serial.println("Adding task taskDoProcessErrors...");
  runner.addTask(taskDoProcessErrors);
  Serial.println("Adding task taskDoProcessModbus...");
  runner.addTask(taskDoProcessModbus);
  Serial.println("Adding task taskBlink...");
  runner.addTask(taskBlink);
  Serial.println("Adding task taskLED...");
  runner.addTask(taskLED);
  Serial.println("Adding task taskMBcoilReg11...");
  runner.addTask(taskMBcoilReg11);

  Serial.println("Enabling Tasks...");
  taskDoTempHumidity.enable();
  taskDoWifiStatus.enable();
  taskDoThermostatDetect.enable();
  taskDoIoControl.enable();
  taskWrapper.enable();
  taskBlink.enable();
  taskDoDebugPrint.disable();
  taskDoProcessErrors.enable();
  taskDoProcessEEprom.disable();
  taskDoProcessModbus.enable();

  //update registers with important data
  //mb.Hreg(ESP_RESET_REASON_MB_HREG, ESP.getResetReason());

  unsigned long chipID = ESP.getChipId();
  word idLowWord = chipID;
  unsigned long chipIDtmp = chipID >> 16;
  word idHighWord = (word) (chipIDtmp);
  mb.Hreg(ESP_CHIP_ID_HIGH_MB_HREG , idHighWord );
  mb.Hreg(ESP_CHIP_ID_LOW_MB_HREG , idLowWord );

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(0);
  //tft.print("ChipID:");
  //tft.println(44444);
  Serial.println("Starting...");
}


void doTempHumidity()
{
  bool debug = 0;
  static int err1 = 0;
  static int err2 = 0;

  if (debug) Serial.println();
  //tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(4);
    if (debug) Serial.println("Processing temperature and humity sensor...");

  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  temperature = dht.toFahrenheit(temperature);
  dhtStatusError = dht.getStatusString();
  if (debug) Serial.println(dhtStatusError);

  if (dhtStatusError == "OK")
  {
    mb.Hreg(TEMPERATURE_SENSOR_MB_HREG, (word)(humidity));
    mb.Hreg(HUMIDITY_SENSOR_MB_HREG, (word)(temperature));
    mb.Hreg(DHT_STATUS_ERR_MB_HREG, 0);
    errorDHT = 0;
    char buf1[4];
    char buf2[4];
    sprintf (buf1, "%03i", temperature);
    sprintf (buf2, "%03i", humidity);
    testdrawtext(20, 45, buf1, ST7735_WHITE, TFT_BLACK);
    testdrawtext(20, 75, buf2, ST7735_WHITE, TFT_BLACK);
  }

  if (dhtStatusError == "TIMEOUT"){
     errorDHT = 7;
     err1++;
     mb.Hreg(DHT_STATUS_ERR_MB_HREG, 1);
     testdrawtext(0, 45, dhtStatusError, ST7735_WHITE, TFT_BLACK);
  }
  mb.Hreg(DHT_STATUS_ERR_TIMEOUT_COUNTER_MB_HREG, err1);

  if (dhtStatusError == "CHECKSUM"){
    errorDHT = 8;
    err2++;
    mb.Hreg(DHT_STATUS_ERR_MB_HREG, 2);
    testdrawtext(0, 45, dhtStatusError, ST7735_WHITE, TFT_BLACK);
  }

  mb.Hreg(DHT_STATUS_ERR_CHECKSUM_COUNTER_MB_HREG, err2);
  if (debug) Serial.println(dhtStatusError);
}

void testdrawtext(int wid, int hei, String text, uint16_t textcolor,uint16_t backcolor) {

  bool debug = 0;
  String realText = "                ";

  unsigned int start = millis();
  //if (debug) Serial.println(text);
  tft.setCursor(wid, hei);
  tft.setTextColor(textcolor, backcolor);
  tft.setTextWrap(true);
  tft.print(realText);
  tft.setCursor(wid, hei);
  tft.print(text);
  unsigned int end = millis();
  unsigned int elapsed = end - start;
  if (debug) Serial.println(elapsed);

}

void stop()
    {
      Serial.print("Sleeping");
      ESP.deepSleep(20e6);
    }

void loop()
{
  yield();
  mb.task();
  ArduinoOTA.handle();
  runner.execute();
}

#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
