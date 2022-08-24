// PRUEBA SENSOR TEMPERATURA Y LUZ INTEGRADOS CON MODO DEEP SLEEP.

#include <OneWire.h>
#include <DallasTemperature.h>

#define SENSOR_PIN 26 // ESP32 pin GIOP21 connected to DS18B20 sensor's DQ pin
#include <Wire.h>     //include library
#define Address 0x4A  // GY-49 I2C Address is 0x4A(74)

OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);

float tempC; // temperature in Celsius
float tempF; // temperature in Fahrenheit

// Define deep sleep options
uint64_t uS_TO_S_FACTOR = 1000000; // Conversion factor for micro seconds to seconds
// Sleep for 5 minutes = 300 seconds
uint64_t TIME_TO_SLEEP = 5;

#include "BluetoothSerial.h"
#include "ESP32Time.h"

#define LED 2 // LED pin is GPIO2

//////////////////////////////////////TEMPERATURA*///////////////////////////////////////////////
void temp()
{
  DS18B20.requestTemperatures();      // send the command to get temperatures
  tempC = DS18B20.getTempCByIndex(0); // read temperature in °C
  tempF = tempC * 9 / 5 + 32;         // convert °C to °F

  Serial.print("Temperature: ");
  Serial.print(tempC); // print the temperature in °C
  Serial.print("°C");
  Serial.print("  ~  "); // separator between °C and °F
  Serial.print(tempF);   // print the temperature in °F
  Serial.println("°F");
}

////////////////////////////////////// LIGHT*/////////////////////////////////////////////////////////
void luxInit()
{
  Wire.beginTransmission(Address); // start wire iic transmission
  Wire.write(0x02);                // Select configuration register
  Wire.write(0x40);                // Continuous mode, Integration time = 800 ms
  Wire.endTransmission();          // Stop iic transmission
}

void lux()
{
  unsigned int data[2];            // variable that storages the data of the sensor
  Wire.beginTransmission(Address); // start wire iic transmission
  Wire.write(0x03);                // Select data register
  Wire.endTransmission();          // Stop iic transmission
  Wire.requestFrom(Address, 1);    // Request 1 byte of data

  if (Wire.available() == 1)
  {
    data[0] = Wire.read(); // stores de fisrt byte of data in the first position
  }

  Wire.beginTransmission(Address); // start wire iic transmission
  Wire.write(0x04);                // Select data register
  Wire.endTransmission();          // Stop iic transmission
  Wire.requestFrom(Address, 1);    // Request 1 byte of data

  if (Wire.available() == 1)
  {
    data[1] = Wire.read(); // Stores the second byte of data in the second position
  }

  // Converts the data to lux
  int exponent = (data[0] & 0xF0) >> 4;
  int mantissa = ((data[0] & 0x0F) << 4) | (data[1] & 0x0F);
  float luminance = (float)(((0x00000001 << exponent) * (float)mantissa) * 0.045);

  // Output data to serial monitor
  Serial.print("Ambient Light Luminance :");
  Serial.print(luminance);
  Serial.println(" lux");
}

///////////////////////////////////////////DEEP SLEEP/////////////////////////////////////////
void DeepSleep()
{
  // Set timer to 5 seconds
  // turn off de peripherals

  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF); // Estas 3 instrucciones no han mostrado diferencia en consumo hasta ahora
  // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // 4000000-5sec,240000000-5minutes
  Serial.println("DONE! Going to sleep now.");
  esp_deep_sleep_start();
}

//****************Conexión BT********************************//

char rx_byte = 0;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int BTDatos = 0;

void BTsetup()
{
  // Serial.begin(9600);
  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  // Serial.end(); //apaga el BT
}

void BTData()
{
  if (SerialBT.available())
  {
    BTDatos = SerialBT.read(); // Toma el caracter.
    if (BTDatos == 25)
    {
      Serial.println("Derecha");
    }
    else if (BTDatos == 75)
    {
      Serial.println("Izquierda");
    }
    else
    {
      Serial.println(BTDatos);
    }
  }
  delay(10);
}

//********************************************MAIN*******************************************//
void setup()
{

  Wire.begin();       // initialize library
  Serial.begin(9600); // initialize serial
  DS18B20.begin();    // initialize the DS18B20 sensor
  luxInit();
  Serial.println("Sensors setup complete");
  BTsetup();
  delay(300);
}

ESP32Time rtc;

void loop()
{
  // lux();
  // temp();
  Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
  BTData();
  DeepSleep();
}
