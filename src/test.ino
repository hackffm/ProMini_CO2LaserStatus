/*
  A4, A5: OLED I2C
  13: LED
  12 TACHO Pump
  11 TACHO Flow-Sensor

  A0 Laser current (1V = 10mA)
  A1 Laser voltage (1V = 10kV)

  2: OneWire

  3: free (grey)
  5: free (orange)

*/

#define PCINT_ENABLE_MANUAL
#define PCINT_ENABLE_PORT0
#define PCINT_ENABLE_PCINT2
#define PCINT_ENABLE_PCINT3
#define PCINT_ENABLE_PCINT4
#define PCINT_ENABLE_PCINT5
#include "PinChangeInterrupt.h"
#include "TimerOne.h"
#include <avr/wdt.h>

#include <U8g2lib.h>
#include <Wire.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define TACHO_PUMP  12
#define TACHO_FLOW  11
#define ONEWIREPIN  2

#define LASER_CURRENT A0
#define LASER_VOLTAGE A1

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.
    
  This is a page buffer example.    
*/

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_NULL u8g2(U8G2_R0);	// null device, a 8x8 pixel display which does nothing
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_ALT0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 16, /* data=*/ 17, /* reset=*/ U8X8_PIN_NONE);   // ESP32 Thing, pure SW emulated I2C
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);   // ESP32 Thing, HW I2C with pin remapping
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, A5, A4, /* reset=*/ U8X8_PIN_NONE);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

volatile uint16_t intTachoPumpCounter = 0;
volatile uint16_t intTachoFlowCounter = 0;
volatile uint16_t intTachoPumpVal = 0;
volatile uint16_t intTachoFlowVal = 0;
uint16_t TachoPumpVal = 0;
uint16_t TachoFlowVal = 0;

OneWire  oneWire(ONEWIREPIN);
DallasTemperature tempSensors(&oneWire);
DeviceAddress tempSensLaserTube   = { 0x28, 0xFF, 0x90, 0x6D, 0x52, 0x17, 0x04, 0x25 }; 
DeviceAddress tempSensWaterIn     = { 0x28, 0xFF, 0x8F, 0x4C, 0x52, 0x17, 0x04, 0xC9 }; 
DeviceAddress tempSensWaterOut    = { 0x28, 0xFF, 0x46, 0xC4, 0x51, 0x17, 0x04, 0x39 }; 
float tempLaserTube, tempWaterIn, tempWaterOut;
#define TEMPERATURE_PRECISION 10 /* 9-12 */

int16_t laserVoltageADC = 0;
int16_t laserCurrentADC = 0;
int32_t laserVoltageV = 0;
int32_t laserCurrentuA = 0;
float laser_kV = 0.0;
float laser_mA = 0.0; 

void setup(void) {
  Serial.begin(115200);
  Serial.println(F("Reset."));
  pinMode(TACHO_PUMP, INPUT_PULLUP);
  pinMode(TACHO_FLOW, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  
  attachPCINT(digitalPinToPCINT(TACHO_PUMP), []() { intTachoPumpCounter++; }, RISING);
  attachPCINT(digitalPinToPCINT(TACHO_FLOW), []() { intTachoFlowCounter++; }, RISING);
  Timer1.initialize(1000000);
  Timer1.attachInterrupt([]() {
    static uint8_t ledState = 0;
    ledState = !ledState;
    digitalWrite(13, ledState?LOW:HIGH);
    intTachoPumpVal = intTachoPumpCounter; intTachoPumpCounter = 0;
    intTachoFlowVal = intTachoFlowCounter; intTachoFlowCounter = 0;
  }); 

  wdt_reset();
  wdt_enable(WDTO_4S); // 4s / System Reset

  u8g2.setBusClock(100000); // make it slow for long cable
  u8g2.begin();
  //u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_6x10_tf);
  //u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  tempSensors.begin();
  tempSensors.setResolution(tempSensLaserTube, TEMPERATURE_PRECISION);
  tempSensors.setResolution(tempSensWaterIn, TEMPERATURE_PRECISION);
  tempSensors.setResolution(tempSensWaterOut, TEMPERATURE_PRECISION);
  printSensorAddresses();


}


// function to print device addresses
void printSensorAddresses()
{
  /*
  int devCount = tempSensors.getDeviceCount();
  Serial.print("#devices: ");
  Serial.println(devCount);

  for(int j=0; j<devCount; j++) {
    DeviceAddress dA;
    tempSensors.getAddress(dA, j);
    Serial.print("Sens");
    Serial.print(j);
    Serial.print(" = {");
    for (uint8_t i = 0; i < 8; i++)
    {
      // zero pad the address if necessary
      Serial.print("0x");
      if (dA[i] < 0x10) Serial.print("0");
      Serial.print(dA[i], HEX);
      if(i < 7) Serial.print(", "); 
      else Serial.println(" };");
    }
  }
  */
}



char sbuf[50];

#define C_NOM   (uint32_t)(30000UL)
#define C_DENOM (uint32_t)(613)
#define V_NOM   (uint32_t)(30000UL)
#define V_DENOM (uint32_t)(613)

void loop(void) {
  // Read sensors
  tempSensors.requestTemperatures();
  tempLaserTube = tempSensors.getTempC(tempSensLaserTube);
  tempWaterIn   = tempSensors.getTempC(tempSensWaterIn);
  tempWaterOut  = tempSensors.getTempC(tempSensWaterOut);

  noInterrupts();
  TachoPumpVal = intTachoPumpVal;
  TachoFlowVal = intTachoFlowVal;
  interrupts();

  laserVoltageADC = analogRead(LASER_VOLTAGE); // 3.00V = 613
  laserCurrentADC = analogRead(LASER_CURRENT);
  
  laserCurrentuA = (((uint32_t)laserCurrentADC * C_NOM) / C_DENOM);
  laserVoltageV  = (((uint32_t)laserVoltageADC * V_NOM) / V_DENOM);
  laser_kV = laserVoltageV / 1000.0;
  laser_mA = laserCurrentuA / 1000.0;

  Serial.print(tempLaserTube,2);
  Serial.print(", ");
  Serial.print(tempWaterIn,2);
  Serial.print(", ");
  Serial.print(tempWaterOut,2);

  Serial.print(" | ");
  
  Serial.print(TachoFlowVal);
  Serial.print(", ");
  Serial.print(TachoPumpVal); 

  Serial.print(" | ");

  Serial.print(laserCurrentADC);
  Serial.print(", ");
  Serial.println(laserVoltageADC);

  // picture loop  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.setFontPosTop();

  u8g2.firstPage();  
  do {
    u8g2.drawStr( 0, 0, "Tube In Out ");
    sprintf_P(sbuf, PSTR("%0.1f °C, %0.1f °C, %0.1f °C"), tempLaserTube, tempWaterIn, tempWaterOut);
    u8g2.drawUTF8(0, 10, sbuf);
    sprintf_P(sbuf, PSTR("%d ADC_I, %d ADC_U"), laserCurrentADC, laserVoltageADC);
    u8g2.drawUTF8(0, 20, sbuf);
    sprintf_P(sbuf, PSTR("%0.1f kV, %0.1f mA"), laser_kV, laser_mA);
    u8g2.drawUTF8(0, 30, sbuf);
    sprintf_P(sbuf, PSTR("%0.1f W_In, %0.1f W_Out"), laser_kV * laser_mA, laser_kV * laser_mA * 0.15);
    u8g2.drawUTF8(0, 40, sbuf);
    sprintf_P(sbuf, PSTR("%d Flow, %d Pump"), TachoFlowVal, TachoPumpVal);
    u8g2.drawUTF8(0, 50, sbuf);
  } while ( u8g2.nextPage() );
  // u8g2.updateDisplay();
  // u8g2.sendBuffer();
  


  // delay between each page
  delay(150);

  wdt_reset();
}