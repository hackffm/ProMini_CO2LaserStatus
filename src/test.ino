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

// unclear if those .._ENABLE_... defines work if placed here...
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


// We are using the smaller picture loop page buffer, otherwise the RAM gets full and the MCU crashes

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_NULL u8g2(U8G2_R0);	// null device, a 8x8 pixel display which does nothing
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_ALT0_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
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
  // Init serial out and ports
  Serial.begin(115200);
  Serial.println(F("Reset."));
  pinMode(TACHO_PUMP, INPUT_PULLUP);
  pinMode(TACHO_FLOW, INPUT_PULLUP);
  pinMode(13, OUTPUT); // LED...
  
  // RPM counting: Increment counter in PinChangeInterrupt on rising edge
  attachPCINT(digitalPinToPCINT(TACHO_PUMP), []() { intTachoPumpCounter++; }, RISING);
  attachPCINT(digitalPinToPCINT(TACHO_FLOW), []() { intTachoFlowCounter++; }, RISING);
  // RPM counting: In Timer1 interrupt copy&reset Counters every second
  Timer1.initialize(1000000);
  Timer1.attachInterrupt([]() {
    static uint8_t ledState = 0;
    ledState = !ledState;
    digitalWrite(13, ledState?LOW:HIGH); // LED only for debug, can be turned off
    intTachoPumpVal = intTachoPumpCounter; intTachoPumpCounter = 0;
    intTachoFlowVal = intTachoFlowCounter; intTachoFlowCounter = 0;
  }); 

  // Activate watchdog. Needs Optiboot bootloader! Otherwise endless watchdog-reset.
  wdt_reset();
  wdt_enable(WDTO_4S); // 4s / System Reset

  // Init OLED. For extra long cable to Laser-Front the 10k I2C resistors has been changed by 1k on OLED
  u8g2.setBusClock(100000); // make it slow for long cable
  u8g2.begin();
  //u8g2.setFlipMode(0);
  u8g2.setFont(u8g2_font_6x10_tf);
  //u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);

  // One-wire bus with temperature sensors here
  tempSensors.begin();
  tempSensors.setResolution(tempSensLaserTube, TEMPERATURE_PRECISION);
  tempSensors.setResolution(tempSensWaterIn, TEMPERATURE_PRECISION);
  tempSensors.setResolution(tempSensWaterOut, TEMPERATURE_PRECISION);
  printSensorAddresses();

}


// function to print device addresses
void printSensorAddresses()
{
  // Uncomment the following block to print out your device adresses
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


// Static common buffer used for sprintf
char sbuf[50];

// Nominator and Denominator for calibration (30000V or 30000uA equals ADC value 613)
#define C_NOM   (uint32_t)(30000UL)
#define C_DENOM (uint32_t)(613)
#define V_NOM   (uint32_t)(30000UL)
#define V_DENOM (uint32_t)(613)

void loop(void) {
  // Read sensors, takes longer if resolution is set high
  tempSensors.requestTemperatures();
  tempLaserTube = tempSensors.getTempC(tempSensLaserTube);
  tempWaterIn   = tempSensors.getTempC(tempSensWaterIn);
  tempWaterOut  = tempSensors.getTempC(tempSensWaterOut);

  // Make copy to avoid that value gets overwritten when accessed
  noInterrupts();
  TachoPumpVal = intTachoPumpVal;
  TachoFlowVal = intTachoFlowVal;
  interrupts();

  // get ADC value
  laserVoltageADC = analogRead(LASER_VOLTAGE); // 3.00V = 613
  laserCurrentADC = analogRead(LASER_CURRENT);
  
  // calculate physical values
  laserCurrentuA = (((uint32_t)laserCurrentADC * C_NOM) / C_DENOM);
  laserVoltageV  = (((uint32_t)laserVoltageADC * V_NOM) / V_DENOM);
  laser_kV = laserVoltageV / 1000.0;
  laser_mA = laserCurrentuA / 1000.0;

  // print out all data in one long line via serial
  Serial.print(tempLaserTube,2);
  Serial.print(F(", "));
  Serial.print(tempWaterIn,2);
  Serial.print(F(", "));
  Serial.print(tempWaterOut,2);

  Serial.print(F(" | "));
  Serial.print(TachoFlowVal);
  Serial.print(F(", "));
  Serial.print(TachoPumpVal); 

  Serial.print(F(" | "));
  Serial.print(laserCurrentADC);
  Serial.print(F(", "));
  Serial.println(laserVoltageADC);

  // picture loop  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.setFontPosTop();

  u8g2.firstPage();  
  do {
    // This draws all data on the display ### NOT NICE FORMATED YET ####
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

  // delay between each page
  // delay(150); // nonsense, slow enough anyways...

  wdt_reset();
}