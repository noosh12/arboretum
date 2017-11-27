#include <OneWire.h> 
#include <avr/sleep.h> 
#include <avr/power.h>
#include <Wire.h>
#include <DS1337.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "SeeeduinoStalker.h"

//Datalogger
//The following code is taken from sleep.h as Arduino Software v22 (avrgcc) in w32 does not have the latest sleep.h file
#define sleep_bod_disable() \
{ \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
}
DS1337 RTC; //Create RTC object for DS1337 RTC
static uint8_t prevSecond=0; 
static DateTime interruptTime;
static uint16_t interruptInterval = 30; //21600 seg. = 6 horas.
SoftwareSerial XBee(0, 1); // arduino RX,TX (XBee Dout, Din)

////////////////Setup Variables/////////////////
  //Soilmoisture 1
  float vwc1;
  int SoilMoisture1;
  //Soilmoisture 2 
  float vwc2;
  int SoilMoisture2;
  //Battery
  float voltage;
  int BatteryValue;

// Temp Sensor 
int DS18S20_Pin = 7; //DS18S20 Signal pin on digital 8

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 8


 //starttime = millis();//get the current time;
//////////////////////////////////////////////////////////////////////////
void setup() {

   /*Initialize INT0 pin for accepting interrupts */   
  PORTD |= 0x04; 
  DDRD &=~ 0x04;
  
  
  analogReference(INTERNAL);// To increase the resolution, currently in 154 (air) to 525 (for water)
  Wire.begin();
  Serial.begin(57600);
  RTC.begin();
  ////////////////////////////////////////////////////////////////////////
 //Setting digital pins to turn on different devices
  pinMode(8, OUTPUT);// Relay control pin for the 2 soil sensors. HIGH ON
  //digitalWrite(8, HIGH);
       
  pinMode(9,OUTPUT);//XBEE power control pin. HIGH = On, LOW = Off
  //digitalWrite(9, HIGH);
    //////////////////////////////////////////////////////////////////////
     
  attachInterrupt(0, INT0_ISR, LOW); //Only LOW level interrupt can wake up from PWR_DOWN
     set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
     //Enable Interrupt 
     //RTC.enableInterrupts(0,0,30); //interrupt at  EverySecond, EveryMinute, EveryHour or configured by RTC.enableInterrupts(h, m, s) . 
     // or this
    DateTime  start = RTC.now();
    interruptTime = DateTime(start.get() + interruptInterval); //Add interruptInterval in seconds to start time
  //////////////////////////////////////////////////////////////////////
}

void loop() {
  
  // Temperature Sensor
  float temperature = getTemp();
  
    //Soil Sensor 1
    SoilMoisture1 = analogRead(A1);
    vwc1 = (SoilMoisture1 * 0.0015 - 0.234)* 100;  //Decagon manual and my calculations
    delay(1000);

    //Soil Sensor 2
    SoilMoisture2 = analogRead(A2);
    vwc2 = (SoilMoisture2 * 0.0015 - 0.234)* 100;  //Decagon manual and my calculations
  
  
  // Battery reading
  BatteryValue = analogRead(A7);
  voltage = BatteryValue * (1.1 / 1024)* (10+2)/2;  //Voltage divider
  
  DateTime now = RTC.now(); //get the current date-time    

     Serial.print("NODE1");
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(",");
    Serial.print(now.date(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(" ");
    Serial.print(voltage);
    Serial.print(" ");
    Serial.print(temperature);
    Serial.print(" ");
    Serial.print(vwc1);
    Serial.print(" ");
    Serial.print(vwc2);
    Serial.println();
    delay(100);
    
    ////////////////////////END : Application code //////////////////////////////// 
    RTC.clearINTStatus(); //This function call is a must to bring /INT pin HIGH after an interrupt.
    RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
    sleep_enable();      // Set sleep enable bit
    attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
    //Power Down routines
    cli(); 
    sleep_enable();      // Set sleep enable bit
    sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
    sei();

    digitalWrite(8, LOW);    // Power off the 2 soil sensors 
    delay(2000); //This delay i
    digitalWrite(9, LOW); //xbee power off
    
   
    delay(2000); //This delay is required to allow print to complete
    //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
    power_all_disable(); //This shuts down ADC, TWI, SPI, Timers and USART
    sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)  
    sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
    power_all_enable();  //This shuts enables ADC, TWI, SPI, Timers and USART
    delay(1000); //This delay is required to allow CPU to stabilize
   // Serial.println("Awake from sleep");  
    // Relay control
    digitalWrite(9, HIGH); //xbee power on 
    delay(2000);
    digitalWrite(8, HIGH);    // Power on the 2 soil sensors and Temperature sensor
    delay(5000);
    
    
    //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 
//////////////////////////////////////////////////////////////////////////////////////////////////
//returns the temperature from one DS18S20 in DEG Celsius
float getTemp(){
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end
  delay(750);
  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt service routine for external interrupt on INT0 pin conntected to DS1337 /INT
void INT0_ISR()
{
  //Keep this as short as possible. Possibly avoid using function calls
    detachInterrupt(0); 
    interruptTime = DateTime(interruptTime.get() + interruptInterval);  //decide the time for next interrupt, configure next interrupt  
}


