#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <DHT.h>
#include <RTClib.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
//CPE 301- Final Project
//Swamp Cooler
//Created by Dylan Cox and Erik Marsh
//20201125
//v1.0

//Registers for UART
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *my_UCSR0A = (unsigned char *)0xC0;    //USART MSPIM Control and Status Register n A (pg. 233)
volatile unsigned char *my_UCSR0B = (unsigned char *)0xC1;    //USART MSPIM Control and Status Register n B (pg. 234)
volatile unsigned char *my_UCSR0C = (unsigned char *)0xC2;    //USART MSPIM Control and Status Register n B (pg. 235)
volatile unsigned int  *my_UBRR0L = (unsigned int  *)0xC4;    //USART MSPIM Baud Rate Registers (pg. 235)
volatile unsigned char *my_UDR0   = (unsigned char *)0xC6;    //USART I/O Data Register n (pg. 218)

//Registers for ADC
volatile unsigned char *my_ADCSRA  = (unsigned char *)0x7A;
volatile unsigned char *my_ADCSRB  = (unsigned char *)0x7B;
volatile unsigned char *my_ADMUX   = (unsigned char *)0x7C;
volatile unsigned int  *my_ADCDATA = (unsigned int  *)0x78;   //Data register is 10 bit and char can only hold 8 bits.

//Define Port B Register Pointers For LEDs
volatile unsigned char *portDDR_b = (unsigned char *) 0x24;   //Port B Data Direction Register (pg. 96)
volatile unsigned char *port_b    = (unsigned char *) 0x25;   //Port B Data Register (pg. 96)

//Timer/Counter Registers
//Water Level Sensor ISR
volatile unsigned char *myTCCR3A  = (unsigned char *) 0x90;   //Timer/Counter3 Control Register A (pg. 156)
volatile unsigned char *myTCCR3B  = (unsigned char *) 0x91;   //Timer/Counter3 Control Register B (pg. 156)
volatile unsigned char *myTCCR3C  = (unsigned char *) 0x92;   //Timer/Counter3 Control Register C (pg. 157)
volatile unsigned char *myTIMSK3  = (unsigned char *) 0x71;   //Timer/Counter3 Interrupt Mask Register (pg. 161)
volatile unsigned int  *myTCNT3   = (unsigned  int *) 0x94;   //Timer/Counter3 (pg. 158)
volatile unsigned char *myTIFR3   = (unsigned char *) 0x38;   //Timer/Counter3 Interrupt Flag Register (pg. 162)

//Registers for pushbutton (enable/disable)
volatile unsigned char *portDDR_L = (unsigned char *) 0x10A;   //Port L Data Direction Register (pg. 96)
volatile unsigned char *port_L    = (unsigned char *) 0x10B;   //Port L Data Register (pg. 96)
volatile unsigned char *pin_L     = (unsigned char *) 0x109;   //Port L Input Pins Address (pg.96)

//LEDs (330 ohm resistor)
//Yellow => pin 13 (PB7)
//Blue   => pin 12 (PB6)
//Green  => pin 11 (PB5)
//Red    => pin 10 (PB4)
// using port B as the port for togglable circuit components (LEDs, fan motor)
const unsigned char RED_LED    = 0x10; // PB4
const unsigned char GREEN_LED  = 0x20; // PB5
const unsigned char BLUE_LED   = 0x40; // PB6
const unsigned char YELLOW_LED = 0x80; // PB7
const unsigned char FAN_MOTOR  = 0x01; // PB0

//RTC DS1307 Module pin mapping:
//SDA => pin20
//SCL => pin 21
RTC_DS1307 rtc;

//Temp/Hmdy Sensor pin mapping: (L to R starting at S)
// pin 32
// VCC
// GND
DHT dht(32, DHT11);
float temperature = 0.0f;
float humidity = 0.0f;

//Water Level Sensor
// + => VCC
// - => GND
// S => A0
unsigned int liquid_level;

//DC Motor Circuit (connect signal to pin 53)
//  https://www.tutorialspoint.com/arduino/arduino_dc_motor.htm
// Servo mapping:
// Brown    => ground
// Red      => VCC
// Orange   => digital 49
// Pot dial => ADC A1
Servo myservo;  // create servo object to control a servo

// LCD pin mapping:
//  RS => digital 23
//  E  => digital 25
//  D4 => digital 22
//  D5 => digital 24
//  D6 => digital 26
//  D7 => digital 28
//  Vss=> ground
//  Vdd=> 5V
//  V0 => ground
//  RW => ground
//  A =>  5V
//  K =>  ground
LiquidCrystal lcd(23, 25, 22, 24, 26, 28);

//Disable Dial (Connect input signal to pin 45)
//    -This code uses LCD Contrast dial control (V0)

int MotorCrntStatus=0;
unsigned int clkstart=1;
unsigned char lastButtonState = 0x00;

// 0 => disabled (yellow light)
// 1 => idle (green light)
// 2 => error (red light)
// 3 => running (blue light)
int systemState = 0;

const float TEMPERATURE_THRESHOLD = 72.0f;


ISR(TIMER3_OVF_vect){
  *myTCCR3B &= 0xF8;             //Stops clock, no prescaler (pg.157), by disabling bit 0
  *myTCNT3 = clkstart;           //Sets point for Timer/Counter1 to count up form (pg.158)
 
  //Enters Error State
  if(liquid_level <=300)
  {     
    systemState = 2;
  }
  //Transistions to Idle State if water was previously low
  else
  {
    if (systemState == 2)
      systemState = 1;
  }
  
  *myTCCR3B |= 0x01;    //Starts clock, (pg 157), by enbling bit 0
}

void setup() {
  U0init(9600);           //setup UART
  adc_init();             //setup ADC
  *portDDR_b |= 0xF1;     //sets PB7,6,5,4,0 as output
  *myTCCR3A   = 0x00;     //set timer registers to zero
  *myTCCR3B   = 0x00;     //^
  *myTCCR3C   = 0x00;     //^
  *myTIMSK3  |= 0x01;     //Enables bit 0 (Timer/Counter, Overflow Interrupt Enable)
  *myTIFR3   |= 0x01;     //Enables bit 0 (Timer/Countern, Overflow Flag)
  *myTCNT3    = clkstart; //Set timer/counter starting point

  while (U0kbhit()==0){}; // wait for RDA = true indicating Serial.available

  // initializes all LEDs to the OFF state
  setToggleable(RED_LED, 0);
  setToggleable(GREEN_LED, 0);
  setToggleable(BLUE_LED, 0);
  setToggleable(YELLOW_LED, 0);
  setToggleable(FAN_MOTOR, 0);

  // initialize LCD screen
  lcd.begin(16, 2);

  // initializes Servo
  myservo.attach(49);  // attaches the servo on pin 49 to the servo object

  // disabled state control
  *portDDR_L &= 0x10;       //sets PL4 as input
  *port_L    |= 0x10;       //enables pullup resistor

  // initialize data log
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //auto update from computer time

  // initialize humidity/temperature sensor
  dht.begin();
}

void loop()
{
  unsigned char currButtonState = *pin_L & 0x10;
  
  if ((currButtonState & 0x10) && !(lastButtonState & 0x10))
  {
    for (volatile unsigned int i = 0; i < 1000; i++);

    if ((*pin_L & 0x10))
    {
      // toggle disabled state
      if (systemState == 0)
      {
        systemState = 1; // idle
        *myTCCR3B |= 0x01; // enable timer
      }
      else
      {
        systemState = 0; // disabled
        *myTCCR3B &= 0xFE; // disable timer
      }
    }
  }

  switch (systemState)
  {
  case 0: // disabled
  {
    setToggleable(YELLOW_LED, 1);
    setToggleable(GREEN_LED, 0);
    setToggleable(RED_LED, 0);
    setToggleable(BLUE_LED, 0);
    if(MotorCrntStatus==1){
      setToggleable(FAN_MOTOR, 0);
      MotorCrntStatus =0;
    }
  }
  break;
  case 1: // idle
  {
    setToggleable(YELLOW_LED, 0);
    setToggleable(GREEN_LED, 1);
    setToggleable(RED_LED, 0);
    setToggleable(BLUE_LED, 0);
    
    if(MotorCrntStatus==1){
      setToggleable(FAN_MOTOR, 0); // explicitly disable fan
      MotorCrntStatus =0;
    }
    
    liquid_level= adc_read(0);                      //takes input from ADC A0 for reading

    temperature = dht.readTemperature(true);
    humidity = dht.readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("*F");
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(humidity);
    lcd.print("%");

    if (temperature >= TEMPERATURE_THRESHOLD)
      systemState = 3; // change to running state
  }
  break;
  case 2: // error
  {
    setToggleable(YELLOW_LED, 0);
    setToggleable(GREEN_LED, 0);
    setToggleable(RED_LED, 1);
    setToggleable(BLUE_LED, 0);

    if(MotorCrntStatus==1){
      setToggleable(FAN_MOTOR, 0); // explicitly disable fan
      MotorCrntStatus =0;
    }
    
    liquid_level = adc_read(0);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WATER LEVEL LOW");
  }
  break;
  case 3: // running
  {
    setToggleable(YELLOW_LED, 0);
    setToggleable(GREEN_LED, 0);
    setToggleable(RED_LED, 0);
    setToggleable(BLUE_LED, 1);
    
    if(MotorCrntStatus==0){
      setToggleable(FAN_MOTOR, 1);
      MotorCrntStatus=1;
    }
    
    liquid_level= adc_read(0);                      //takes input from ADC A0 for reading

    temperature = dht.readTemperature(true);
    humidity = dht.readHumidity();

    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("*F");
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(humidity);
    lcd.print("%");

    if (temperature < TEMPERATURE_THRESHOLD)
      systemState = 1; // change to idle state
  }
  break;
  }
  
  //Servo Control - happens every loop
  unsigned int val = adc_read(1);      // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there

  lastButtonState = currButtonState;
}

void adc_init(){
  //Setup the ADCSRA Register
  *my_ADCSRA |= 0b10000000;      //Enables ADEN: ADC Enable
  *my_ADCSRA &= 0b11000000;      //Disable ADATE: ADCE Auto Trigger Enable (bit 5) and ADIE: ADC Interrupt Enable (bit 3).
                                 //Set bit 2:0 to 0 for a division factor of 2 to slow the reading
  //Setup the ADCSRB Register
  *my_ADCSRB &= 0b11110000;      //Sets MUX5 (bit 3): Analog Channel and Gain Selection Bit to zero
                                 //Sets ADTS (bit 2:0): ADC Auto Trigger Source to zeres to enable free running mode
  //Setup *ADMUX
  *my_ADMUX |= 0b01000000;       //Sets REFS0 (bit 6): Reference Select Bits to 1 to enable AVCC, when REFS1 (bit 7) is zero
  *my_ADMUX &= 0b01000000;       //Sets REFS1 (bit 7) to zero, ADLAR (bit 5): ADC left adjust results
                                 //Sets MUX4:0 (bit4-0): Analog Channel and Gain Selecton Bits to zero to reset channel and gain bits
  //set ADCSRB MUX5 (bit 3) to required channel range                              
  *my_ADCSRB &=0xF7;             //For ADC channels 0-7
}

unsigned int adc_read(unsigned char adc_channel){
  unsigned char muxmask[]= {64,65,66,67,68,69,70,71,64,65,66,67,68,69,70,71};
  unsigned char channel[]= {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
  
  //AMUX mask for bit 2:0 is applied for individual channel used
  for (unsigned int i=0; i<16;i++){
    if (adc_channel == channel[i]){
      *my_ADMUX &=0x00;                 //clears out bits before mask for a clean transfer
      *my_ADMUX |= muxmask[i];          //Mask keeps bit 6 enabled and modifies bit2:0 depending on channel
    }
  }
  
  *my_ADCSRA |= 0b01000000;                //Enable bit 6: ADSC: ADC Start conversion
  while ((*my_ADCSRA &0b01000000) !=0);    //Wait for bit 6 to equal zero meaning that conversion is complete
  return *my_ADCDATA;                      //Returns the resulting conversion data in the ADC data register 
}

//initailizes the communication lines with the desired baud rate
void U0init(unsigned long U0baud)       //Serial.begin()
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *my_UCSR0A = 0x20;                       
 *my_UCSR0B = 0x18;                      
 *my_UCSR0C = 0x06;                      
 *my_UBRR0L = tbaud;                     
}

// Read USART0 RDA status bit and return non-zero true if set
unsigned char U0kbhit()        //Serial.available
{
  if (*my_UDR0 & RDA){       //verifies established communications are clear 
    return 1;                //for the USART I/O Data Register 0
  }
}

void datalog(){
  unsigned char ASCII_num[] = {48,49,50,51,52,53,54,55,56,57};
  unsigned char storage[] = {0,0,0,0,47};               
  unsigned char num[]     = {0,1,2,3,4,5,6,7,8,9};             //11 
  unsigned char q, temp;
  unsigned int data;

  DateTime now =rtc.now();
  
  data=now.year(),DEC;
  storage[3]=data%10;      //Assigns year to array starting at LSB
  temp=data/10;            //Removes LSB from year

  //Puts data into an array
  for (q=3;q>0;q--){
    storage[q-1]=temp%10;      //Assigns year to array starting at LSB
    temp=temp/10;            //Removes LSB from year
  }
  
  //Compares Storage array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<4;q++){
    for(unsigned int x=0; x<10;x++){
      if (storage[q]==num[x]){
        storage[q]=ASCII_num[x];
      }
    }
  }

  //Writes year to *my_UDRO (serial plot)
  for (q=0;q<5;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= storage[q];
  }


  unsigned char holding[] = {0,0,47};
  data=now.month(),DEC;
  holding[1]=data%10;      //Assigns month to array starting at LSB
  temp=data/10;            //Removes LSB from month
  holding[0]=temp%10;      //Assigns month to array starting at LSB

  //Compares ADC array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<2;q++){
    for(unsigned int x=0; x<10;x++){
      if (holding[q]==num[x]){
        holding[q]=ASCII_num[x];
      }
    }
  }
  //Writes month to *my_UDRO (serial plot)
  for (q=0;q<3;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= holding[q];
  }

  data=now.day(),DEC;
  holding[1]=data%10;      //Assigns month to array starting at LSB
  temp=data/10;            //Removes LSB from month
  holding[0]=temp%10;      //Assigns month to array starting at LSB

  //Compares holding array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<2;q++){
    for(unsigned int x=0; x<10;x++){
      if (holding[q]==num[x]){
        holding[q]=ASCII_num[x];
      }
    }
  }
  //Writes day to *my_UDRO (serial plot)
  for (q=0;q<2;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= holding[q];
  }
  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 32;
  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 40;

  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 41;
    while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 32;
  
  data=now.hour(),DEC;
  holding[1]=data%10;      //Assigns month to array starting at LSB
  temp=data/10;            //Removes LSB from month
  holding[0]=temp%10;      //Assigns month to array starting at LSB

  //Compares holding array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<2;q++){
    for(unsigned int x=0; x<10;x++){
      if (holding[q]==num[x]){
        holding[q]=ASCII_num[x];
      }
    }
  }
  //Writes hour to *my_UDRO (serial plot)
  for (q=0;q<2;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= holding[q];
  }

  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 58;
  
  data=now.minute(),DEC;
  holding[1]=data%10;      //Assigns minute to array starting at LSB
  temp=data/10;            //Removes LSB from month
  holding[0]=temp%10;      //Assigns minute to array starting at LSB

  //Compares holding array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<2;q++){
    for(unsigned int x=0; x<10;x++){
      if (holding[q]==num[x]){
        holding[q]=ASCII_num[x];
      }
    }
  }
  //Writes minute to *my_UDRO (serial plot)
  for (q=0;q<2;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= holding[q];
  }
  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 58;

  data=now.second(),DEC;
  holding[1]=data%10;      //Assigns month to array starting at LSB
  temp=data/10;            //Removes LSB from month
  holding[0]=temp%10;      //Assigns month to array starting at LSB

  //Compares ADC array values with 0-9 and writes in ASCII equivalent dec value
  for (q=0;q<2;q++){
    for(unsigned int x=0; x<10;x++){
      if (holding[q]==num[x]){
        holding[q]=ASCII_num[x];
      }
    }
  }
  //Writes second to *my_UDRO (serial plot)
  for (q=0;q<2;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= holding[q];
  }
  while (!(*my_UCSR0A &(TBE)));
  *my_UDR0= 10;
  delay(1000);
}

void setToggleable(unsigned char destination, int logicLevel)
{
    if (logicLevel == 0)
        *port_b &= ~(destination);
    else
        *port_b |= destination;

    if ((destination == FAN_MOTOR)){
      *myTCCR3B = 0xF8;                      //Stops clock, no prescaler (pg.157), by disabling bit 0, no more monitoring
      datalog();
      *myTCCR3B  |= 0x01;                    //Starts clock, (pg 157), by enbling bit 0 for ISR(TIMER3_OVF_vect)
    }
}