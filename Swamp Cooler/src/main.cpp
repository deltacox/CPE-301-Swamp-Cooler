#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "RTClib.h"
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


// using port B as the port for togglable circuit components (LEDs, fan motor)
const unsigned char RED_LED    = 0x10; // PB4
const unsigned char GREEN_LED  = 0x20; // PB5
const unsigned char BLUE_LED   = 0x40; // PB6
const unsigned char YELLOW_LED = 0x80; // PB7
const unsigned char FAN_MOTOR  = 0x01; // PB0

//Timer/Counter Registers
//Water Level Sensor ISR
volatile unsigned char *myTCCR3A  = (unsigned char *) 0x90;   //Timer/Counter3 Control Register A (pg. 156)
volatile unsigned char *myTCCR3B  = (unsigned char *) 0x91;   //Timer/Counter3 Control Register B (pg. 156)
volatile unsigned char *myTCCR3C  = (unsigned char *) 0x92;   //Timer/Counter3 Control Register C (pg. 157)
volatile unsigned char *myTIMSK3  = (unsigned char *) 0x71;   //TimerCounter3 Interrupt Mask Register (pg. 161)
volatile unsigned int  *myTCNT3   = (unsigned  int *) 0x94;   //Timer/Counter3 (pg. 158)
volatile unsigned char *myTIFR3   = (unsigned char *) 0x38;   //Timer/Counter3 Interrupt Flag Register (pg. 162)

//Registers for pushbutton (enable/disable)
volatile unsigned char *portDDR_L = (unsigned char *) 0x10A;   //Port L Data Direction Register (pg. 96)
volatile unsigned char *port_L    = (unsigned char *) 0x10B;   //Port L Data Register (pg. 96)
volatile unsigned char *pin_L     = (unsigned char *) 0x109;   //Port L Input Pins Address (pg.96)

//Global Variable
unsigned int clkstart=1;
unsigned int liquid_level;

Servo myservo;  // create servo object to control a servo
// Servo mapping:
// Brown    => ground
// Red      => VCC
// Orange   => digital 49
// Pot dial => ADC A1

// LCD pin mapping:
//  RS => digital 23
//  E  => digital 25
//  D4 => digital 22
//  D5 => digital 24
//  D6 => digital 26
//  D7 => digital 28
LiquidCrystal lcd(23, 25, 22, 24, 26, 28);
int LCDErrorCode = 0;
const char * ERROR_MESSAGES[] = {
  " ",
  "WATER LEVEL LOW"
};

ISR(TIMER3_OVF_vect){
  unsigned char h2olow[] = {87,65,84,69,82,32,76,69,86,69,76,32,76,79,87,10};

  *myTCCR3B = 0xF8;             //Stops clock, no prescaler (pg.157), by disabling bit 0
  *myTCNT3 = clkstart;          //Sets point for Timer/Counter1 to count up form (pg.158)
 
  //Enters Error State
  if(liquid_level <=300){     
    *port_b = 0x10;             //Turns on PB4-LED if ADC value is less that 300.
     //RTC;                       //Logs date and time when motor turns off
                                
    
    // sets lcd error code to 1, printing the message "WATER LEVEL LOW"
    LCDErrorCode = 1;
  }
  //Transistions to Idle State if water was previously low
  else{

    if(LCDErrorCode){     //Go to Idle State
      *port_b = 0x20;     //Turns off PB4-LED and turns on PB6-LED if ADC value is greater that 300
                          //keeps motor off
    }
    // sets error message to none
    LCDErrorCode = 0;
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

  // initialize LCD screen
  lcd.begin(16, 2);
  //lcd.print("Hello, world!");

  // initializes Servo
  myservo.attach(49);  // attaches the servo on pin 49 to the servo object

  // disabled state control
  *portDDR_L &= 0x10;       //sets PL4 as input
  *port_L    |= 0x10;       //enables pullup resistor

  // initialize RTC Module
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));//auto update from computer time
  
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


void loop() {
  if(!(*pin_L & 0x10)){
    for(volatile unsigned int q=0; q<1000;q++){};     //Debounces read and write to verify but is pressed and not noise
    if(!(*pin_L & 0x10)){
      *myTCCR3B  |= 0x01;                             //Starts clock, (pg 157), by enbling bit 0 for ISR(TIMER3_OVF_vect) 
      //Collect input from ADC for water level
      liquid_level= adc_read(0);                      //takes input from ADC A0 for reading
      lcd.setCursor(0, 0);
      lcd.clear();
      lcd.print(ERROR_MESSAGES[LCDErrorCode]);
    
      //Servo Control
      unsigned int val = adc_read(1);      // reads the value of the potentiometer (value between 0 and 1023)
      val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
      myservo.write(val);                  // sets the servo position according to the scaled value
      delay(15);                           // waits for the servo to get there
    }  
  }
  else{
    *myTCCR3B = 0xF8;                      //Stops clock, no prescaler (pg.157), by disabling bit 0, no more monitoring
    setToggleable(RED_LED, 0);
    setToggleable(GREEN_LED, 0);
    setToggleable(BLUE_LED, 0);
    setToggleable(YELLOW_LED, 1);
  }
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

void toggle(unsigned char destination)
{
    *port_b ^= destination;
}

void setToggleable(unsigned char destination, int logicLevel)
{
    if (logicLevel == 0)
        *port_b &= ~(destination);
    else
        *port_b |= destination;
}

void RTC(){
  char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


  //Writes ADC to *my_UDRO (serial plot)
  for (q=0;q<9;q++){                      //Runs only if transmission is clear and x<8  
    while (!(*my_UCSR0A &(TBE)));
    *my_UDR0= ADC_set[q];
  
}
