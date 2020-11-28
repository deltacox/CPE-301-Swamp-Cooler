#include <Arduino.h>
#include <LiquidCrystal.h>
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
volatile unsigned char *myTCCR1A  = (unsigned char *) 0x80;   //Timer/Counter1 Control Register A (pg. 156)
volatile unsigned char *myTCCR1B  = (unsigned char *) 0x81;   //Timer/Counter1 Control Register B (pg. 156)
volatile unsigned char *myTCCR1C  = (unsigned char *) 0x82;   //Timer/Counter1 Control Register C (pg. 157)
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6F;   //TimerCounter1 Interrupt Mask Register (pg. 161)
volatile unsigned int  *myTCNT1   = (unsigned  int *) 0x84;   //Timer/Counter1 (pg. 158)
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;   //Timer/Counter1 Interrupt Flag Register (pg. 162)

//Global Variable
unsigned int clkstart=1;
unsigned int liquid_level;

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

ISR(TIMER1_OVF_vect){ //MODIFY CODE to toggle PB4 "RED" LED from water sensor
  unsigned char h2olow[] = {87,65,84,69,82,32,76,69,86,69,76,32,76,79,87,10};

  *myTCCR1B = 0xF8;             //Stops clock, no prescaler (pg.157), by disabling bit 0
  *myTCNT1 = clkstart;          //Sets point for Timer/Counter1 to count up form (pg.158)
 
  //Enters Error State
  if(liquid_level <=300){     
    *port_b = 0x10;             //Turns on PB4-LED if ADC value is less that 127.
                                //turn off motor and all LEDs
    
    // //Writes water level low to *my_UDRO (serial monitor)
    // for (unsigned int q=0;q<16;q++){     //Runs only if transmission is clear and x<8  
    //   while (!(*my_UCSR0A &(TBE)));
    //   *my_UDR0= h2olow[q];
    //  }

    // sets lcd error code to 1, printing the message "WATER LEVEL LOW"
    LCDErrorCode = 1;
  }
  //Transistions to Idle State
  else{
    *port_b = 0x28;     //Turns off PB4-LED and turns on PB6-LED if ADC value is greater that 300
    //Needs to turn off motor

    // sets error message to none
    LCDErrorCode = 0;
  }
  
  *myTCCR1B |= 0x01;    //Starts clock, (pg 157), by enbling bit 0
}

void setup() {
  U0init(9600);           //setup UART
  adc_init();             //setup ADC
  *portDDR_b |= 0xF0;     //sets PB7,6,5,4 as output
  *myTCCR1A   = 0x00;     //set timer registers to zero
  *myTCCR1B   = 0x00;     //^
  *myTCCR1C   = 0x00;     //^
  *myTIMSK1  |= 0x01;     //Enables bit 0 (Timer/Counter, Overflow Interrupt Enable)
  *myTIFR1   |= 0x01;     //Enables bit 0 (Timer/Countern, Overflow Flag)
  *myTCNT1    = clkstart; //Start timer/counter starting point
  *myTCCR1B  |= 0x01;     //Starts clock, (pg 157), by enbling bit 0
  *my_ADMUX   = 0x40;     //Mask keeps bit 6 enabled and modifies bit2:0 to use ADC A0
  while (U0kbhit()==0){}; // wait for RDA = true indicating Serial.available

  // initializes all LEDs to the OFF state
  setToggleable(RED_LED, 0);
  setToggleable(GREEN_LED, 0);
  setToggleable(BLUE_LED, 0);
  setToggleable(YELLOW_LED, 0);

  // initialize LCD screen
  lcd.begin(16, 2);
  //lcd.print("Hello, world!");
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
  //Collect input from ADC
  liquid_level= adc_read(0);       //takes input from ADC A0 for reading
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print(ERROR_MESSAGES[LCDErrorCode]);
}


unsigned int adc_read(unsigned char adc_channel){
  
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
    return 1;               //for the USART I/O Data Register 0
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