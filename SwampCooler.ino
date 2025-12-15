//Juan Rivvero
//Include paths if compiling outside Arduino IDE
#include <DHT.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>
#define POT_ADC_CHANNEL 1
#define POT_CENTER 512
#define POT_DEADZONE 10
#define DISABLED 0
#define IDLE 1
#define ERROR 2
#define RUNNING 3


//Registers for GPIO
volatile unsigned char* myPORTD = (unsigned char*)0x2B;
volatile unsigned char* myDDRD = (unsigned char*)0x2A;
volatile unsigned char* myPORTH = (unsigned char*)0x102;
volatile unsigned char* myDDRH = (unsigned char*)0x101;
volatile unsigned char* myPORTL = (unsigned char*)0x10B;
volatile unsigned char* myDDRL = (unsigned char*)0x10A;
volatile unsigned char* myPINL = (unsigned char*)0x109;

//Registers for ADC
volatile unsigned char* myADMUX = (unsigned char*)0x7C;
volatile unsigned char* myADCSRB = (unsigned char*)0x7B;
volatile unsigned char* myADCSRA = (unsigned char*)0x7A;
volatile unsigned char* myADCH = (unsigned char*)0x79;
volatile unsigned char* myADCL = (unsigned char*)0x78;

//Registers for USART
volatile unsigned char* myUDR0 = (unsigned char*)0xC6;
volatile unsigned char* myUBRR0H = (unsigned char*)0xC5;
volatile unsigned char* myUBRR0L = (unsigned char*)0xC4;
volatile unsigned char* myUCSR0C = (unsigned char*)0xC2;
volatile unsigned char* myUCSR0B = (unsigned char*)0xC1;
volatile unsigned char* myUCSR0A = (unsigned char*)0xC0;

//Global DHT
DHT dht11(53, DHT11);

//Global LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Global RTC
RTC_DS1307 rtc;
unsigned long prev_mil = 0;

//Global Stepper
//Stepper stepper(200, 23, 25, 27, 29);
Stepper stepper(2048, 23, 27, 25, 29);

//Program Flags
unsigned int stepper_displaced = 0;
unsigned int system_disabled = 0;
unsigned int system_state = 0;
unsigned int system_state_reported = 0;

//clear adc multiple selection register
void adc_init() {
  *myADMUX = 0b00000000;
  //Set voltage reference to AVcc
  *myADMUX |= 0b01000000;
  //Clear adc control and status register b
  *myADCSRB = 0b00000000;
  //Clear adc control and status register a
  *myADCSRA = 0b00000000;
  //Enable adc
  *myADCSRA |= 0b10000000;
  //Set pre-scaler to 128
  *myADCSRA |= 0b00000111;
}

//Clear input channel selection
unsigned int adc_read(unsigned char adc_input_channel) {
  *myADMUX &= 0b11100000;
  //set input channel selection to 'adc_input_channel'
  *myADMUX |= adc_input_channel & 0b00000111;
  //start analog to digital conversion
  *myADCSRA |= 0b01000000;
  //Wait until conversion is complete
  while (*myADCSRA & 0b01000000)
    ;
  //return conversion result
  return *myADCL | (*myADCH << 8);
}

//Local bufffer
void uint_to_str(unsigned int n, unsigned char* str) {
  unsigned char buff[10];
  //Local index
  unsigned int i = 0;
  //Store digits of 'n' in local bufffer
  do {
    buff[i++] = n % 10 + '0';
    n /= 10;
  } while (n > 0);
  //Reverse 'buff' into 'str'
  for (unsigned int j = 0; j < i; ++j) {
    str[j] = buff[i - j - 1];
  }
  //null-terminate 'str'
  str[i] = '\0';
}

//Set baud rate hi byte to 'usart_baud_rate'
void usart_init(unsigned int usart_baud_rate) {
  *myUBRR0H = (unsigned char)usart_baud_rate >> 8;
  //Set baud rate lo byte to 'usart_baud_rate'
  *myUBRR0L = (unsigned char)usart_baud_rate;
  //Enable reciever and transmitter
  *myUCSR0B = 0b00011000;
  //Set frame format to 8 data bits; 2 stop bits
  *myUCSR0C = 0b00001110;
}

//Wait until there are unread data in rx bufffer
unsigned char usart_rx(void) {
  while (!(*myUCSR0A & 0b10000000))
    ;
  //Return data in rx bufffer
  return *myUDR0;
}

//Wait until there are no data in tx bufffer
void usart_tx_char(unsigned char usart_tx_data) {
  while (!(*myUCSR0A & 0b00100000))
    ;
  //Set tx bufffer to 'usart_tx_data'
  *myUDR0 = usart_tx_data;
}

//tx 'usart_tx_data
void usart_tx_str(unsigned char* usart_tx_data) {
  while (*usart_tx_data) {
    usart_tx_char(*usart_tx_data++);
  }
}
//Local string
void usart_tx_uint(unsigned int usart_tx_data) {
  unsigned char str[11];
  //convert 'usart_tx_data' into 'str'
  uint_to_str(usart_tx_data, str);
  //tx 'str'
  usart_tx_str(str);
}

// DateTime object
void rtc_tx_time(void) {
  DateTime now = rtc.now();
  //tx 'now' as "M / D / Y | H : M : S"
  usart_tx_uint(now.month());
  usart_tx_str(" / ");
  usart_tx_uint(now.day());
  usart_tx_str(" / ");
  usart_tx_uint(now.year());
  usart_tx_str(" | ");
  usart_tx_uint(now.hour());
  usart_tx_str(" : ");
  usart_tx_uint(now.minute());
  usart_tx_str(" : ");
  usart_tx_uint(now.second());
}

//potentiometer control
void stepper_from_pot(void) {
  unsigned int pot = adc_read(POT_ADC_CHANNEL);

  if (pot > POT_CENTER + POT_DEADZONE) {
    // Rotate forward
    stepper.step(10);
  } else if (pot < POT_CENTER - POT_DEADZONE) {
    // Rotate backward
    stepper.step(-10);
  }
  // else: inside deadzone → no movement
}


void setup(void) {  //initialise usart
  usart_init(16000000 / 16 / 9600 - 1);
  //initialise adc
  adc_init();
  //initialise dht
  dht11.begin();
  //initialise lcd
  lcd.begin(16, 2);
  //set initial cursor position to 0,0
  lcd.setCursor(0, 0);
  //set stepper motor speed
  stepper.setSpeed(30);
  //initialise rtc
  rtc.begin();
  //set initial rtc time to compile time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //set interrupt service routine one to 'system_power_isr'
  attachInterrupt(digitalPinToInterrupt(18), system_power_isr, CHANGE);
  //set interrupt service routine two to 'system_reset_isr'
  attachInterrupt(digitalPinToInterrupt(19), system_reset_isr, CHANGE);
  //clear port d data register
  *myPORTD = 0b00000000;
  //clear port d data direction register
  *myDDRD = 0b00000000;
  //set digital pin 18 to recieve input with pullup
  *myPORTD |= 0b00001000;
  //clear port h data register
  *myPORTH = 0b00000000;
  //clear port h data direction register
  *myDDRH = 0b00000000;
  //set digital pin 16 to recieve input with pullup
  *myPORTH |= 0b00000010;
  //set digital pins 7, 8, and 9 directions to out
  *myDDRH |= 0b01110000;
  //clear port l data register
  *myPORTL = 0b00000000;
  //clear port l data direction register
  *myDDRL = 0b00000000;
  //set digital pin 45 to recieve input with pullup
  *myPORTL |= 0b00010000;
  //set digital pin 42 direction to out
  *myDDRL |= 0b10000000;
}

void system_power_isr() {
  system_disabled = !system_disabled;

  if (system_disabled) {
    system_state = DISABLED;
  } else {
    system_state = IDLE;
  }

  system_state_reported = 0;
}

void system_reset_isr() {
  if (system_state == ERROR && adc_read(0) > 20) {
    system_state = IDLE;
    system_state_reported = 0;
  }
}

void loop(void) {
  if (system_state != DISABLED) {
    stepper_from_pot();
    lcd.clear();
    lcd.write("Tem: ");
    lcd.print((unsigned int)dht11.readTemperature());
    lcd.setCursor(0, 1);
    lcd.write("Hum: ");
    lcd.print((unsigned int)dht11.readHumidity());
  }
  switch (system_state) { /* state: disabled */
    case 0:
      /* set led colour to yellow */
      *myPORTH = 0b01100000;
      *myPORTL &= 0b01111111;
      if (!system_state_reported) {
        usart_tx_str("System DISABLED: ");
        rtc_tx_time();
        usart_tx_char('\n');
        system_state_reported = 1;
      }
      if (!system_disabled) {
        system_state = IDLE;
        system_state_reported = 0;
      }
      break;
    /* state: idle */
    case 1:

      *myPORTH = 0b01000000;  // Green LED
      *myPORTL &= 0b01111111;  // Fan OFF indicator

      if (!system_state_reported) {
        usart_tx_str("System IDLE: ");
        rtc_tx_time();
        usart_tx_char('\n');
        system_state_reported = 1;
      }

      // ERROR has highest priority
      if (adc_read(0) < 20) {
        system_state = ERROR;
        system_state_reported = 0;
      }
      // RUNNING only if water is OK
      else if (dht11.readTemperature() > 20) {
        system_state = RUNNING;
        system_state_reported = 0;
      }
      break;
    /* state: error */
    case 2:
      /* set led colour to red */
      *myPORTH = 0b00100000;
      lcd.clear();
      lcd.print("ERROR: Water lvl");
      lcd.setCursor(7, 1);
      lcd.print("is low");
      if (!system_state_reported) {
        usart_tx_str("System ERROR: ");
        rtc_tx_time();
        usart_tx_char('\n');
        system_state_reported = 1;
      }
      break;
    /* state: running */
    case 3:
        *myPORTH = 0b00010000;   // Blue LED
      *myPORTL |= 0b10000000;  // Fan ON

      stepper_from_pot();  // ✅ RUNS CONTINUOUSLY

      if (!system_state_reported) {
        usart_tx_str("System RUNNING: ");
        rtc_tx_time();
        usart_tx_char('\n');
        system_state_reported = 1;
      }

      if (dht11.readTemperature() < 20) {
        *myPORTL &= 0b01111111;  // Fan OFF
        system_state = IDLE;
        system_state_reported = 0;
      } else if (adc_read(0) < 20) {
        *myPORTL &= 0b01111111;  // Fan OFF
        system_state = ERROR;
        system_state_reported = 0;
      }
      break;
  }
  static unsigned long last = 0;
  if (millis() - last >= 1000) {
    last = millis();
  }
}
