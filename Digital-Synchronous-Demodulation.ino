#include <Wire.h>                    // I2C
#define F_CPU 16000000UL             // Setting Arduino system clock

#include "SerialTransfer.h"          // serial transfer
SerialTransfer myTransfer;

// =========================================================================
const uint32_t ADC_INTERVAL_MICROSECONDS = 30; // 33.33 kHz
const uint32_t SWITCH_CONTROL_INTERVAL_MICROSECONDS = 480; // 2.083kHz
const uint32_t CLOCKS_PER_MICROSECOND = F_CPU / 1000000ul; // 16
const float PRESCALED_CLOCKS_PER_MICROSECOND = CLOCKS_PER_MICROSECOND / 32 / 2; // timer2 presecalar set to 32
const uint32_t TIMER1_TOP = ADC_INTERVAL_MICROSECONDS * CLOCKS_PER_MICROSECOND - 1;
const uint32_t TIMER2_TOP = SWITCH_CONTROL_INTERVAL_MICROSECONDS / 4 - 1;


const uint16_t MAX_RESULTS = 160;

volatile uint16_t Vdec [MAX_RESULTS];
volatile uint16_t Vspst[MAX_RESULTS];
volatile uint16_t spstCounter;
volatile uint16_t decCounter;

long int t1 = 0;
long int t2 = 0;

void setupTimer2(){
  // set Timer2 fast PWM for switch control
  // OC2A at pin D11
  
  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= _BV(COM2A0);               // Toggle OC2A on compare match.

  // Fast PWM, TOP = OCRA
  TCCR2A |= _BV(WGM21) | _BV(WGM20);   // Set to fast pwm
  TCCR2B |= _BV(WGM22);                // Set OCR2A as TOP

  switch (TIMER2_PRESCALER) {
    case 1:
      TCCR2B |= _BV(CS20);
      break;
    case 8:
      TCCR2B |= _BV(CS21);
      break;
    case 32:
      TCCR2B |= _BV(CS21) | _BV(CS20);
      break;
    case 64:
      TCCR2B |= _BV(CS22);
      break;
    case 128:
      TCCR2B |= _BV(CS22) | _BV(CS20);
      break;
    case 256:
      TCCR2B |= _BV(CS22) | _BV(CS21);
      break;
    case 1024:
      TCCR2B |= _BV(CS22) | _BV(CS21) | _BV(CS20);
      break;
    default:
      break;
  }
  OCR2A = TIMER2_TOP;                  // set TOP       
  
  // frequency @ OC2A: f_clk/(64*OCR2A)/2 = 16MHz/(64*62)/2 = 2016.13Hz
}



void setupTimer1(){
  // set Timer1 to trigger ADC when overflow
  
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = _BV(TOIE1);        // Overflow interrupt enabled

  ICR1 = TIMER1_TOP;          // Set TOP
  
  // Set Fast PWM, TOP in ICR1, TOV1 at TOP
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM13) | _BV(WGM12);

  TIFR1 |= _BV(TOV1);         // Clear any pending Timer1 Overflow

  TCCR1B |= _BV(CS10);        // set prescaler to 1

  // frequency: f_clk/(N*ICR1) = 16MHz/(1*320) = 50000Hz
}




void setupADC(){
  // Set up the ADC to start a conversion when Timer1 overflows
  
  // initialized analog channel to pin A0
  // initialized ADLAR=0 for right adjusted (in ADCH and ADCL, the ADC data register)
  ADMUX = 0x40;                         // AVCC with external capacitor at AREF pin

  // set ADC converter register A
  ADCSRA = 0;
  ADCSRA |= _BV(ADEN);                 // enable ADC
  ADCSRA |= _BV(ADATE);                // ADC auto trigger enable
  ADCSRA |= _BV(ADIF);                 // ADC interrupt flag
  ADCSRA |= _BV(ADIE);                 // ADC interrupt enable
  ADCSRA |= _BV(ADPS2);                // set prescaler to 16

  // set ADC converter register B
  ADCSRB = 0;
  ADCSRB |= _BV(ADTS2) | _BV(ADTS1);   // Timer/Counter1 overflow as trigger source
}


ISR(ADC_vect) {
  TIFR1 |= _BV(TOV1);  // Clear the pending Timer1 Overflow Interrupt

  if (decCounter >= MAX_RESULTS && spstCounter >= MAX_RESULTS) {
    ADCSRA = 0;
  } else {
    switch (ADMUX) { // switching between A0 and A1
      case 0x40:
        Vspst[spstCounter++] = ADC;
        ADMUX = 0x41;
        break;
      case 0x41:
        Vdec[decCounter++] = ADC;
        ADMUX = 0x40;
      default:
        break;
    }
  }
}

EMPTY_INTERRUPT (TIMER1_OVF_vect);

void setup() {
  Serial.begin(115200);
  pinMode(11,OUTPUT);

  // ---- Serial Transfer 
  myTransfer.begin(Serial);

  // ---- I2C begin
  Wire.begin();

  sei();
  
  setupTimer2();
  setupTimer1();
  setupADC();
}


void loop() {  

  
  if (decCounter == MAX_RESULTS && spstCounter == MAX_RESULTS) {
    // All samples have been collected.
    uint32_t Von = 0;
    uint32_t on_cnt = 0;
    uint32_t Voff = 0;
    uint32_t off_cnt = 0;
    int32_t demod = 0;
    
    for (int i = 0 ; i < MAX_RESULTS ; i ++) {
      // ==== Serial Transfer
//      char buffer[40];
//      sprintf(buffer, "%d %d", Vspst[i], Vdec[i]);
//      Serial.println(buffer); 
//      Serial.flush();
      
      if (i != 0 && i != MAX_RESULTS - 1) { // ignore edge values
        if (Vspst[i - 1] < 512 && Vspst[i + 1] < 512) { // 1024/2 = 512
          Von += Vdec[i];
          on_cnt ++;
        }
        else if (Vspst[i - 1] > 512 && Vspst[i + 1] > 512) {
          Voff += Vdec[i];
          off_cnt ++;
        }
      }
    }

//    Serial.flush(); // Make sure all the character get sent

//    uint16_t sendSize = 0;
//    sendSize = myTransfer.txObj(Vdec1, sendSize);
//    myTransfer.sendData(sendSize);
    

    // ---- demodulation calculation
    demod = Voff/off_cnt - Von/on_cnt;

//    char x[8];
//    dtostrf(demod, 8, 2, x);

//    Serial.print(Von);
//    Serial.print(" ");
//    Serial.print(on_cnt);
//    Serial.print(" ");
//    Serial.print(Voff);
//    Serial.print(" ");
//    Serial.println(off_cnt);

//     serial transfer
//    uint16_t sendSize = 0;
//    sendSize = myTransfer.txObj(demod, sendSize);
//    myTransfer.sendData(demod);

//    char buffer[8];
//    sprintf(buffer, "%d",demod);
//    Serial.println(buffer); 
//    Serial.flush();
//     Serial.println(demod);
//     Serial.flush();

    //----- time interval
//    t2 = millis();
//    Serial.println(t2 - t1);
//    Serial.flush();
//    t1 = t2;

    // ----- I2C
    char x[8];
    dtostrf(demod, 8, 2, x);
    Wire.beginTransmission(8); // transmit to device #8
    Wire.write(x);        // sends the given value
    Wire.endTransmission();    // stop transmitting
    Serial.println(demod);
    Serial.flush();

    delay(1);
    // ---- reset variables and turn on ADC
    decCounter = 0; //reset counter
    spstCounter = 0;

//    ADCSRA |= _BV(ADEN);                 // enable ADC
      // set ADC converter register A
    ADCSRA = 0;
    ADCSRA |= _BV(ADEN);                 // enable ADC
    ADCSRA |= _BV(ADATE);                // ADC auto trigger enable
    ADCSRA |= _BV(ADIF);                 // ADC interrupt flag
    ADCSRA |= _BV(ADIE);                 // ADC interrupt enable
    ADCSRA |= _BV(ADPS2);                // set prescaler to 16
  }
}
