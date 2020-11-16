/* Zscope307_2CH  a simple oscilloscope program to 
 *  acquire and display time varying signals with an 
 *  Arduino and computer.
 *  Samples two analog pins and also outputs a sqaure wave at an 
 *  integer fraction of the sample rate.
 *  Run this with the serial plotter or monitor to collect your
 *  data.
 *  You can pull the scopeSelectPin to GND to put it into MATLAB scope mode
 *  
 *  Aleks Zosuls Boston University 2020
 */
#include <avr/interrupt.h>
#include <avr/io.h>

#define SAMPLE_PERI_SEC 0.002
#define PRESCALER 64
#define OSC_PERI_NSEC 62.5

int scopeSelectPin = 12;
bool scopePinState;
bool outState = true; //the state of the digital square wave pin
float DCOffsetNull1 = 0; //tune this to subtract your rail splitter voltage
float DCOffsetNull2 = 0; //tune this to subtract your rail splitter voltage
float gain1 = 1;  //software gain, useful for calibration
float gain2 = 1;  //software gain, useful for calibration
volatile int z = 0;  //square wave iterator
int valOCR1A;
int squareWaveOut = 8;  // pin that outputs a square wave
int squareHalfCycSamples = 250; //number of analog samples betwen
//square wave transistions
int anaInput1 = 0;  //the analog pin that is the scope input
int anaInput2 = 1;
int ch1Binary;
int ch2Binary;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(squareWaveOut,OUTPUT);
  pinMode(scopeSelectPin, INPUT_PULLUP); 
  /*  if left open the pin will go high
      thus selecting the arduino plotter format. If you pull it to GND it will go into 
      economo MATLAB 2 channel scope mode
  */
  //Serial.println("2 Channel Z scope. Outputs a square on D8");
  scopePinState = digitalRead(scopeSelectPin);
  if(scopePinState == 1){ //if set high for Arduino plottr..  
  Serial.println("ch1, ch2");
  }
  valOCR1A = (int) (SAMPLE_PERI_SEC / (PRESCALER * OSC_PERI_NSEC * 0.000000001));
  //Serial.println(valOCR1A);
  armTimer();  
}

void loop() {
//Nothing!!! HOW??
}

ISR(TIMER1_COMPA_vect)  {
    sei(); //enable interrupts in an isr. Dangerous!!
 
   //This is the square wave generator
  ++z;  //iterate each loop for square transition delay
  if(z == squareHalfCycSamples){  //when we reach the max then change state
   outState = !outState; //toggle the state of the variable
    z = 0;    //reset half cycle counter
    digitalWrite(squareWaveOut,outState);
  }
  // this line reads the ADCs
  ch1Binary = analogRead(anaInput1);
  ch2Binary = analogRead(anaInput2);
  // and calibrates to volts and prints to the serial port
  scopePinState = digitalRead(scopeSelectPin);
  if( scopePinState == 0) { //run the matlab formatt
    Serial.println(gain1*(ch1Binary*0.00488)-DCOffsetNull1);
    //Serial.print(",");
    Serial.println(gain2*(ch2Binary*0.00488)-DCOffsetNull2);
  }

   if( scopePinState == 1) { //arduino plotter formatt
    Serial.print(gain1*(ch1Binary*0.00488)-DCOffsetNull1);
    Serial.print(",");
    Serial.println(gain2*(ch2Binary*0.00488)-DCOffsetNull2);
  }
} //end of ISR

void armTimer(void){
    TCCR1B = 0;
    TCCR1A = 0;
    // set up timer with prescaler = 64
    TCCR1B |= (1 << CS11) | (1 << CS10);
    // initialize counter
    TCNT1 = 0;
    // set compare match register for 1hz increments
    OCR1A = valOCR1A;// desired interval/(clock int * prescaler) - 1(must be <65536)
    //tunr on CTC mode
    TCCR1B |= (1 << WGM12);
    //enable compare interrupt
    TIMSK1 |= (1 <<OCIE1A);
    // enable overflow interrupt
    //TIMSK1 |= (1 << TOIE1);
    sei();
   // PORTD = (1<<PD2);  //start up square wave gen
}
