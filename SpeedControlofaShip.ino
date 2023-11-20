/*If the voltage signal from the potentiometer is 0V, the duty cycle of
the PWM signal on pin 10 should be 0% and the duty cycle of the
PWM signal on pin 9 should be 100 %. This corresponds to full speed
astern

If the voltage signal from the potentiometer is 5V, the duty cycle of
the PWM signal on pin 10 should be 100% and the duty cycle of the
PWM signal on pin 9 should be 0 %. This corresponds to full speed
ahead (forwards)

If the voltage signal from the potentiometer is 2.5V, the duty cycle of
the PWM signal on pin 10 should be 0% and the duty cycle of the
PWM signal on pin 9 should be 0 %. This corresponds to stop.

The duty cycle on the two pins should change linearly with the voltage
on the potentiometer between these points.

The PWM signal should have a frequency of approximately 2 kHz.

The value of the potentiometer should be read with approximately20 Hz.

*/

#include "SpeedControlShip.h"

float PotentiometerValue  = 0;
int ControlCommand        = 0;
int timer0account         = 39;
//PWM TEST
float PWMTEST             = 0;

/*
struct ShipControlType{

  float EngineOrderTelegraph;
  bool ShipDirection; //0->Ahead , 1->Astern
  int  ForwardSpeed ;

};
ShipControlType* ShipControl = (ShipControlType*)malloc(sizeof(ShipControlType));
*/
/*Timer0 
The value of the potentiometer should be read with approximately 20 Hz (50ms)
Time of interrupt = TicksCount * TickTime

Time of interrupt = (Prescaler * TicksCount)/CLKfreq

Time of interrupt = (1*255)/16M =4.1ms (12times approximately 50ms)

In COMPARE model,OCR0A = [16MHz/(Clock Division*hz)]-1
                 hz = 16MHz/[Clock Division*(OCR0A+1)]

*/
ISR(TIMER0_COMPA_vect){
  //TCNT0 = 255;
  //Serial.print("in timer0");
  timer0account++;
  if(timer0account == 40){
      PotentiometerValue = analogRead(A0);
      ControlCommand = ShipCommandProcess(PotentiometerValue);
      //Serial.print(PotentiometerValue);
      timer0account = 0;
  }
  else{
     /* Serial.print("Timer0 wait \n");
      Serial.print(timer0account);
      Serial.print("\n ");
      */
  }  
}

void setup() {
  // put your setup code here, to run once:
  pinMode(ForwardOutputPin,OUTPUT);
  pinMode(AsternOutputPin,OUTPUT);

  Serial.begin(9600);
  noInterrupts();
    //-----------------------------Timer1 Fast PWM model(16MHz / 32 / 256 = 7:8125 kHz)
  /*
  TCCR1A = 0;
  TCCR1B = 0;
  //TCNT1 = 65536;
 // TCCR1B |= (1<<CS12);
  //TIMSK1 |= (1<<TOIE1);
  // Here, we change the mode of operation
  // from "PWM, Phase Correct, 8bit" to "fast PWM, 8bit".
  // This increases the PWM frequency with a factor of 2.
  TCCR1A |= (1 << WGM10);
  TCCR1A &= ~(1 << WGM11);
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << WGM13);
  // Here, we change the pre-scalar from 64 to 8,
  // increasing PWM frequency with a factor of 8
  TCCR1B &= ~(1 << CS10);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS12);
  */
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21) | _BV(CS20);
  //-----------------------------Timer0 OVF interrpution which 20Hz collect input 
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  OCR0A = 255;
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS00);
  TIMSK0 |= (1 << OCIE0A);
  interrupts();

}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch (ControlCommand) {

    case 1:
      //Astern, PIN 10(100% PWM), PIN 9(0% PWM)
      analogWrite(ForwardOutputPin,127);
      //analogWrite(AsternOutputPin, 160);
      
      
      pinMode(ForwardOutputPin,OUTPUT);
      Serial.print("Execuated Astern \n");
      break;

    case 2:
    // Standby, PIN 10(0% PWM), PIN 9(0% PWM)
      analogWrite(ForwardOutputPin,Stop);
      analogWrite(AsternOutputPin, Stop);
      Serial.print("Execuated Standby \n");
      break;

    case 3:
    // Forward, PIN 10(0% PWM), PIN 9(100% PWM)
      analogWrite(ForwardOutputPin,FullSpeed);
      analogWrite(AsternOutputPin, Stop);
      Serial.print("Execuated  Forward\n");
      break;

    case 4:
    // Linear
      analogWrite(ForwardOutputPin,FullSpeed);
      analogWrite(AsternOutputPin, Stop);
      Serial.print("Execuated Linear \n");
      break;

    default:
    // Stop or Initial 
      analogWrite(ForwardOutputPin,Stop);
      analogWrite(AsternOutputPin, Stop);
      Serial.print("Execuated Stop or Initail \n");
      break;
  }
}
int ShipCommandProcess(float PotentiometerValue1){

      int ControlCommand1;
      if(PotentiometerValue1 == 0){
        ControlCommand1 = 1;           //Astern, PIN 10(100% PWM), PIN 9(0% PWM)
      }
      else if(PotentiometerValue1 >= 520 && PotentiometerValue1 <= 550){
        ControlCommand1 = 2;           // Standby, PIN 10(0% PWM), PIN 9(0% PWM)
      }
      else if(PotentiometerValue1 == 1023 ){ 
        ControlCommand1 = 3;           // Forward, PIN 10(0% PWM), PIN 9(100% PWM)
      }
      else if((PotentiometerValue1 >0 && PotentiometerValue1 < 512)||(PotentiometerValue1 >512 && PotentiometerValue1 < 1023)){
        ControlCommand1 = 4 ;           // Linear 
      }
      else{
        ControlCommand1 = 0;           // Stop or Initial 
      }      
      
      Serial.print("Value: ");
      Serial.print(PotentiometerValue1);
      Serial.print(" ,ControlCommand: ");
      Serial.print(ControlCommand);
      Serial.print("\n ");
      
      return ControlCommand1;
}