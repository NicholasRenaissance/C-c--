#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

#define   PWMOut                3
#define   ReadPin               10
#define   AsternOutputPin       3        //PIN 9
#define   ForwardOutputPin      11       //PIN10
#define            F_EN       6
#define            R_EN       5

float EncoderPeriod();

float PotentiometerValue  = 0;
int ControlCommand        = 0;
int timer0account         = 12;
float volt;
float Encoder_Frequency;
char StringEncoderPeriod[50];
int i =12;
float j =3.14;

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, A5, A4);

ISR(TIMER0_COMPA_vect){
  //TCNT0 = 255;
  //Serial.print("in timer0");
  
  timer0account++;
  
  if(timer0account == 40){

      PotentiometerValue = analogRead(A0);
      volt = (PotentiometerValue*5.0)/1023.0;
      //Serial.print(volt);
      //Serial.print("\n");
      timer0account = 0;

  }
}

void setup() {

  pinMode(ReadPin,INPUT);

  //analogWrite(AsternOutputPin,254);
  pinMode(ForwardOutputPin,OUTPUT);
  pinMode(AsternOutputPin,OUTPUT);
  pinMode(F_EN,OUTPUT);
  pinMode(R_EN,OUTPUT);
  digitalWrite(F_EN,HIGH); // Enable F_EN
  digitalWrite(R_EN,HIGH); // ENable R_EN
  Serial.begin(9600);
  u8g2.begin();
  noInterrupts();
  //------------------------------------------Timer 2 FastPWM Model
                  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
                  TCCR2B = _BV(CS21) | _BV(CS20);
  //-----------------------------Timer0 OVF interrpution which 20Hz collect input 
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0 = 0;
  OCR0A = 255;
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS02);
  TIMSK0 |= (1 << OCIE0A);
  interrupts();

}

void loop(){

  //unsigned long durationHigh;
  //unsigned long durationLow;


  Encoder_Frequency = EncoderPeriod();
  //OLEDdisplay();
  PotentiometerProcess();
  OLEDdisplay();
  //u8g2.print(Encoder_Frequency);
  Serial.print("Encoder Period-->");
  Serial.print(Encoder_Frequency,2);
  Serial.print("ms \n");

}

void PotentiometerProcess(){

    if((PotentiometerValue >0 && PotentiometerValue < 550)){

            analogWrite(AsternOutputPin,0);
            analogWrite(ForwardOutputPin,0);
            analogWrite(AsternOutputPin, (-volt+2.5)*255/2.5);           // Linear below 2.5V   analogWrite(F_PWM, ((-volt+2.5)*255/2.5));

      }
      else if((PotentiometerValue >550 && PotentiometerValue < 1023)){

            analogWrite(AsternOutputPin,0);
            analogWrite(ForwardOutputPin,0);
            analogWrite(ForwardOutputPin,(volt-2.5)*255/2.5);          // Linear over 2.5V  analogWrite(R_PWM, ((volt-2.5)*255/2.5));
      }
}

float EncoderPeriod(){

  unsigned long durationHigh;
  unsigned long durationLow;
  float PWM_Period;
  durationHigh = pulseIn(ReadPin,HIGH);
  durationLow = pulseIn(ReadPin,LOW);
  PWM_Period = durationHigh + durationLow;
  /*
  Serial.print("durationHigh =");
  Serial.print(durationHigh );
  Serial.print("  durationLow =");
  Serial.print(durationLow);
  Serial.print("PWM_Period =");
  Serial.print(PWM_Period );
  Serial.print("\n");
  */
  PWM_Period = PWM_Period/1000;
  //Serial.print("frequency =");
  //Serial.print(frequency);
  return PWM_Period;

}


void OLEDdisplay(){
    sprintf(StringEncoderPeriod,"%f ms",Encoder_Frequency); 
    u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,15,"Period=");
    //u8g2.drawStr(0,45,StringEncoderPeriod);
    //u8g2.print(Encoder_Frequency);
    u8g2.setCursor(0,45);
    u8g2.print(Encoder_Frequency);
    u8g2.drawStr(45,45,"ms");

  } while ( u8g2.nextPage() );

}


