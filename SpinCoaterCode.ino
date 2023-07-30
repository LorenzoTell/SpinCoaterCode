/*
/*
  SpinCoaterCode

  This code is for control a brushless motor recicled from an old Hard Disk Drive with PID with a pulse input from a transistor-photodiode circuit 
  and a encoded wheel for a Spincoater device, using a comercial ESC controller and a 16x2 display with separated encoder selector an switch input

 
  Created by Lorenzo Tell 10 November 2022
  Modified by Lorenzo Tell 17 may 2023

  The diagrams and 3D files can be found on github
  https://github.com/LorenzoTell/SpinCoaterCode/

*/




#define DEBUG 0
#define SWITCH 7
#define PCINT_PINClock 3
#define PCINT_PINData 4

// LCD PINES
//SDA -> A4
// SCL -> A5
// Contador vueltas
// PIN PWM 9
// PIN SensorVueltas 2

#include <PID_v2.h> 
#include <LiquidCrystal_I2C.h>
#include <Servo.h>


#include <YetAnotherPcInt.h>
volatile int contador = 4500;
volatile int selector=0; //Use this variable to store "steps"
volatile int tinter=0;
int previous_counter = 0;           //Use this variable to store previous "steps" value
int currentStateClock;              //Store the status of the clock pin (HIGH or LOW)
int StateData;                      //Store the status of the data pin (HIGH or LOW)
int lastStateClock;                 //Store the PREVIOUS status of the clock pin (HIGH or LOW)
String currentDir ="";              //Use this to print text 
unsigned long lastButtonPress = 0; 
bool prev;

LiquidCrystal_I2C lcd(0x27, 16, 2); //0x3F//Usar detector de i2c para conocer la ubicación del LCD sino, este no encendera
//Define Variables we'll be connecting to
double Setpoint, Input, Output;



//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 0.25, .7, 0.007, DIRECT); // IN , OUT, SETPOINT, Kp, Ki, KD, Direccion    
										// PID myPID(&Input, &Output, &Setpoint, 0.25, .7, 0.007, DIRECT);  // funcionando en MotorPIDv2
                                        //PID myPID(&Input, &Output, &Setpoint, 0.0730, 18.2, 0.3, DIRECT);
                                      // PID config con funcion cebadora PID myPID(&Input, &Output, &Setpoint, 0.0030, 1.2, 0.16, DIRECT);
                                       // 0.3, 0.77, 0.019
                                       // 0.0300, .75, 0.008 anda bien a 7000 RPM sample de 45ms
                                      //0.0500, 0.8, 0.001 original y andaba bien a 4000RPM
#define PWMpin  9
Servo esc;
int SampleDuration = 40; //30 in Milliseconds
volatile unsigned long timeX = 1;
int PulsesPerRevolution = 2;

int inicio=1;

int a =0;
int b=0;
int c=0;
int rpmLCD =0;
int sw_state=0;
volatile int Counts = 10;
double PulsesPerMinute;
volatile unsigned long LastTime;
volatile int PulseCtrX;
int PulseCtr;
unsigned long Counter;
unsigned long Time;


unsigned long tInicio;
unsigned long tFin;
unsigned long tInicioU;
unsigned long tFinU;
unsigned long tTrans;
bool lcdClear;

/*
  //Self Test
  int RPM = 1;
  double TextRPM = 1;
*/
void pinClock(const char* message, bool pinstate) {
  //Serial.print(message);
  //Serial.print(pinstate ? "HIGH" : "LOW");
 prev=pinstate;
}
void pinData(const char* message, bool pinstate) {
  
  cli(); //We pause interrupts happening before we read pin values
  currentStateClock =   prev;       //Check pin D9 state? Clock
  StateData  =   pinstate;              //Check pin D8 state? Data
  if (currentStateClock != lastStateClock){
    // If "clock" state is different "data" state, the encoder is rotating clockwise
    if (StateData != currentStateClock){
      
      contador=contador-10;
      selector=selector-1;// We increment
      tinter=tinter-1;
       lcdClear=1;
      lastStateClock = currentStateClock;
      // Updates the previous state of the clock with the current state
      sei(); //restart interrupts
    }
    //Else, the encoder is rotating counter-clockwise 
    else {
      
      lcdClear=1;
      contador=contador+10;
      selector=selector+1;// We decrement
      tinter=tinter+1;
      lastStateClock = currentStateClock;         // Updates  previous state of the clock with the current state    
      sei(); //restart interrupts
    } 
  }  
}







void setup() {
	lcd.begin();
	lcd.backlight();
	lcd.clear();
	lcd.setCursor(2,0);
	lcd.print("Spin Coater");
	lcd.setCursor(1,1);
	lcd.print("UNViMe - UNSL");
	delay(3000);
	lcd.clear();
	// note that 1666666.67 = (60 seonds * 1000000 microseconds)microseconds in a minute / (36 / 9) pulses in 1 revolution
	PulsesPerMinute = (60 * 1000000) / (PulsesPerRevolution / Counts);
	//Setpoint = 4000;
	pinMode(2, INPUT_PULLUP); //IRsensor
	pinMode(SWITCH, INPUT); //switch
	Serial.begin(115200);
	esc.attach(9,  1000, 2000);
	esc.write(0);
	delay(5000);
	//Serial.println("Spin Coater - Lorenzo Tell - CEB - UNViMe");
	delay(1000);
	//Digital Pin 2 Set As An Interrupt for tacho.
	attachInterrupt(0, sensorInterrupt, FALLING);
	pinMode(PCINT_PINClock, INPUT_PULLUP);
	PcInt::attachInterrupt(PCINT_PINClock, pinClock, "Clock has changed to ", CHANGE);
	pinMode(PCINT_PINData, INPUT_PULLUP);
	PcInt::attachInterrupt(PCINT_PINData, pinData, "Data has changed to ", CHANGE);
	myPID.SetSampleTime(1);
	myPID.SetOutputLimits(30, 180);
	PulseCtr = 0;
	myPID.SetMode(AUTOMATIC);
	myPID.PrimeIntegral(50);
  //myPID.Compute();
 
 //esc.write(Output);
  
  //analogWrite(PWMpin, Output);
  delay(100);
  //myPID.Compute();
  Setpoint=contador;
}

int nMenuPrincipal=2;
int nMenuAuto;
int nMenuManual;
int selecPrev=0;
char estado=0;
int off=0; 
int tiempo=0;
void loop() {

      
  switch (estado){
  
    case 0: {//Principal////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      selector=constrain(selector, 0,nMenuPrincipal);
      if (lcdClear ==1){lcd.clear();delay(10);lcdClear=0;}
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(1,0-selector);
      if ((0-selector)>=0){
      lcd.print("Automatico     ");}
      lcd.setCursor(1, 1-selector);
      if ((1-selector)>=0){
      lcd.print("Manual         ");}
      lcd.setCursor(1,2-selector);
      lcd.print("Informacion    ");
      
      if (digitalRead(SWITCH)==1){lcd.clear();estado=selector; selector=estado;selecPrev=0; if (selector==0){estado=3;}delay(800);}
      
    
    }
    break;

    case 1:{ //manual/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    int rpmint=Setpoint;
    if(selector==0){
    selector=selector-5;} 
    selector=constrain(selector, 0,2);
    
    if (digitalRead(SWITCH)==1 && selector==0){
         delay(800);
         lcd.setCursor(0, selector);
         lcd.print("*");
         
         while(digitalRead(SWITCH)==0){ // RPM
          if (lcdClear ==1){lcd.clear();delay(10); lcdClear=0;}
         lcd.setCursor(0, 0);
         lcd.print("*");
         lcd.setCursor(1, 0);
         rpmint=contador;
         lcd.print(rpmint);
         lcd.setCursor(5, 0);
         lcd.print("RPM");
         }
         
         Setpoint=rpmint;
         selector=0; 
         delay(800);
         
    }
    if (off==0){ 
    if (lcdClear ==1){lcd.clear();delay(10);lcdClear=0;}
    
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.setCursor(1,0-selector);
      if ((0-selector)>=0){
      lcd.print(rpmint);
      lcd.setCursor(5,0-selector);
      lcd.print("RPM");}
      lcd.setCursor(1, 1-selector);
      if ((1-selector)>=0){
      lcd.print("Empezar        ");}
      lcd.setCursor(1,2-selector);
      lcd.print("Volver         ");
      selecPrev=selector; }
    
 
     if (digitalRead(SWITCH)==1 && selector==1){
        
        off=1;
        rampa(Setpoint);
        RpmPID();
        lcd.setCursor(1, 0);
        rpmint=Input;
        lcd.print(rpmint);
        lcd.setCursor(5,0);
        lcd.print("RPM");
        lcd.setCursor(12, 0);
        lcd.print(tFin/1000);
        lcd.setCursor(15,0);
        lcd.print("S");
      if (digitalRead(SWITCH)==0){lcd.clear(); esc.write(0); inicio=1; off=0; }
        }
      if (digitalRead(SWITCH)==1 && selector ==2){lcd.clear();estado=selector-2; selector=1;selecPrev=0;delay(800);}    
    
    break;
    }
    case 2:{ // informacion////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      lcd.setCursor(0, 0);
      lcd.print(">");
      selector=constrain(selector, 0,2);
      if (lcdClear ==1){lcd.clear();delay(10);lcdClear=0;}
      int subsel = selector;
      lcd.setCursor(1,0-subsel);
      if ((0-subsel)>=0){
      lcd.print("Fabricado por  ");}
      lcd.setCursor(1, 1-subsel);
      if ((1-subsel)>=0){
      lcd.print("Lorenzo Tell   ");}
      lcd.setCursor(1,2-subsel);
      lcd.print("Volver         "); 
      
      
      if (digitalRead(SWITCH)==1){lcd.clear();estado=selector-2; selector=2;selecPrev=0;delay(800);}
    break;
    }

    case 3: {//////////////////AUTOMATICO/////////////////////////////////
    int rpmint=Setpoint;
   if (lcdClear ==1){lcd.clear(); lcdClear=0;}
    selector=constrain(selector, 0,3);
    if (digitalRead(SWITCH)==1 && selector==0){ //RPM
         delay(800);
         lcd.setCursor(0, selector);
         lcd.print("*");
         if (lcdClear ==1){lcd.clear(); lcdClear=0;}
         while(digitalRead(SWITCH)==0){
         if (lcdClear ==1){lcd.clear();delay(10); lcdClear=0;}
         lcd.setCursor(0, 0);
         lcd.print("*");
         lcd.setCursor(1, 0);
         rpmint=contador;
         lcd.print(rpmint);
         lcd.setCursor(5, 0);
         lcd.print("RPM");
         }
         
         Setpoint=rpmint;
         selector=0; selecPrev=0;
         delay(800);
         
    }
    if (digitalRead(SWITCH)==1 && selector==1){ //// TIEMPO
         delay(800);
         lcd.setCursor(0, 1-selector);
         lcd.print("*");
         tinter=0;
         while(digitalRead(SWITCH)==0){
         if (lcdClear ==1){lcd.clear();delay(10); lcdClear=0;}
         lcd.setCursor(0, 0);
         lcd.print("*");
         lcd.setCursor(1, 0);
         tiempo=tinter; 
         lcd.print(tiempo);
         lcd.setCursor(4, 0);
         lcd.print("S    ");
         }
         
         selector=1; selecPrev=0;
         delay(800);
         
    }

    if (off==0){ 
    if (selecPrev != selector){lcd.clear();}
    
      Serial.println(selector);
      lcd.setCursor(0, 0);
      lcd.print(">");
      
      if ((0-selector)>=0){
        lcd.setCursor(1,0-selector);
        lcd.print(rpmint);
        lcd.setCursor(5,0-selector);
        lcd.print("RPM");}
      
      
      if ((1-selector)>=0){
        lcd.setCursor(1, 1-selector);
        lcd.print(tiempo);
        lcd.setCursor(4, 1-selector);
        lcd.print("S    ");}
      
      
      if ((2-selector)>=0 /*&& 2-selector <= 2*/){
        lcd.setCursor(1,2-selector);
        lcd.print("Empezar        ");}
        
      
      if ((3-selector)>=0 && 3-selector < 2){
        lcd.setCursor(1,3-selector);
        lcd.print("Volver         ");}
     
      
      selecPrev=selector; }
    
 
     if (digitalRead(SWITCH)==1 && selector==2){ //////////////// TOCASTE EMPEZAR
        lcd.clear(); estado=4; selector=0; selecPrev=0; delay(800);
        
        }
        
      if (digitalRead(SWITCH)==1 && selector ==3){lcd.clear();estado=selector-3; selector=0;selecPrev=0;delay(800);}  // TOCASTE VOLVER
      
      
      break;}
      case 4: /// AUTOMATICO RUTINA
      {
        
        rampa(Setpoint);
        RpmPID();
        lcd.setCursor(1, 0);
        int rpmint=Input;
        lcd.print(rpmint);
        lcd.setCursor(5,0);
        lcd.print("RPM");
        lcd.setCursor(12, 0);
        lcd.print(tFin/1000);
        lcd.setCursor(15,0);
        lcd.print("S");
        
      if (digitalRead(SWITCH)==1 or tiempo == tFin/1000){  esc.write(0); lcd.clear(); inicio=1; off=0; estado=3; selector=0;selecPrev=0;delay(800); }
        
        
        break;}
  }
  
  
  }
  








/*    
   //tInicioU=micros();
  //ACCIONAMIENTO
  if (digitalRead(SWITCH)==1){
    rampa(Setpoint);
		RpmPID();
    //tFinU=micros()-tInicioU;
    //Serial.print("tiempo:");
    //Serial.print(tFinU);
    //delay(10000);
		lcd.setCursor(0, 0);
		lcd.print(Input);
        lcd.setCursor(7,0);
        lcd.print("RPM");
        lcd.setCursor(0, 1);
        lcd.print(tFin/1000);
        lcd.setCursor(3,1);
        lcd.print("S");
    if (digitalRead(SWITCH)==0){esc.write(0);}
  } else {
     esc.write(0);
     Setpoint=contador;
    lcd.setCursor(0, 0);
    lcd.print(Setpoint);
    lcd.setCursor(7,0);
    lcd.print("RPM");
     inicio=1;
    }
    
 */   




void rampa (int Setpoint){
  //esc.write(70);
  
  
 /* if (inicio ==1 && Setpoint<=4100){
    lcd.clear();
    tInicio=millis();
    esc.write(180);
    delay(400);
    inicio=0;
    }*/
if (inicio ==1 && Setpoint<=3000 && Setpoint>=500 ){
  
  double kpv=0.7; //0.7
  double kiv=1.7; //0.3
  double kdv=0.05; //0.01
  myPID.SetTunings(kpv, kiv, kdv);
  double a,b,c;
  a=myPID.GetKp();  // These functions query the pid for interal values.
  b=myPID.GetKi();
  c=myPID.GetKd();              
  Serial.print(a);
  Serial.print(b);
  Serial.print(c);
    lcd.clear();
    tInicio=millis();
    esc.write(180);
    delay(10);
    inicio=0;
    }
  if (inicio ==1 && Setpoint<=4100 && Setpoint>=3001 ){
	double kpv=0.95; //0.25
	double kiv=.9; //0.9
	double kdv=0.01; //0.007
	myPID.SetTunings(kpv, kiv, kdv);
	double a,b,c;
	a=myPID.GetKp();	// These functions query the pid for interal values.
	b=myPID.GetKi();
	c=myPID.GetKd();						  
	Serial.print(a);
	Serial.print(b);
	Serial.print(c);
    lcd.clear();
    tInicio=millis();
    esc.write(180);
    delay(100);
    inicio=0;
    }
  
  if (inicio==1 && ( Setpoint < 8000 && Setpoint > 4100)){
	double kpv=0.25;
	double kiv=1.5;
	double kdv=0.007;
	myPID.SetTunings(kpv, kiv, kdv);
	double a,b,c;
	a=myPID.GetKp();	// These functions query the pid for interal values.
	b=myPID.GetKi();
	c=myPID.GetKd();						  
	Serial.print(a);
	Serial.print(b);
	Serial.print(c);
    tInicio=millis();
    int cebador = Setpoint*0.7;
	
    int rpm;
    esc.write(180);
    delay(2000);
    //esc.write(0);
    //delay(200);
    //rpm=getRPM();
    inicio=0;
    int estado = 1;

   while (estado==1){
	
	Serial.println(rpm);
	  lcd.clear();
	  lcd.setCursor(0, 0);
	  lcd.print(rpm);
    lcd.setCursor(5,0);
    lcd.print("RPM");
    rpm = getRPM();
    if (cebador < rpm){estado=0;lcd.clear(); break;}
    esc.write(180);
    delay(800);
	rpm = getRPM();
    if (cebador < rpm){estado=0;lcd.clear();break;}
    esc.write(0);
    delay(10);
    rpm = getRPM();
    if (cebador < rpm){estado=0;lcd.clear();break;}
    esc.write(180);
    delay(800);
	
    if (cebador < rpm){estado=0;lcd.clear();break;}
    }
  }
  
  if (inicio==1 && Setpoint >= 8000){
    tInicio=millis();
	
    int cebador = Setpoint*0.95;
    int rpm;
    esc.write(180);
    delay(5000);
    //esc.write(0);
    //delay(200);
    rpm=getRPM();
    inicio=0;
    int estado = 1;

   while (estado==1){
        Serial.println("MODO PULENTA");
		Serial.println(rpm);
        lcd.clear();
        lcd.setCursor(0, 0);
		lcd.print(rpm);
        lcd.setCursor(6,0);
        lcd.print("RPM");
        rpm=getRPM();
         if (cebador < rpm){estado=0;lcd.clear(); break;}
			esc.write(180);
			delay(300);
			rpm=getRPM();
		if (cebador < rpm){estado=0;lcd.clear();break;}
			esc.write(0);
			delay(10);
			rpm=getRPM();
		if (cebador < rpm){estado=0;lcd.clear();break;}
			esc.write(180);
			delay(700);
			rpm=getRPM();
		if (cebador < rpm){estado=0;lcd.clear();break;}
    }
   
    
  }
  inicio=0;
  
  inicio=0;
}

/*
  void SelfTest(){
  static unsigned long TestTimer;
  static bool done = false;
  static unsigned long TestTime;
  if(done)return;
  if ((unsigned long)(micros() - TestTimer) >= TestTime) { // rpm test
    TestTimer += TestTime;// we want an exact duration for our tests
    TestTime = (60000000 / (PulsesPerRevolution * RPM));// we are one pulse behind so calculate next delay
    sensorInterrupt();
    if (RPM <= 850) {
      TextRPM += .04;
      RPM = (int) TextRPM;
    }
    else done = true;
  }

  }
*/

// New version of sensorInterrupt
void sensorInterrupt(){
  static int Ctr;
  unsigned long Time;
  Ctr++;
  if (Ctr >= Counts) { // so we are taking an average of "Counts" readings to use in our calculations
    Time = micros();
    timeX += (Time - LastTime); // this time is accumulative ovrer those "Counts" readings
    LastTime = Time;
    PulseCtrX ++; // will usually be 1 unless something else delays the sample
    Ctr = 0;
  }
}

void sensorInterruptX(){
  static int Ctr;
  unsigned long Time;
  Ctr++;
  if (Ctr >= Counts) { // so we are taking an average of "Counts" readings to use in our calculations
    Time = micros();
    timeX += (Time - LastTime); // this time is accumulative ovrer those "Counts" readings
    LastTime = Time;
    PulseCtrX ++; // will usually be 1 unless something else delays the sample
    Ctr = 0;
  }
}

const unsigned long sampleTime = 1000;
const int maxRPM = 1260;                  // maximum RPM for LCD Bar
int rpmMaximum = 0;

int getRPM(){
  int count = 0;
  boolean countFlag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  while (currentTime <= sampleTime)
  {
    if (digitalRead(2) == HIGH)
    {
      countFlag = HIGH;
    }
    if (digitalRead(2) == LOW && countFlag == HIGH)
    {
      count++;
      countFlag=LOW;
    }
    currentTime = millis() - startTime;
  }
  int countRpm = int(30000/float(sampleTime))*count;
  return countRpm;
}

void RpmPID(){
  if (!PulseCtrX) return; // << Added lets not stop interrupts unless we know we are ready (keep other code happy).
  cli ();         // clear interrupts flag
  Time = timeX;   // Make a copy so if an interrupt occurs timeX can be altered and not affect the results.
  timeX = 0;
  PulseCtr = PulseCtrX;
  PulseCtrX = 0;
  sei ();         // set interrupts flag
  if (PulseCtr > 0) {
    Input =  (double) (PulsesPerMinute /  (double)(( (unsigned long)Time ) *  (unsigned long)PulseCtr)); // double has more percision
    //   PulseCtr = 0; // set pulse Ctr to zero
    AverageCapture(Input);
    //debug();
    myPID.Compute();
    esc.write(Output);
    //analogWrite(PWMpin, Output);
    //
    if (Time > ((SampleDuration + 1) * 1000))Counts--;
    if (Time < ((SampleDuration - 1) * 1000))Counts++;
    Counts = constrain(Counts, PulsesPerRevolution * .1, PulsesPerRevolution * 4);
    Time = 0; // set time to zero to wait for the next rpm trigger.
    Counter += PulseCtr;
    PulseCtr = 0; // set pulse Ctr to zero
    PulsesPerMinute = (60.0 * 1000000.0) / (double)((double)PulsesPerRevolution / (double)Counts);
    tFin=millis()-tInicio;
    
  }
}

void RpmInter(){
  if (!PulseCtrX) return; // << Added lets not stop interrupts unless we know we are ready (keep other code happy).
  cli ();         // clear interrupts flag
  Time = timeX;   // Make a copy so if an interrupt occurs timeX can be altered and not affect the results.
  timeX = 0;
  PulseCtr = PulseCtrX;
  PulseCtrX = 0;
  sei ();         // set interrupts flag
  if (PulseCtr > 0) {
    Input =  (double) (PulsesPerMinute /  (double)(( (unsigned long)Time ) *  (unsigned long)PulseCtr)); // double has more percision
    //   PulseCtr = 0; // set pulse Ctr to zero
    AverageCapture(Input);
    //debug();
    //analogWrite(PWMpin, Output);
    //
    if (Time > ((SampleDuration + 1) * 1000))Counts--;
    if (Time < ((SampleDuration - 1) * 1000))Counts++;
    Counts = constrain(Counts, PulsesPerRevolution * .1, PulsesPerRevolution * 4);
    Time = 0; // set time to zero to wait for the next rpm trigger.
    Counter += PulseCtr;
    PulseCtr = 0; // set pulse Ctr to zero
    PulsesPerMinute = (60.0 * 1000000.0) / (double)((double)PulsesPerRevolution / (double)Counts);
    tFin=millis()-tInicio;
    
    
  }
}


float AvgArray[100];
int Readings = 0;
void AverageCapture(float in) {
  static int Position = 0;
  AvgArray[Position] = in;
  Position++;
  Readings++;
  Readings = min (100, Readings); // 100 readings 0-99;
  if (Position >= 100)Position = 0;//99 spots
}
float AverageValue() {
  float Total = 0;
  float Average;
  if (!Readings)return (0.0);
  for (int Position = 0; Position < Readings; Position++) {
    Total += AvgArray[Position];
  }
  Average = Total / Readings;
  return (Average);
}
