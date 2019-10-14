//------------------------------------------------------
// Ejemplo receptor Bluetooth y control 0-100 PWM
// 
//
// Autor: Celestí Capell
//------------------------------------------------------

#include <SoftwareSerial.h>
#include <Servo.h>
#include <AFMotor.h>

#define LINE_BUFFER_LENGTH 512

char STEP = MICROSTEP ;

// Servo position for Up and Down 
const int penZUp = 120;
const int penZDown =50;

// Servo on PWM pin 10
const int penServoPin =10 ;

// Should be right for DVD steppers, but is not too important here
const int stepsPerRevolution = 256; 

// create servo object to control a servo 
Servo penServo;  

// Initialize steppers for X- and Y-axis using this Arduino pins for the L293D H-bridge
AF_Stepper myStepperY(stepsPerRevolution,1);            
AF_Stepper myStepperX(stepsPerRevolution,2);  

/* Structures, global variables    */
struct point { 
  float x; 
  float y; 
  float z; 
};

// Current position of plothead
struct point actuatorPos;

//  Drawing settings, should be OK
float StepInc = 1;
int StepDelay = 0;
int LineDelay =0;
int penDelay = 50;

// Motor steps to go 1 millimeter.
// Use test sketch to go 100 steps. Measure the length of line. 
// Calculate steps per mm. Enter here.
float StepsPerMillimeterX = 7.0;
float StepsPerMillimeterY = 7.0;

// Drawing robot limits, in mm
// OK to start with. Could go up to 50 mm if calibrated well. 
float Xmin = 0;
float Xmax = 40;
float Ymin = 0;
float Ymax = 40;
float Zmin = 0;
float Zmax = 1;

float Xpos = Xmin;
float Ypos = Ymin;
float Zpos = Zmax; 

float Xant = Xmin;
float Yant = Ymin;

SoftwareSerial mySerial(2, 3); // RX, TX



String frase; // Para almacenar varios caracteres
int valor;
boolean tocabajar;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  delay(100);
  serialEvent(); // Comprovar si tenemos datos entrantes
  analiza_datos();
}

void serialEvent() {
  while (mySerial.available()) {
    if (mySerial.available() > 0) { // Mirar si han entrado datos
      char letra = mySerial.read();
      frase += letra; // Almacena los datos creando una frase
    }
  }
}

void analiza_datos() {
 
  Serial.println(frase);
  if (frase.length() > 0) {// Si hay datos, miramos su valor
    int i=0;
    while (i<frase.length())
    {
      switch(frase[i])
      {
        case 'R':
          if (frase.substring(i,i+3)=="RES")
          {
            
            mySerial.print("ANC"+String(Xmax)+"ALT"+String(Ymax));
            Serial.println("ANC"+String(Xmax)+"ALT"+String(Ymax));
            i=i+3;
          }
          else
            i=i+1;
          
          break;
        case 'P':
          i=i+3;
          Xant = Xmin;
          Yant = Ymin;
          Xpos = Xmin;
          Ypos = Ymin;
          penServo.attach(penServoPin);
          penServo.write(penZUp);
          delay(100);
          penUp();
          // Decrease if necessary
          myStepperX.setSpeed(20);
        
          myStepperY.setSpeed(20);  
          myStepperX.step(256, BACKWARD);
          myStepperY.step(256, BACKWARD);
          penUp();
          break;
        case 'L':
          i=i+2;
          tocabajar=true;
          penUp();
          break;
        case 'X':
          if (frase.indexOf('X',i+1)==-1)
          {
            String cadx=frase.substring(i+1,frase.indexOf('Y',i));
            String cady=frase.substring(frase.indexOf('Y',i)+1,frase.indexOf("EOP",i+1));
            drawLine(Xant+cadx.toFloat(),Yant+cady.toFloat());
            i=frase.indexOf("EOP",i+1)+3;
            frase = "";
            mySerial.print("IOK");

          }
          else
          {
            String cadx=frase.substring(i+1,frase.indexOf('Y',i));
            String cady=frase.substring(frase.indexOf('Y',i)+1,frase.indexOf('X',i+1));
            drawLine(Xant+cadx.toFloat(),Yant+cady.toFloat());
            i=frase.indexOf('X',i+1);
          }
          if (tocabajar)
          {
            penDown();
            tocabajar=false;
          }
          break;
      }
    }
  }
  frase = ""; // Borrar buffer...
}

void penUp() { 
  penServo.write(penZUp); 
  delay(penDelay); 
  Zpos=Zmax; 
  digitalWrite(15, LOW);
    digitalWrite(16, HIGH);

    
  
}
//  Lowers pen
void penDown() { 
  penServo.write(penZDown); 
  delay(penDelay); 
  Zpos=Zmin; 
  digitalWrite(15, HIGH);
    digitalWrite(16, LOW);

}

void drawLine(float x1, float y1) {

  

  //  Bring instructions within limits
  if (x1 >= Xmax) { 
    x1 = Xmax; 
  }
  if (x1 <= Xmin) { 
    x1 = Xmin; 
  }
  if (y1 >= Ymax) { 
    y1 = Ymax; 
  }
  if (y1 <= Ymin) { 
    y1 = Ymin; 
  }

  
  //  Convert coordinates to steps
  Xant=x1;
  Yant=y1;
  x1 = (int)(x1*StepsPerMillimeterX);
  y1 = (int)(y1*StepsPerMillimeterY);
  float x0 = Xpos;
  float y0 = Ypos;
  //  Let's find out the change for the coordinates
  long dx = abs(x1-x0);
  long dy = abs(y1-y0);
  int sx = x0<=x1 ? StepInc : -StepInc;
  int sy = y0<=y1 ? StepInc : -StepInc;

  long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      if(sx<0){
         myStepperX.step(1,BACKWARD);
      }
      else {
         myStepperX.step(1,FORWARD);
      }
      over+=dy;
      if (over>=dx) {
        over-=dx;
        if(sy<0){
           myStepperY.step(1,BACKWARD);
        }
        else {
           myStepperY.step(1,FORWARD);
        }
      }
    delay(StepDelay);
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      if(sy<0){
         myStepperY.step(1,BACKWARD);
      }
      else {
         myStepperY.step(1,FORWARD);
      }
      over+=dx;
      if (over>=dy) {
        over-=dy;
        if(sx<0){
           myStepperX.step(1,BACKWARD);
        }
        else {
           myStepperX.step(1,FORWARD);
        }
      }
      delay(StepDelay);
    }    
  }

  

  //  Delay before any next lines are submitted
  delay(LineDelay);
  //  Update the positions
  Xpos = x1;
  Ypos = y1;
}
