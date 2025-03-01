#include <Wire.h> //For i2c communication
#include <LiquidCrystal_I2C.h> // For LCD Display
#include <Servo.h> //For the mini-servo

LiquidCrystal_I2C lcd(0x27, 16, 2); /* Initialize LCD and Set the LCD address 
                                        to 0x27 for a 16 chars and 2 line display */

Servo bottleGrip;                   //Variable assign of servo

/*------ALL PIN DECLARATION------*/

int bottle_detect     = A0; //To detect the presence of bottle
int cap_detect        = A1; //To detect the presence of cap
int limit_X_start     = A2; //To detect the INITIAL position of X-axis Stepper
int limit_X_end       = A3; //To detect the FINAL position of X-axis Stepper
int limit_Y_start     = A4; //To detect the INITIAL position of Y-axis Stepper

const int direction_X = 2; //Direction Pin of X-axis Stepper
const int step_X      = 3; //Step Pin of X-axis Stepper 

const int direction_Y = 4; //Direction Pin of Y-axis Stepper
const int step_Y      = 5; //Step Pin of Y-axis Stepper

int conveyor          = 6; //Relay Signal to switch conveyor
int pump              = 7; //Relay Signal to switch pump
int flow_sensor       = 8; //To detect pulses and therefore calculate total volume passed 
int bottleGripPin     = 9; //(~PWM pin) - To control the precise direction of Servo
int solenoid          = 10;//Relay Signal to switch solenoid
int capping_motor     = 11;//Relay Signal to switch 25GA 370 Motor

long pulse;   //Variables required for
float volume; //the working of flow-Sensor.

void setup() {
  
  // 16x2 Liquid Crystal Display
  lcd.init();      //Initialize the LCD
  lcd.backlight(); //Turn on the LCD screen backlight

  
  //NEMA 17, Bottle Displacer X-Axis
  pinMode(step_X, OUTPUT);
  pinMode(direction_X, OUTPUT);
  

  //NEMA 17, Botlle Capping Y-Axis
  pinMode(step_Y, OUTPUT);
  pinMode(direction_Y, OUTPUT);
  
   
  //Servo SG 90
  bottleGrip.attach(bottleGripPin); //Drive servo's via PWM pins


  //Input-Output Operations(Boolean Equipments)
  pinMode(bottle_detect, INPUT);   
  pinMode(conveyor, OUTPUT);  
  pinMode(pump, OUTPUT);  
  pinMode(flow_sensor, INPUT);   
    attachInterrupt(digitalPinToInterrupt(flow_sensor), increase, RISING);
  pinMode(solenoid, OUTPUT); 
  pinMode(cap_detect, INPUT);  
  pinMode(capping_motor, OUTPUT); 
  pinMode(limit_X_start, INPUT);  
  pinMode(limit_X_end, INPUT);  
  pinMode(limit_Y_start, INPUT);  
}


void increase(){       //To count the no. of pulses(Flow_Sensor)
  pulse++;
  }


void loop() {
  
  /*Checking required if Stepper X-axis is in postion
   * Limit Switch X axis start is HIGH
   * Only then further steps can be proceeded
   */

   
  /*Checking required if Stepper Y-axis is in postion
   * Limit Switch Y axis start is HIGH
   * Only then further steps can be proceeded
   */
  
  
  //Conveyer 385 DC
  while(digitalRead(bottle_detect) != HIGH){
    digitalWrite(conveyor, HIGH);
    }
    digitalWrite(conveyor, LOW);
    
   //Bottle at initial Position (LCD)
    lcd.setCursor(0, 0);
    lcd.print("  Bottle Found  ");
    delay(400);
    lcd.clear();

    //Servo Locking(LCD)
    lcd.setCursor(0, 0);
    lcd.print("  Servo Locks  ");
    delay(100);
    bottleGrip.write(150);
    delay(100);
    
    //Bottle Filling Mechanism and LCD
    lcd.setCursor(0, 1);
    lcd.print("  Filling....  ");
    delay(100);
    volume = 0.1856 * pulse;        //Calculated before using another code.
    
    if(volume <= 400){              //Fill till 400mL.
      digitalWrite(solenoid, HIGH); //Solenoid OPEN.
      delay(5);                     //5ms delay
      digitalWrite(pump, HIGH);     //Pump ON.
      }
    
    digitalWrite(pump, LOW);        //Pump OFF.
    delay(5);                       //5ms delay
    digitalWrite(solenoid, LOW);    //Solenoid CLOSE.
    volume = 0.0;                   //Volume set to default value.
    
    lcd.clear();
    delay(50);

    //Stepper X-Axis


    
    //Cap Detector
      //if(true) move ahead X-axis
      //else break from loop & RESET

      
    //Stepper Y-axis
      //Capping DC 25GA 370
    //Stepper Y-axis
      //Limit Y Start


    //Stepper X-axis
      //Limit X-axis End
}
