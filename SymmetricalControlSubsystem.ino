#include <PID_v1.h> //PID Calculator Library
double setpoint;                            //PID to return to this value
double leftSensor, rightSensor;
double Steer;                               //Angle between 65 and 115
double Motor;                               //PWM signal between 30 and 85
double error;                               //Difference between left ad right sensor outputs
double PIDOUTPUT;                           //PID response to error
double p, delta;                            //Distance from the left sensor ,
float ran;                                  //Random slight deviation from current position
double Kps = 1, Kis = 0.075, Kds = 0.05;  //K Values for PID
unsigned long previousMillis = 0;           //The last time which the loop excecuted.
const long interval = 100;                  //How often the loop excecutes in milliseconds.
PID MyPID(&error, &PIDOUTPUT, &setpoint, Kps, Kis, Kds, DIRECT); 
                                            //&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction
                                            
void setup() {                              //Executes once at initialisation
  Serial.begin(9600);
  p = 0;                                    //Initial distance from left sensor
  setpoint = 0;                             //Aiming for 0 error
  Steer = 90;
  Motor = 85;
                                            //Initialise steer and motor at best values - going straight at full speed.
  MyPID.SetOutputLimits(-250, 250);         //Ensure the PID output is within this range.
  MyPID.SetMode(AUTOMATIC);                 //Turns the PID on.
}

void loop() {
  unsigned long currentMillis = millis();           //Gets current time in seconds
  if (currentMillis - previousMillis >= interval) { //Run this every 0.1 seconds.
    ran = random(-500,500)/2000.0;
    //ran = 0;
    previousMillis = currentMillis;                 //Sets the new last time to current time.
    leftSensor = 3.3 * (1024 / 5) * exp(-p);
    rightSensor =  3.3 * (1024 / 5) * exp(-(5 - p));
                                                    //Calculate reading of left and right sensors.
    error = rightSensor - leftSensor;               //find the error by taking the difference between sensors.
    MyPID.Compute();                                //(Kp * error) + (Ki * ∫error) + (Kd * derror/dT)​
    Steer = 90 - 0.1 * (PIDOUTPUT) ;                //Turns the buggy so the center alligns with the current carrying wire.
    Motor = 85 - (2 * (abs(Steer - 90)));           //Reduces the motor PWM by up to 50 dependent on how straight the steering is.
    Steer = constrain(Steer, 65, 115);
    Motor = constrain(Motor, 30, 85);
                                                    //Keeps steering and speed within its limits.
    delta = Motor * (0.1 / 3) * (cos(Steer * 0.0174533)) + ran;
                                                    //Calculate how much P will deviate by depending on the steering and speed.
    p += delta;
    p = constrain(p, 0, 5);                         //Keeps P within the limits of the sensor.                       
    Serial.print(error);
    Serial.print("  ");
    Serial.print(PIDOUTPUT);                      
    Serial.print("  ");
    Serial.println(p);                              //Plot Error, PIDOUTPUT and p.

  }
}
