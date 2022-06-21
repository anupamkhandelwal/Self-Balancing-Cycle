#include "MPU9250.h"
#include <math.h> 
#include <AFMotor.h>

AF_Stepper motor(48, 2);

double theta;
double alpha_prev = 0;
double theta_prev = 0;
double alpha_prev_2 = 0;
double theta_prev_2 = 0;

double acc_y;
double acc_z;

MPU9250 IMU(Wire,0x68);
int status;


void setup() 
    {
      while(1)
        {
          status = IMU.begin();
          if (status>0)
            {
              break;
            }
          else continue;
        }
      
      motor.setSpeed(10);
      Serial.begin(9600);
    }

void loop() {

  double alpha;
  int stepper_steps;
  String stepper_direction;
  double starttime = millis();
  
  // SENSOR INPUT AND CONTROL EQUATION

  while(millis()-starttime<50)
    {
      IMU.readSensor();
  
      acc_y = IMU.getAccelY_mss();
      acc_z = IMU.getAccelZ_mss();
  
      alpha = atan(acc_y/acc_z);

      theta = (0.1147*(1-0.5*pow(alpha,2))*alpha - 8.33*alpha - 34848*pow((alpha-alpha_prev),2))/(0.1411-0.1147*(1-pow(alpha,2)));
      Serial.print(theta);
      Serial.print(",");
      Serial.println(alpha);
      alpha_prev = alpha;
    }
  
  
  // MOTOR CONTROL
  if(abs(theta)<0.75){
        
    
  if((theta-theta_prev)>=0)
    {
      stepper_steps = (int) abs(theta-theta_prev)*100/PI;    
      motor.step(stepper_steps, FORWARD, DOUBLE); 
              
    }
  else
    { 
      stepper_steps = (int) abs(theta-theta_prev)*100/PI;
      motor.step(stepper_steps, BACKWARD, DOUBLE); 
              
    }

 
  } // STORING PREVIOUS VALUES
  
  theta_prev = theta;

}
