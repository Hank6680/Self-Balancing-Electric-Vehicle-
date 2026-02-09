/* Interrupt timing 5ms timer interrupt */
#include "Balance.h"
extern float positions;


long last_time = 0;
long detect_time = 0;
int left = 0;
int right = 0;
int j = 0;
int k = 0;

void inter()
{
  if (millis() > 20000){
   
    if (analogRead (hall_L_pin) > 210 && millis() - detect_time > 6000){
      digitalWrite(RPin,HIGH);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW); 
      balance_robot.positions = 200;
      rdistance = 0;
      ldistance = 0;
      front = 0; back = 0; j = 1; left = 1; right = 0;
      detect_time = millis();
      left_count++;
    }
    if (analogRead (hall_R_pin) > 210 && millis() - detect_time > 6000){
      digitalWrite(RPin,HIGH);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW); 
      balance_robot.positions = 200;
      rdistance = 0;
      ldistance = 0;
      front = 0; back = 0; j = 1; right = 1; left = 0;
      detect_time = millis();
      right_count++;
    }

    if (analogRead (hall_L_pin) < 210 && millis() - detect_time > 1000 && j == 1 && right == 1){
      yaw_target += 25;
      digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,HIGH);
      balance_robot.positions = 0;
      j = 0;
      //if (abs(left_count - right_count) == 1){yaw_target += 0;}
      //if (abs(left_count - right_count) > 1){yaw_target += 20;}
    }

    if (analogRead (hall_R_pin) < 210 && millis() - detect_time > 1000 && j == 1 && left == 1){
      yaw_target -= 30;
      digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,HIGH);
      balance_robot.positions = 0;
      j = 0;
      //if (abs(left_count - right_count) == 1){yaw_target -= 0;}
      //if (abs(left_count - right_count) > 1){yaw_target -= 20;}
    }

    if (millis() - detect_time > 1000){
      front = 17; back = 0;
      digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,HIGH);
      j = 0;
    }
  } 

  if (distance < 15){
    balance_robot.positions = 500; front = 0; back = 0; 
  }
  /*if (millis() > 35000){
    
    if (analogRead (hall_R_pin) > 210){
      digitalWrite(RPin,HIGH);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW);
      balance_robot.positions = 0;
      front = 0;
      back = 30; 
      detect_time = millis();
      right++;
      j = 0;
      k = 0;
    }
    if (millis() - detect_time > 2000 && right > left){
      if(millis() - last_time > 8000 && analogRead (hall_R_pin) < 210 && millis() - detect_time > 3000 && k == 0){
        yaw_target -= 40;
        back = 0;
        rdistance = 0;
        ldistance = 0;
        balance_robot.positions = 0;
        k = 1;
      }
      back = 0;
      balance_robot.positions = 0;
      if (millis() - detect_time > 10000 && j == 0){
        last_time = millis();
        balance_robot.positions = 0;
        back = 0;
        rdistance = 0;
        ldistance = 0;
        j = 1;
        }   
      }  

    
    if (analogRead (hall_L_pin) > 210){
      digitalWrite(RPin,HIGH);digitalWrite(GPin,HIGH);digitalWrite(BPin,LOW); 
      balance_robot.positions = 0;
      front = 0;
      back = 30;
      detect_time = millis();
      left++;
      j = 0;
      k = 0;
    }
    if (millis() - detect_time > 2000 && right < left){
      if(millis() - last_time > 8000 && analogRead (hall_L_pin) < 210 && millis() - detect_time > 3000 && k == 0){
         yaw_target += 40; 
         balance_robot.positions = 0;
         back = 0;
         rdistance = 0;
         ldistance = 0;
         k = 1; 
      }
      back = 0;
      balance_robot.positions = 0;
      if (millis() - detect_time > 8000 && j == 0){
        last_time = millis();
        balance_robot.positions = 0;
        back = 0;
        rdistance = 0;
        ldistance = 0;
        j = 1;
        }
      }
      
    if (millis() - last_time > 2000 && millis() - detect_time > 8000){
      digitalWrite(RPin,LOW);digitalWrite(GPin,HIGH);digitalWrite(BPin,HIGH);
      front = 20;
      back = 0;
      right = 0;
      left = 0;
    }
  }*/
  sei();         // enable interupts                                
  countpulse(); 

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     // I2C get MPU6050 six axis data ax ay az gx gy gz
  
  // calculate pitch angle using Kalman filter, 
  // roll angle using complementary filter, yaw rate
  kalmanfilter.calculate_angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle,K1);  

  //PD angle loop control
  balance_robot.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.angle_dot; 
  speedcc++;
  if (speedcc >= 8) // 5ms x 8 = 40ms into the speed loop control (25 times/sec)
  {
//    if(millis()>15000){
//      move_distance();
//    }
    Outputs = balance_robot.speedPiOut(kp_speed,ki_speed,front,back,setp0); // forward/backward PI speed controller
    speedcc = 0;
      
    // read from Hall effect sensors
    hall_L = analogRead(hall_L_pin);
    hall_R = analogRead(hall_R_pin);
    /*Serial.print(hall_R);
    Serial.print(" ");
    Serial.println(hall_L);*/
 
    // check if buzzer is require to make a short beep
    if (flag_buzzer == true)
    {
      digitalWrite(buzzerPin, HIGH);
      flag_buzzer = false;
    }
    else
    {
      digitalWrite(buzzerPin, LOW);
    }
  }

  turncount++;
  if (turncount > 2) // 10ms into the yaw turn control loop (100 times/sec)
  {                                
      // yaw turn controller
      turnoutput = balance_robot.turnSpin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);  
      turncount = 0; // reset turncount after 10ms
  } 

  //float position_output = distance_control();
  //balance_robot.distance_control(desired_distance);
  // every 5ms determine motor outputs
  //balance_robot.distance_control();
  balance_robot.pwma(Outputs,turnoutput,kalmanfilter.angle,kalmanfilter.rollangle,turnl,turnr,spinl,spinr,
                     front,back,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB); // motor commands   

  detTime++;
  if(detTime>=50) // 5ms x 50 = 250ms, read distance, heading, and buzzer flag every 0.25s (4 times/sec)
  { 
    distance = Dist.getDistanceCentimeter(); // read distance in centimeters
    Serial.println(distance);
    detTime = 0; // reset detTime
    
    // Get heading data from BNO055
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
    
    yaw_angle = event.orientation.x;  // yaw angle in degrees
    yaw_error = yaw_control(yaw_angle);//-------------------------------------------------------------------------------------------
    

    // Serial.print("yaw_angle = ");Serial.print(yaw_angle);
    // Serial.print("yaw_error = ");Serial.print(yaw_error);
    // Serial.print("yaw_target = ");Serial.print(yaw_target);
    // Serial.print("accumulated_yaw = ");Serial.println(accumulated_yaw);

  }
}
